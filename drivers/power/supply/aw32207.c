/*
 * aw32207.c   aw32207 charge module
 *
 * Copyright (c) 2018 AWINIC Technology CO., LTD
 *
 *  Author: Joseph <zhangzetao@awinic.com.cn>
 *
 * Copyright (c) 2019 MediaTek Inc.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/slab.h>
#include <linux/pm_runtime.h>
#include <linux/i2c.h>
#include <linux/of_device.h>
#include <linux/mutex.h>
#include <linux/power_supply.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/kthread.h>
#include <linux/reboot.h>

#include <mt-plat/upmu_common.h>
#include <charger_class.h>
#include <mt-plat/v1/charger_type.h>
#ifdef CONFIG_RT_REGMAP
#include <mt-plat/rt-regmap.h>
#endif /* CONFIG_RT_REGMAP */
#include <linux/timer.h>

//nclude "mtk_charger_intf.h"
#define AW32207_DRIVER_VERSION "V2.0.4"
#include "aw32207.h"
#include "mtk_charger.h"

const unsigned int VBAT_CVTH[] = {
	3500000, 3520000, 3540000, 3560000,
	3580000, 3600000, 3620000, 3640000,
	3660000, 3680000, 3700000, 3720000,
	3740000, 3760000, 3780000, 3800000,
	3820000, 3840000, 3860000, 3880000,
	3900000, 3920000, 3940000, 3960000,
	3980000, 4000000, 4020000, 4040000,
	4060000, 4080000, 4100000, 4120000,
	4140000, 4160000, 4180000, 4200000,
	4220000, 4240000, 4260000, 4280000,
	4300000, 4320000, 4340000, 4360000,
	4380000, 4400000, 4420000, 4440000,
	4460000, 4480000, 4500000
};

const unsigned int CSTH56[] = {
	485000, 607000, 850000, 971000,
	1092000, 1214000, 1336000, 1457000,
	1578000, 1699000, 1821000, 1821000,
	1821000, 1821000, 1821000, 1821000
};

const unsigned int INPUT_CSTH[] = {
	100000, 500000, 800000, 5000000
};

const unsigned int BOOST_CURRENT_LIMIT[] = {
	500, 750, 1200, 1400, 1650, 1875, 2150,
};

enum charge_ic_stat {
	CHARGE_ICSTAT_SLEEP = 0,
	CHARGE_ICSTAT_PRECHG,
	CHARGE_ICSTAT_FASTCHG,
	CHARGE_ICSTAT_CHGDONE,
	CHARGE_ICSTAT_MAX,
};

#define aw32207_SLAVE_ADDR_WRITE	0xD4
#define aw32207_SLAVE_ADDR_Read		0xD5
#define AW_I2C_RETRIES			5
#define AW_I2C_RETRY_DELAY		2

struct aw32207_info {
	struct charger_device *chg_dev;
	struct power_supply *psy;
	struct power_supply *ext_psy;
	struct charger_properties chg_props;
	struct device *dev;
	const char *chg_dev_name;
	const char *eint_name;
	/*enum charger_type chg_type;*/
	int irq;
	int cv;
};

#ifdef CONFIG_OF
static int aw32207_cd_pin;
#endif
static struct i2c_client *new_client;
static const struct i2c_device_id aw32207all_i2c_id[] = { {"aw32207", 0}, {} };

struct timer_list		timer_aw32207;
struct work_struct	timer_work;

unsigned int charging_value_to_parameter(const unsigned int *parameter,
			const unsigned int array_size, const unsigned int val)
{
	if (val < array_size)
		return parameter[val];
	pr_info("Can't find the parameter\n");
	return parameter[0];
}

unsigned int charging_parameter_to_value(const unsigned int *parameter,
			const unsigned int array_size, const unsigned int val)
{
	unsigned int i;

	pr_debug_ratelimited("array_size = %d\n", array_size);

	for (i = 0; i < array_size; i++) {
		if (val == *(parameter + i))
			return i;
	}

	pr_info("NO register value match\n");
	/* TODO: ASSERT(0); not find the value */
	return 0;
}

static unsigned int bmt_find_closest_level(const unsigned int *pList,
			unsigned int number, unsigned int level)
{
	unsigned int i;
	unsigned int max_value_in_last_element;

	if (pList[0] < pList[1])
		max_value_in_last_element = 1;
	else
		max_value_in_last_element = 0;

	if (max_value_in_last_element == 1) {
		/* max value in the last element */
		for (i = (number - 1); i != 0; i--) {
			if (pList[i] <= level) {
				pr_debug_ratelimited("zzf_%d<=%d, i=%d\n",
				pList[i], level, i);
				return pList[i];
			}
		}
		pr_info("Can't find closest level\n");
		return pList[0]; /* return 000; */
	} else {
		/* max value in the first element */
		for (i = 0; i < number; i++) {
			if (pList[i] <= level)
				return pList[i];
		}
		pr_info("Can't find closest level\n");
		return pList[number - 1];
		/* return 000; */
	}
}

unsigned char aw32207_reg[AW32207_REG_NUM] = { 0 };
static DEFINE_MUTEX(aw32207_i2c_access);
static DEFINE_MUTEX(aw32207_access_lock);

static int aw32207_read_byte(u8 reg_addr, u8 *rd_buf, int rd_len)
{
	int ret = 0;
	unsigned char cnt = 0;
	struct i2c_adapter *adap = new_client->adapter;
	struct i2c_msg msg[2];
	u8 *w_buf = NULL;
	u8 *r_buf = NULL;

	memset(msg, 0, 2 * sizeof(struct i2c_msg));

	w_buf = kzalloc(1, GFP_KERNEL);
	if (w_buf == NULL)
		return -1;
	r_buf = kzalloc(rd_len, GFP_KERNEL);
	if (r_buf == NULL)
		return -1;

	*w_buf = reg_addr;

	msg[0].addr = new_client->addr;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = w_buf;

	msg[1].addr = new_client->addr;
	msg[1].flags = 1;
	msg[1].len = rd_len;
	msg[1].buf = r_buf;

	while (cnt < AW_I2C_RETRIES) {
		ret = i2c_transfer(adap, msg, 2);
		if (ret < 0)
			pr_err("%s: i2c read byte error, ret = %d\n",
								__func__, ret);
		else
			break;

		cnt++;
		mdelay(AW_I2C_RETRY_DELAY);
	}

	memcpy(rd_buf, r_buf, rd_len);

	kfree(w_buf);
	kfree(r_buf);
	return ret;
}

int aw32207_write_byte(unsigned char reg_num, u8 *wr_buf, int wr_len)
{
	int ret = 0;
	unsigned char cnt = 0;
	struct i2c_adapter *adap = new_client->adapter;
	struct i2c_msg msg;
	u8 *w_buf = NULL;

	memset(&msg, 0, sizeof(struct i2c_msg));

	w_buf = kzalloc(wr_len, GFP_KERNEL);
	if (w_buf == NULL)
		return -1;
	w_buf[0] = reg_num;
	memcpy(w_buf + 1, wr_buf, wr_len);

	msg.addr = new_client->addr;
	msg.flags = 0;
	msg.len = wr_len;
	msg.buf = w_buf;

	while (cnt < AW_I2C_RETRIES) {
		ret = i2c_transfer(adap, &msg, 1);
		if (ret < 0)
			pr_err("%s: i2c write byte error, ret = %d\n",
								__func__, ret);
		else
			break;

		cnt++;
		mdelay(AW_I2C_RETRY_DELAY);
	}

	kfree(w_buf);
	return ret;
}

unsigned int aw32207_read_interface(unsigned char reg_num, unsigned char *val,
					unsigned char MASK, unsigned char SHIFT)
{
	unsigned char aw32207_reg = 0;
	unsigned int ret = 0;

	ret = aw32207_read_byte(reg_num, &aw32207_reg, 1);
	pr_debug_ratelimited("[aw32207_read_interface] Reg[%x]=0x%x\n", reg_num,
								aw32207_reg);
	aw32207_reg &= (MASK << SHIFT);
	*val = (aw32207_reg >> SHIFT);
	pr_debug_ratelimited("[aw32207_read_interface] val=0x%x\n", *val);

	return ret;
}

unsigned int aw32207_config_interface(unsigned char reg_num, unsigned char val,
					unsigned char MASK, unsigned char SHIFT)
{
	unsigned char aw32207_reg = 0;
	unsigned char aw32207_reg_ori = 0;
	unsigned int ret = 0;

	mutex_lock(&aw32207_access_lock);
	ret = aw32207_read_byte(reg_num, &aw32207_reg, 1);
	aw32207_reg_ori = aw32207_reg;

	aw32207_reg &= ~(MASK << SHIFT);
	aw32207_reg |= (val << SHIFT);

	if (reg_num == AW32207_CON4)
		aw32207_reg &= ~(1 << CON4_RESET_SHIFT);

	ret = aw32207_write_byte(reg_num, &aw32207_reg, 2);
	mutex_unlock(&aw32207_access_lock);

	return ret;
}

/* write one register directly */
unsigned int aw32207_reg_config_interface(unsigned char reg_num,
							unsigned char val)
{
	unsigned char aw32207_reg = val;

	return aw32207_write_byte(reg_num, &aw32207_reg, 2);
}

void aw32207_set_tmr_rst(unsigned int val)
{
	aw32207_config_interface((unsigned char)(AW32207_CON0),
				(unsigned char)(val),
				(unsigned char)(CON0_TMR_RST_MASK),
				(unsigned char)(CON0_TMR_RST_SHIFT)
				);
}
EXPORT_SYMBOL(aw32207_set_tmr_rst);


unsigned int aw32207_get_otg_status(void)
{
	unsigned char val = 0;

	aw32207_read_interface((unsigned char)(AW32207_CON0),
				(unsigned char *)(&val),
				(unsigned char)(CON0_OTG_MASK),
				(unsigned char)(CON0_OTG_SHIFT)
				);
	return val;
}

void aw32207_set_en_stat(unsigned int val)
{
	aw32207_config_interface((unsigned char)(AW32207_CON0),
				(unsigned char)(val),
				(unsigned char)(CON0_EN_STAT_MASK),
				(unsigned char)(CON0_EN_STAT_SHIFT)
				);
}

unsigned int aw32207_get_chip_status(void)
{
	unsigned char val = 0;

	aw32207_read_interface((unsigned char)(AW32207_CON0),
				(unsigned char *)(&val),
				(unsigned char)(CON0_STAT_MASK),
				(unsigned char)(CON0_STAT_SHIFT)
				);
	return val;
}

unsigned int aw32207_get_boost_status(void)
{
	unsigned char val = 0;

	aw32207_read_interface((unsigned char)(AW32207_CON0),
				(unsigned char *)(&val),
				(unsigned char)(CON0_BOOST_MASK),
				(unsigned char)(CON0_BOOST_SHIFT)
				);
	return val;

}

unsigned int aw32207_get_chg_fault_status(void)
{
	unsigned char val = 0;

	aw32207_read_interface((unsigned char)(AW32207_CON0),
				(unsigned char *)(&val),
				(unsigned char)(CON0_CHG_FAULT_MASK),
				(unsigned char)(CON0_CHG_FAULT_SHIFT)
				);
	return val;
}

void aw32207_set_input_charging_current(unsigned int val)
{
	aw32207_config_interface((unsigned char)(AW32207_CON1),
				(unsigned char)(val),
				(unsigned char)(CON1_LIN_LIMIT_MASK),
				(unsigned char)(CON1_LIN_LIMIT_SHIFT)
				);
}

unsigned int aw32207_get_input_charging_current(void)
{
	unsigned char val = 0;

	aw32207_read_interface((unsigned char)(AW32207_CON1),
				(unsigned char *)(&val),
				(unsigned char)(CON1_LIN_LIMIT_MASK),
				(unsigned char)(CON1_LIN_LIMIT_SHIFT)
				);

	return val;
}

void aw32207_set_v_low(unsigned int val)
{

	aw32207_config_interface((unsigned char)(AW32207_CON1),
				(unsigned char)(val),
				(unsigned char)(CON1_LOW_V_MASK),
				(unsigned char)(CON1_LOW_V_SHIFT)
				);
}

void aw32207_set_te(unsigned int val)
{
	aw32207_config_interface((unsigned char)(AW32207_CON1),
				(unsigned char)(val),
				(unsigned char)(CON1_TE_MASK),
				(unsigned char)(CON1_TE_SHIFT)
				);
}

void aw32207_set_cen(unsigned int val)
{
	aw32207_config_interface((unsigned char)(AW32207_CON1),
				(unsigned char)(val),
				(unsigned char)(CON1_CEN_MASK),
				(unsigned char)(CON1_CEN_SHIFT)
				);
}

static unsigned int aw32207_get_ce(void)
{
	unsigned char val = 0;
	aw32207_read_interface((unsigned char)(AW32207_CON1),
				(unsigned char *)(&val),
				(unsigned char)(CON1_CEN_MASK),
				(unsigned char)(CON1_CEN_SHIFT)
				);
	return val;
}

void aw32207_set_hz_mode(unsigned int val)
{
	aw32207_config_interface((unsigned char)(AW32207_CON1),
				(unsigned char)(val),
				(unsigned char)(CON1_HZ_MODE_MASK),
				(unsigned char)(CON1_HZ_MODE_SHIFT)
				);
}

static __maybe_unused unsigned int aw32207_get_hz_mode(void)
{
	unsigned char val = 0;
	aw32207_read_interface((unsigned char)(AW32207_CON1),
				(unsigned char *)(&val),
				(unsigned char)(CON1_HZ_MODE_MASK),
				(unsigned char)(CON1_HZ_MODE_SHIFT)
				);
	return val;
}

void aw32207_set_opa_mode(unsigned int val)
{
	aw32207_config_interface((unsigned char)(AW32207_CON1),
				(unsigned char)(val),
				(unsigned char)(CON1_OPA_MODE_MASK),
				(unsigned char)(CON1_OPA_MODE_SHIFT)
				);
}

void aw32207_set_voreg(unsigned int val)
{
	aw32207_config_interface((unsigned char)(AW32207_CON2),
				(unsigned char)(val),
				(unsigned char)(CON2_VOREG_MASK),
				(unsigned char)(CON2_VOREG_SHIFT)
				);
}
void aw32207_set_otg_pl(unsigned int val)
{
	aw32207_config_interface((unsigned char)(AW32207_CON2),
				(unsigned char)(val),
				(unsigned char)(CON2_OTG_PL_MASK),
				(unsigned char)(CON2_OTG_PL_SHIFT)
				);
}
void aw32207_set_otg_en(unsigned int val)
{
	aw32207_config_interface((unsigned char)(AW32207_CON2),
				(unsigned char)(val),
				(unsigned char)(CON2_OTG_EN_MASK),
				(unsigned char)(CON2_OTG_EN_SHIFT)
				);
}

unsigned int aw32207_get_vender_code(void)
{
	unsigned char val = 0;

	aw32207_read_interface((unsigned char)(AW32207_CON3),
				(unsigned char *)(&val),
				(unsigned char)(CON3_VENDER_MASK),
				(unsigned char)(CON3_VENDER_SHIFT)
				);
	return val;
}
unsigned int aw32207_get_pn(void)
{
	unsigned char val = 0;

	aw32207_read_interface((unsigned char)(AW32207_CON3),
				(unsigned char *)(&val),
				(unsigned char)(CON3_PN_MASK),
				(unsigned char)(CON3_PN_SHIFT)
				);
	return val;
}

unsigned int aw32207_get_revision(void)
{
	unsigned char val = 0;

	aw32207_read_interface((unsigned char)(AW32207_CON3),
				(unsigned char *)(&val),
				(unsigned char)(CON3_REVISION_MASK),
				(unsigned char)(CON3_REVISION_SHIFT)
				);
	return val;
}

void aw32207_set_reset(unsigned int val)
{
	aw32207_config_interface((unsigned char)(AW32207_CON4),
				(unsigned char)(val),
				(unsigned char)(CON4_RESET_MASK),
				(unsigned char)(CON4_RESET_SHIFT)
				);
	mdelay(50);
}

void aw32207_set_iocharge(unsigned int val)
{
	aw32207_config_interface((unsigned char)(AW32207_CON4),
				(unsigned char)(val),
				(unsigned char)(CON4_I_CHR_MASK),
				(unsigned char)(CON4_I_CHR_SHIFT)
				);
}

void aw32207_set_iterm(unsigned int val)
{
	aw32207_config_interface((unsigned char)(AW32207_CON4),
				(unsigned char)(val),
				(unsigned char)(CON4_I_TERM_MASK),
				(unsigned char)(CON4_I_TERM_SHIFT)
				);
}

void aw32207_set_bit3_iocharge(unsigned int val)
{
	aw32207_config_interface((unsigned char)(AW32207_CON5),
				(unsigned char)(val),
				(unsigned char)(CON5_I_CHR_BIT3_MASK),
				(unsigned char)(CON5_I_CHR_BIT3_SHIFT)
				);
}

void aw32207_set_bit4_iocharge(unsigned int val)
{
	aw32207_config_interface((unsigned char)(AW32207_CON5),
				(unsigned char)(val),
				(unsigned char)(CON5_I_CHR_BIT4_MASK),
				(unsigned char)(CON5_I_CHR_BIT4_SHIFT)
				);
}

void aw32207_set_iocharge_offset(unsigned int val)
{
	aw32207_config_interface((unsigned char)(AW32207_CON4),
				(unsigned char)(val),
				(unsigned char)(CON4_I_CHR_OFFSET_MASK),
				(unsigned char)(CON4_I_CHR_OFFSET_SHIFT)
				);
}

void aw32207_set_iochargeh(unsigned int val)
{
	aw32207_config_interface((unsigned char)(AW32207_CON5),
				(unsigned char)(val),
				(unsigned char)(CON5_I_CHRH_MASK),
				(unsigned char)(CON5_I_CHRH_SHIFT)
				);
}

void aw32207_set_dis_vreg(unsigned int val)
{
	aw32207_config_interface((unsigned char)(AW32207_CON5),
				(unsigned char)(val),
				(unsigned char)(CON5_DIS_VREG_MASK),
				(unsigned char)(CON5_DIS_VREG_SHIFT)
				);
}

void aw32207_set_io_level(unsigned int val)
{
	aw32207_config_interface((unsigned char)(AW32207_CON5),
				(unsigned char)(val),
				(unsigned char)(CON5_IO_LEVEL_MASK),
				(unsigned char)(CON5_IO_LEVEL_SHIFT)
				);
}

unsigned int aw32207_get_sp_status(void)
{
	unsigned char val = 0;

	aw32207_read_interface((unsigned char)(AW32207_CON5),
				(unsigned char *)(&val),
				(unsigned char)(CON5_DPM_STATUS_MASK),
				(unsigned char)(CON5_DPM_STATUS_SHIFT)
				);
	return val;
}

unsigned int aw32207_get_en_level(void)
{
	unsigned char val = 0;

	aw32207_read_interface((unsigned char)(AW32207_CON5),
				(unsigned char *)(&val),
				(unsigned char)(CON5_CD_STATUS_MASK),
				(unsigned char)(CON5_CD_STATUS_SHIFT)
				);
	return val;
}

void aw32207_set_vsp(unsigned int val)
{
	aw32207_config_interface((unsigned char)(AW32207_CON5),
				(unsigned char)(val),
				(unsigned char)(CON5_VSP_MASK),
				(unsigned char)(CON5_VSP_SHIFT)
				);
}

void aw32207_set_i_safe(unsigned int val)
{
	aw32207_config_interface((unsigned char)(AW32207_CON6),
				(unsigned char)(val),
				(unsigned char)(CON6_ISAFE_MASK),
				(unsigned char)(CON6_ISAFE_SHIFT)
				);
}

void aw32207_set_v_safe(unsigned int val)
{
	aw32207_config_interface((unsigned char)(AW32207_CON6),
				(unsigned char)(val),
				(unsigned char)(CON6_VSAFE_MASK),
				(unsigned char)(CON6_VSAFE_SHIFT)
				);
}

void aw32207_set_iinlimit_selection(unsigned int val)
{
	aw32207_config_interface((unsigned char)(AW32207_CON7),
				(unsigned char)(val),
				(unsigned char)(CON7_IINLIMIT_SELECTION_MASK),
				(unsigned char)(CON7_IINLIMIT_SELECTION_SHIFT)
				);
}

void aw32207_set_input_charging_current2(unsigned int val)
{
	aw32207_config_interface((unsigned char)(AW32207_CON7),
				(unsigned char)(val),
				(unsigned char)(CON7_IINLIMIT2_MASK),
				(unsigned char)(CON7_IINLIMIT2_SHIFT)
				);
}

static int aw32207_dump_register(struct charger_device *chg_dev)
{
	int i;

	for (i = 0; i < AW32207_REG_NUM; i++) {
		aw32207_read_byte(i, &aw32207_reg[i], 1);
		pr_info("aw32207_dump_kernel_[0x%x]=0x%x\n", i, aw32207_reg[i]);
	}
	pr_debug("\n");

	return 0;
}

static int aw32207_parse_dt(struct aw32207_info *info, struct device *dev)
{
	struct device_node *np = dev->of_node;

	pr_info("%s\n", __func__);

	if (!np) {
		pr_err("%s: no of node\n", __func__);
		return -ENODEV;
	}

	if (of_property_read_string(np, "charger_name",
						&info->chg_dev_name) < 0) {
		info->chg_dev_name = "primary_chg";
		pr_warn("%s: no charger name\n", __func__);
	}

	if (of_property_read_string(np, "alias_name", &
					(info->chg_props.alias_name)) < 0) {
		info->chg_props.alias_name = "aw32207";
		pr_warn("%s: no alias name\n", __func__);
	}

	if(of_property_read_u32(np, "battery_cv", &info->cv) < 0)
	{
		info->cv = 4200000;
		pr_warn("%s: use default: 4200000\n", __func__);
	}
	return 0;
}

static int aw32207_do_event(struct charger_device *chg_dev, unsigned int event,
							unsigned int args)
{
	if (chg_dev == NULL)
		return -EINVAL;

	pr_info("%s: event = %d\n", __func__, event);
	
	switch (event) {
	case EVENT_FULL:
		charger_dev_notify(chg_dev, CHARGER_DEV_NOTIFY_EOC);
		break;
	case EVENT_RECHARGE:
		charger_dev_notify(chg_dev, CHARGER_DEV_NOTIFY_RECHG);
		break;
	default:
		break;
	}

	return 0;
}

static int aw32207_enable_charging(struct charger_device *chg_dev, bool en)
{
	unsigned int status = 0;

	if (en) {
		aw32207_set_cen(0);
		aw32207_set_hz_mode(0);
		aw32207_set_opa_mode(0);
	} else {
		aw32207_set_cen(1);
	}

	return status;
}

static int aw32207_set_cv_voltage(struct charger_device *chg_dev, u32 cv)
{
	int status = 0;
	unsigned short int array_size;
	unsigned int set_cv_voltage;
	unsigned short int register_value;
	/*static kal_int16 pre_register_value; */
	array_size = ARRAY_SIZE(VBAT_CVTH);
	/*pre_register_value = -1; */
	set_cv_voltage = bmt_find_closest_level(VBAT_CVTH, array_size, cv);

	register_value = charging_parameter_to_value(VBAT_CVTH, array_size,
								set_cv_voltage);
	pr_info("charging_set_cv_voltage register_value=0x%x %d %d\n",
					register_value, cv, set_cv_voltage);
	aw32207_set_voreg(register_value);
	return status;
}

static int aw32207_get_current(struct charger_device *chg_dev, u32 *ichg)
{
	int status = 0;
	unsigned int array_size;
	unsigned char reg_value;

	array_size = ARRAY_SIZE(CSTH56);
	aw32207_read_interface(AW32207_CON4, &reg_value, CON4_I_CHR_MASK,
							CON4_I_CHR_SHIFT);
	*ichg = charging_value_to_parameter(CSTH56, array_size, reg_value);
	return status;
}

static int aw32207_set_current(struct charger_device *chg_dev,
							u32 current_value)
{
	unsigned int status = 0;
	unsigned int set_chr_current;
	unsigned int array_size;
	unsigned int register_value;

	array_size = ARRAY_SIZE(CSTH56);
	set_chr_current = bmt_find_closest_level(CSTH56, array_size,
								current_value);
	register_value = charging_parameter_to_value(CSTH56, array_size,
							set_chr_current);
	/*register_value = 0x0a;*/ /*0x0=494mA------(0x0A-0XF)=1854mA;*/
	pr_info("%s current_value = %d, register_value = 0x%x",__func__,current_value,register_value);
	aw32207_set_iocharge(register_value);
	return status;
}

static int aw32207_get_input_current(struct charger_device *chg_dev, u32 *aicr)
{
	unsigned int status = 0;
	unsigned int array_size;
	unsigned int register_value;

	array_size = ARRAY_SIZE(INPUT_CSTH);
	register_value = aw32207_get_input_charging_current();
	*aicr = charging_parameter_to_value(INPUT_CSTH, array_size, register_value);

	return status;
}

static int aw32207_set_input_current(struct charger_device *chg_dev,
							u32 current_value)
{
	unsigned int status = 0;
	unsigned int set_chr_current;
	unsigned int array_size;
	unsigned int register_value;

	if (current_value > 50000) {
		register_value = 0x3;
	} else {
		array_size = ARRAY_SIZE(INPUT_CSTH);
		set_chr_current = bmt_find_closest_level(INPUT_CSTH, array_size,
								current_value);
		register_value = charging_parameter_to_value(INPUT_CSTH,
						array_size, set_chr_current);
	}
	pr_info("%s current_value = %d, set_chr_current= %d, register_value=%d\n",__func__,current_value,set_chr_current,register_value);

	return status;
}

static int aw32207_get_charging_status(struct charger_device *chg_dev,
								bool *is_done)
{
	unsigned int status = 0;
	unsigned int ret_val;

	ret_val = aw32207_get_chip_status();

	if (ret_val == 0x2)
		*is_done = true;
	else
		*is_done = false;

	return status;
}

static int aw32207_reset_watch_dog_timer(struct charger_device *chg_dev)
{
	pr_debug("cxw aw32207_reset_watch_dog_timer \n");

	return 0;
}

static int aw32207_set_mivr(struct charger_device *chg_dev, u32 uV)
{
	int ret = 0;
//	struct aw32207all_info *chip = dev_get_drvdata(&chg_dev->dev);

//	dev_info(chip->dev,"%s:skip uV:%d\n",__func__,uV);
	return ret;
}

static int aw32207_get_mivr_state(struct charger_device *chg_dev, bool *in_loop)
{
	//struct aw32207all_info *chip = dev_get_drvdata(&chg_dev->dev);

//	*in_loop = 0;
	
	return 0;
}

static void aw32207_poll(struct timer_list *timer)
{
	printk("cxw aw32207_poll..\n");
	schedule_work(&timer_work);
}

static void timer_work_func(struct work_struct *work)
{
	printk("cxw timer_work_func\n");
	aw32207_set_tmr_rst(1);
	mod_timer(&timer_aw32207, jiffies + (15*HZ));
}

static int aw32207_charger_enable_otg(struct charger_device *chg_dev, bool en)
{
	//aw32207_set_otg_pl(1);
	printk("cxw aw32207_enable_otg\n");

	aw32207_set_otg_en(en);
	printk("cxw aw32207_enable_otg en = %d...\n", en);
	aw32207_dump_register(chg_dev);
	return 0;
}


static struct charger_ops aw32207_chg_ops = {
	/* Normal charging */
	.dump_registers = aw32207_dump_register,
	.enable = aw32207_enable_charging,
	.get_charging_current = aw32207_get_current,
	.set_charging_current = aw32207_set_current,
	.get_input_current = aw32207_get_input_current,
	.set_input_current = aw32207_set_input_current,
	/* .get_constant_voltage = aw32207_get_battery_voreg */
	.set_constant_voltage = aw32207_set_cv_voltage,
	.kick_wdt = aw32207_reset_watch_dog_timer,
	.is_charging_done = aw32207_get_charging_status,
	.set_mivr = aw32207_set_mivr,
	.get_mivr_state = aw32207_get_mivr_state,
	/* OTG */
	.enable_otg = aw32207_charger_enable_otg,
	.enable_discharge = NULL,
	.set_boost_current_limit = NULL,
	.event = aw32207_do_event,
};

static enum power_supply_property aw32207_charger_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_TYPE,
	POWER_SUPPLY_PROP_USB_TYPE,
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
};

static enum power_supply_usb_type aw32207_charger_usb_types[] = {
	POWER_SUPPLY_USB_TYPE_UNKNOWN,
	POWER_SUPPLY_USB_TYPE_SDP,
	POWER_SUPPLY_USB_TYPE_DCP,
	POWER_SUPPLY_USB_TYPE_CDP,
	POWER_SUPPLY_USB_TYPE_C,
	POWER_SUPPLY_USB_TYPE_PD,
	POWER_SUPPLY_USB_TYPE_PD_DRP,
	POWER_SUPPLY_USB_TYPE_APPLE_BRICK_ID
};

static char *aw32207_charger_supplied_to[] = {
	"battery",
	"mtk-master-charger"
};


static int aw322027_charger_get_online(struct aw32207_info *info,bool *val)
{
	union power_supply_propval prop;
	int ret = 0;
	
	if (IS_ERR_OR_NULL(info->ext_psy)){
		return -EINVAL;
	}
	
	ret = power_supply_get_property(info->ext_psy,POWER_SUPPLY_PROP_ONLINE,&prop);

	*val = prop.intval;
		
	return 0;
}

static int aw322027_charger_get_usb_type(struct aw32207_info *info)
{
	union power_supply_propval prop;
	int ret = 0;
	
	
	if (IS_ERR_OR_NULL(info->ext_psy)){
		return -EINVAL;
	}
	ret = power_supply_get_property(info->ext_psy,POWER_SUPPLY_PROP_USB_TYPE,&prop);
	
	
	return prop.intval;
}
int aw322027_charger_get_type(struct aw32207_info *info)
{
	union power_supply_propval prop;
	int ret = 0;
	
	if (IS_ERR_OR_NULL(info->ext_psy)){
		return -EINVAL;
	}
	
	ret = power_supply_get_property(info->ext_psy,POWER_SUPPLY_PROP_TYPE,&prop);
	
	
	return prop.intval;
}

static int aw32207_charger_get_property(struct power_supply *psy,
				       enum power_supply_property psp,
				       union power_supply_propval *val)
{
	struct aw32207_info *chip = power_supply_get_drvdata(psy);
		
	enum charge_ic_stat chg_stat = CHARGE_ICSTAT_SLEEP;
	bool pwr_rdy = false, chg_en = false;
	int ret = 0;

	chip->ext_psy = power_supply_get_by_name("mtk_charger_type");
	if (!(chip->ext_psy)) {
		ret = PTR_ERR(chip->ext_psy);
		return -EINVAL;
	}
	//dev_dbg(chg_data->dev, "%s: prop = %d\n", __func__, psp);
	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		ret = aw322027_charger_get_online(chip, &pwr_rdy);
		val->intval = pwr_rdy;
		break;
	case POWER_SUPPLY_PROP_STATUS:
		ret = aw322027_charger_get_online(chip, &pwr_rdy);
		chg_en = (aw32207_get_ce()?false:true);
		if(0x2 == aw32207_get_chip_status()) {
			chg_stat = CHARGE_ICSTAT_CHGDONE;
		}
		pr_err("%s gezi pwr_rdy = %d,chg_en = %d,chg_stat = %d\n",__func__,pwr_rdy,chg_en,chg_stat);
		
		if (!pwr_rdy) {
			val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
			return ret;
		}
		switch (chg_stat) {
		case CHARGE_ICSTAT_SLEEP:
			if(chg_en)
				val->intval = POWER_SUPPLY_STATUS_CHARGING;
			else
				val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
			break;
		case CHARGE_ICSTAT_PRECHG:
		case CHARGE_ICSTAT_FASTCHG:
			if (chg_en)
				val->intval = POWER_SUPPLY_STATUS_CHARGING;
			else
				val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
			break;
		case CHARGE_ICSTAT_CHGDONE:
			val->intval = POWER_SUPPLY_STATUS_FULL;
			break;
		
		default:
			ret = -ENODATA;
			break;
		}
		break;
	case POWER_SUPPLY_PROP_TYPE:
		val->intval = aw322027_charger_get_type(chip);
		break;
	case POWER_SUPPLY_PROP_USB_TYPE:
		val->intval = aw322027_charger_get_usb_type(chip);
		break;
	default:
		ret = -ENODATA;
	}
	printk("%s psp=%d, val->intval=%d\n",__func__,psp,val->intval);
	return ret;

}

static const struct power_supply_desc aw32207_charger_desc = {
	.name 			= "aw32207-charger", // "sgm41511-charger",
	.type			= POWER_SUPPLY_TYPE_USB,
	.properties		= aw32207_charger_properties,
	.num_properties		= ARRAY_SIZE(aw32207_charger_properties),
	.get_property		= aw32207_charger_get_property,
	.usb_types		= aw32207_charger_usb_types,
	.num_usb_types		= ARRAY_SIZE(aw32207_charger_usb_types),
};

static int aw32207_driver_probe(struct i2c_client *client,
						const struct i2c_device_id *id)
{
	int ret = 0;
	struct aw32207_info *info = NULL;
	struct power_supply_config charger_cfg = {};

	pr_info("[aw32207_driver_probe]\n");
	pr_info("aw32207 driver version %s\n", AW32207_DRIVER_VERSION);
	info = devm_kzalloc(&client->dev, sizeof(struct aw32207_info),
								GFP_KERNEL);

	if (!info)
		return -ENOMEM;

	new_client = client;
	info->dev = &client->dev;
	ret = aw32207_parse_dt(info, &client->dev);
	if (ret < 0)
		return ret;
	ret = aw32207_get_vender_code();
	if (ret != 2) {
		pr_err("%s: get vendor id failed\n", __func__);
		devm_kfree(&client->dev, info);
		return -ENODEV;
	}
	/* Register charger device */
	info->chg_dev = charger_device_register(info->chg_dev_name,
			&client->dev, info, &aw32207_chg_ops, &info->chg_props);

	if (IS_ERR_OR_NULL(info->chg_dev)) {
		pr_err("%s: register charger device failed\n", __func__);
		ret = PTR_ERR(info->chg_dev);
		return ret;
	}

	charger_cfg.drv_data = info;
	charger_cfg.of_node = client->dev.of_node;
	charger_cfg.supplied_to = aw32207_charger_supplied_to;
	charger_cfg.num_supplicants = ARRAY_SIZE(aw32207_charger_supplied_to);
	
	info->psy = devm_power_supply_register(&client->dev,
						 &aw32207_charger_desc,
						 &charger_cfg);
	if (!info->psy) {
		pr_err("%s: get power supply failed\n", __func__);
		return -EINVAL;
	}

	/* set default value */
	aw32207_reg_config_interface(AW32207_CON0, 0xd0);
	aw32207_reg_config_interface(AW32207_CON1, 0x38);
	aw32207_set_iterm(0x1);

//	aw32207_reg_config_interface(AW32207_CON8, 0x8e);

	aw32207_dump_register(info->chg_dev);
	pr_info("%s done.\n",__func__);

	return 0;
}
						
#ifdef CONFIG_OF
static const struct of_device_id aw32207_of_match[] = {
	{.compatible = "awinic,aw32207"},
	{},
};
#else
static struct i2c_board_info i2c_aw32207 __initdata = {
	I2C_BOARD_INFO("aw32207all", (aw32207_SLAVE_ADDR_WRITE >> 1))
};
#endif

static struct i2c_driver aw32207_driver = {
	.driver = {
		.name = "aw32207",
#ifdef CONFIG_OF
		.of_match_table = aw32207_of_match,
#endif
		},
	.probe = aw32207_driver_probe,
	.id_table = aw32207all_i2c_id,
};

static int __init aw32207_init(void)
{
	pr_info("aw32207 driver version %s\n", AW32207_DRIVER_VERSION);

	if (i2c_add_driver(&aw32207_driver) != 0)
		pr_info("Failed to register aw32207 i2c driver.\n");
	else
		pr_info("Success to register aw32207 i2c driver.\n");

	return 0;
}

static void __exit aw32207_exit(void)
{
	i2c_del_driver(&aw32207_driver);
}

module_init(aw32207_init);
module_exit(aw32207_exit);
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("I2C aw32207 Driver");
MODULE_AUTHOR("Joseph zhangzetao@awinic.com.cn>");

