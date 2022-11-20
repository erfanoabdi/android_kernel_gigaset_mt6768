
/*
 * SGM ETA6953 charger driver by SUYI
 *
 * Copyright (C) 2021 SGMicro Corporation
 *
 * This driver is for Linux kernel 4.4
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See http://www.gnu.org/licenses/gpl-2.0.html for more details.
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
#if 0
#include <mt-plat/rt-regmap.h>
#endif /* CONFIG_RT_REGMAP */
#include <mt-plat/mtk_boot_common.h>

#include "charger_class.h"
#include "mtk_charger.h"

#include "eta6953.h"
#define ETA6953_DRV_VERSION	"1.0.0_SGM"

enum ETA6953_streg_idx {
	ETA6953_ST_STATUS = 0,
	ETA6953_ST_FAULT,
	ETA6953_ST_STATUS2,
	ETA6953_STREG_MAX,
};

enum eta6953_ic_stat {
	ETA6953_ICSTAT_SLEEP = 0,
	ETA6953_ICSTAT_PRECHG,
	ETA6953_ICSTAT_FASTCHG,
	ETA6953_ICSTAT_CHGDONE,
	ETA6953_ICSTAT_MAX,
};

enum eta6953_mivr_track {
	ETA6953_MIVRTRACK_REG = 0,
	ETA6953_MIVRTRACK_VBAT_200MV,
	ETA6953_MIVRTRACK_VBAT_250MV,
	ETA6953_MIVRTRACK_VBAT_300MV,
	ETA6953_MIVRTRACK_MAX,
};

enum eta6953_port_stat {
	ETA6953_PORTSTAT_NOINPUT = 0,
	ETA6953_PORTSTAT_SDP,
	ETA6953_PORTSTAT_CDP,
	ETA6953_PORTSTAT_DCP,
	ETA6953_PORTSTAT_RESERVED_1,
	ETA6953_PORTSTAT_UKNADPT,
	ETA6953_PORTSTAT_NSADPT,
	ETA6953_PORTSTAT_OTG,
	ETA6953_PORTSTAT_MAX,
};

enum eta6953_usbsw_state {
	ETA6953_USBSW_CHG = 0,
	ETA6953_USBSW_USB,
};

struct eta6953_desc {
	u32 vac_ovp;
	u32 mivr;
	u32 aicr;
	u32 cv;
	u32 ichg;
	u32 ieoc;
	u32 safe_tmr;
	u32 wdt;
	u32 mivr_track;
	bool en_safe_tmr;
	bool en_te;
	bool en_jeita;
	bool ceb_invert;
	bool dis_i2c_tout;
	bool en_qon_rst;
	bool auto_aicr;
	const char *chg_name;
};

/* These default values will be applied if there's no property in dts */
static struct eta6953_desc eta6953_default_desc = {
	.vac_ovp = 14000000,
	.mivr = 4400000,
	.aicr = 500000,
	.cv = 4400000,
	.ichg = 2000000,
	.ieoc = 200000,
	.safe_tmr = 10,
	.wdt = 40,
	.mivr_track = ETA6953_MIVRTRACK_REG,
	.en_safe_tmr = true,
	.en_te = true,
	.en_jeita = true,
	.ceb_invert = false,
	.dis_i2c_tout = false,
	.en_qon_rst = true,
	.auto_aicr = true,
	.chg_name = "primary_chg",
};

static const u32 eta6953_vac_ovp[] = {
	5500000, 6500000, 10500000, 14000000,
};

static const u32 eta6953_wdt[] = {
	0, 40, 80, 160,
};

static const u32 eta6953_otgcc[] = {
	500000, 1200000,
};

struct eta6953_chip {
	struct i2c_client *client;
	struct device *dev;
	struct charger_device *chg_dev;
	struct charger_properties chg_props;
	struct mutex io_lock;
#ifdef CONFIG_MTK_EXTERNAL_CHARGER_TYPE_DETECT
	struct mutex bc12_lock;
	struct mutex bc12_en_lock;
#endif /* CONFIG_MTK_EXTERNAL_CHARGER_TYPE_DETECT */
	int hidden_mode_cnt;
	u8 dev_id;
	u8 dev_rev;
	u8 chip_rev;
	struct eta6953_desc *desc;
	u32 intr_gpio;
	u32 ceb_gpio;
	int irq;
	u8 irq_mask[ETA6953_STREG_MAX];
#ifdef CONFIG_MTK_EXTERNAL_CHARGER_TYPE_DETECT
	struct delayed_work psy_dwork;
	atomic_t vbus_gd;
	bool attach;
	enum eta6953_port_stat port;
	enum charger_type chg_type;
	struct power_supply *psy;
	struct wakeup_source bc12_en_ws;
	int bc12_en_buf[2];
	int bc12_en_buf_idx;
	atomic_t bc12_en_req_cnt;
	wait_queue_head_t bc12_en_req;
	struct task_struct *bc12_en_kthread;
#endif /* CONFIG_MTK_EXTERNAL_CHARGER_TYPE_DETECT */
	bool chg_done_once;
	struct wakeup_source buck_dwork_ws;

	bool enter_shipping_mode;
	//struct completion aicc_done;
	struct completion pe_done;
};

static const u8 eta6953_reg_addr[] = {
	ETA6953_REG_00,
	ETA6953_REG_01,
	ETA6953_REG_02,
	ETA6953_REG_03,
	ETA6953_REG_04,
	ETA6953_REG_05,
	ETA6953_REG_06,
	ETA6953_REG_07,
	ETA6953_REG_08,
	ETA6953_REG_09,
	ETA6953_REG_0A,
	ETA6953_REG_0B,
};

static int eta6953_read_device(void *client, u32 addr, int len, void *dst)
{
	return i2c_smbus_read_i2c_block_data(client, addr, len, dst);
}

static int eta6953_write_device(void *client, u32 addr, int len,
			       const void *src)
{
	return i2c_smbus_write_i2c_block_data(client, addr, len, src);
}

static inline int __eta6953_i2c_read_byte(struct eta6953_chip *chip, u8 cmd,
					 u8 *data)
{
	int ret = 0;
	u8 regval = 0;

	ret = eta6953_read_device(chip->client, cmd, 1, &regval);

	if (ret < 0)
		dev_notice(chip->dev, "%s reg0x%02X fail(%d)\n",
				      __func__, cmd, ret);
	else {
		/*dev_dbg(chip->dev, "%s reg0x%02X = 0x%02X\n",
				   __func__, cmd, regval);*/
		*data = regval;
	}

	return ret;
}

static int eta6953_i2c_read_byte(struct eta6953_chip *chip, u8 cmd, u8 *data)
{
	int ret = 0;

	mutex_lock(&chip->io_lock);
	ret = __eta6953_i2c_read_byte(chip, cmd, data);
	mutex_unlock(&chip->io_lock);

	return ret;
}

static inline int __eta6953_i2c_write_byte(struct eta6953_chip *chip, u8 cmd,
					  u8 data)
{
	int ret = 0;

#if 0
	ret = rt_regmap_block_write(chip->rm_dev, cmd, 1, &data);
#else
	ret = eta6953_write_device(chip->client, cmd, 1, &data);
#endif /* CONFIG_RT_REGMAP */

	if (ret < 0)
		dev_notice(chip->dev, "%s reg0x%02X = 0x%02X fail(%d)\n",
				      __func__, cmd, data, ret);
	/*else
		dev_dbg(chip->dev, "%s reg0x%02X = 0x%02X\n",
				   __func__, cmd, data);*/

	return ret;
}
/*
static int eta6953_i2c_write_byte(struct eta6953_chip *chip, u8 cmd, u8 data)
{
	int ret = 0;

	mutex_lock(&chip->io_lock);
	ret = __eta6953_i2c_write_byte(chip, cmd, data);
	mutex_unlock(&chip->io_lock);

	return ret;
}
*/
static inline int __eta6953_i2c_block_read(struct eta6953_chip *chip, u8 cmd,
					  u32 len, u8 *data)
{
	int ret = 0;/* i = 0;*/

#if 0
	ret = rt_regmap_block_read(chip->rm_dev, cmd, len, data);
#else
	ret = eta6953_read_device(chip->client, cmd, len, data);
#endif /* CONFIG_RT_REGMAP */

	if (ret < 0)
		dev_notice(chip->dev, "%s reg0x%02X..reg0x%02X fail(%d)\n",
				      __func__, cmd, cmd + len - 1, ret);
	/*else
		for (i = 0; i <= len - 1; i++)
			dev_dbg(chip->dev, "%s reg0x%02X = 0x%02X\n",
					   __func__, cmd + i, data[i]);*/

	return ret;
}

static int eta6953_i2c_block_read(struct eta6953_chip *chip, u8 cmd, u32 len,
				 u8 *data)
{
	int ret = 0;

	mutex_lock(&chip->io_lock);
	ret = __eta6953_i2c_block_read(chip, cmd, len, data);
	mutex_unlock(&chip->io_lock);

	return ret;
}

static inline int __eta6953_i2c_block_write(struct eta6953_chip *chip, u8 cmd,
					   u32 len, const u8 *data)
{
	int ret = 0,i = 0;

#if 0
	ret = rt_regmap_block_write(chip->rm_dev, cmd, len, data);
#else
	ret = eta6953_write_device(chip->client, cmd, len, data);
#endif /* CONFIG_RT_REGMAP */

	if (ret < 0) {
		dev_notice(chip->dev, "%s fail(%d)\n", __func__, ret);
		for (i = 0; i <= len - 1; i++)
			dev_notice(chip->dev, "%s reg0x%02X = 0x%02X\n",
					      __func__, cmd + i, data[i]);
	}
	/*else
		for (i = 0; i <= len - 1; i++)
			dev_dbg(chip->dev, "%s reg0x%02X = 0x%02X\n",
					   __func__, cmd + i, data[i]);*/

	return ret;
}
/*
static int eta6953_i2c_block_write(struct eta6953_chip *chip, u8 cmd, u32 len,
				  const u8 *data)
{
	int ret = 0;

	mutex_lock(&chip->io_lock);
	ret = __eta6953_i2c_block_write(chip, cmd, len, data);
	mutex_unlock(&chip->io_lock);

	return ret;
}
*/
static int eta6953_i2c_test_bit(struct eta6953_chip *chip, u8 cmd, u8 shift,
			       bool *is_one)
{
	int ret = 0;
	u8 regval = 0;

	ret = eta6953_i2c_read_byte(chip, cmd, &regval);
	if (ret < 0) {
		*is_one = false;
		return ret;
	}

	regval &= 1 << shift;
	*is_one = (regval ? true : false);

	return ret;
}

static int eta6953_i2c_update_bits(struct eta6953_chip *chip, u8 cmd, u8 data,
				  u8 mask)
{
	int ret = 0;
	u8 regval = 0;

	mutex_lock(&chip->io_lock);
	ret = __eta6953_i2c_read_byte(chip, cmd, &regval);
	if (ret < 0)
		goto out;

	regval &= ~mask;
	regval |= (data & mask);

	ret = __eta6953_i2c_write_byte(chip, cmd, regval);
out:
	mutex_unlock(&chip->io_lock);
	return ret;
}

static inline int eta6953_set_bit(struct eta6953_chip *chip, u8 cmd, u8 mask)
{
	return eta6953_i2c_update_bits(chip, cmd, mask, mask);
}

static inline int eta6953_clr_bit(struct eta6953_chip *chip, u8 cmd, u8 mask)
{
	return eta6953_i2c_update_bits(chip, cmd, 0x00, mask);
}

static inline u8 eta6953_closest_reg(u32 min, u32 max, u32 step, u32 target)
{
	if (target < min)
		return 0;

	if (target >= max)
		target = max;

	return (target - min) / step;
}

static inline u8 eta6953_closest_reg_via_tbl(const u32 *tbl, u32 tbl_size,
					    u32 target)
{
	u32 i = 0;

	if (target < tbl[0])
		return 0;

	for (i = 0; i < tbl_size - 1; i++) {
		if (target >= tbl[i] && target < tbl[i + 1])
			return i;
	}

	return tbl_size - 1;
}

static inline u32 eta6953_closest_value(u32 min, u32 max, u32 step, u8 regval)
{
	u32 val = 0;

	val = min + regval * step;
	if (val > max)
		val = max;

	return val;
}

static int __eta6953_get_vbus_stat(struct eta6953_chip *chip,
				u8 *stat)
{
	int ret = 0;
	u8 regval = 0;

	ret = eta6953_i2c_read_byte(chip, ETA6953_REG_ICSTAT, &regval);
	if (ret < 0)
		return ret;
	*stat = (regval & ETA6953_VBUSSTAT_MASK) >> ETA6953_VBUSSTAT_SHIFT;

	return ret;
}

static int __eta6953_get_chrg_stat(struct eta6953_chip *chip,
				u8 *stat)
{
	int ret = 0;
	u8 regval = 0;

	ret = eta6953_i2c_read_byte(chip, ETA6953_REG_ICSTAT, &regval);
	if (ret < 0)
		return ret;
	*stat = (regval & ETA6953_CHRGSTAT_MASK) >> ETA6953_CHRGSTAT_SHIFT;

	return ret;
}

static int __eta6953_is_fault(struct eta6953_chip *chip, bool *normal)
{
	int ret = 0;
	u8 regval = 0;

	ret = eta6953_i2c_read_byte(chip, ETA6953_REG_FAULT, &regval);
	if (ret < 0)
		return ret;
	*normal = (regval == 0);

	return ret;
}

static int __eta6953_get_ic_stat(struct eta6953_chip *chip,
				enum eta6953_ic_stat *stat)
{
	int ret = 0;
	u8 regval = 0;

	ret = eta6953_i2c_read_byte(chip, ETA6953_REG_ICSTAT, &regval);
	if (ret < 0)
		return ret;
	*stat = (regval & ETA6953_ICSTAT_MASK) >> ETA6953_ICSTAT_SHIFT;

	return ret;
}

static int __eta6953_get_mivr(struct eta6953_chip *chip, u32 *mivr)
{
	int ret = 0;
	u8 regval = 0;

	ret = eta6953_i2c_read_byte(chip, ETA6953_REG_MIVR, &regval);
	if (ret < 0)
		return ret;

	regval = (regval & ETA6953_MIVR_MASK) >> ETA6953_MIVR_SHIFT;
	*mivr = eta6953_closest_value(ETA6953_MIVR_MIN, ETA6953_MIVR_MAX,
				     ETA6953_MIVR_STEP, regval);

	return ret;
}

static int __eta6953_get_aicr(struct eta6953_chip *chip, u32 *aicr)
{
	int ret = 0;
	u8 regval = 0;

	ret = eta6953_i2c_read_byte(chip, ETA6953_REG_AICR, &regval);
	if (ret < 0)
		return ret;

	regval = (regval & ETA6953_AICR_MASK) >> ETA6953_AICR_SHIFT;
	*aicr = eta6953_closest_value(ETA6953_AICR_MIN, ETA6953_AICR_MAX,
				     ETA6953_AICR_STEP, regval);
	if (*aicr > ETA6953_AICR_MIN && *aicr < ETA6953_AICR_MAX)
		*aicr -= ETA6953_AICR_STEP;

	return ret;
}

static int __eta6953_get_cv(struct eta6953_chip *chip, u32 *cv)
{
	int ret = 0;
	u8 regval = 0;

	ret = eta6953_i2c_read_byte(chip, ETA6953_REG_CV, &regval);
	if (ret < 0)
		return ret;

	regval = (regval & ETA6953_CV_MASK) >> ETA6953_CV_SHIFT;
	*cv = eta6953_closest_value(ETA6953_CV_MIN, ETA6953_CV_MAX, ETA6953_CV_STEP,
				   regval);

	return ret;
}

static int __eta6953_get_ichg(struct eta6953_chip *chip, u32 *ichg)
{
	int ret = 0;
	u8 regval = 0;

	ret = eta6953_i2c_read_byte(chip, ETA6953_REG_ICHG, &regval);
	if (ret < 0)
		return ret;

	regval = (regval & ETA6953_ICHG_MASK) >> ETA6953_ICHG_SHIFT;
	*ichg = eta6953_closest_value(ETA6953_ICHG_MIN, ETA6953_ICHG_MAX,
				     ETA6953_ICHG_STEP, regval);

	return ret;
}

static int __eta6953_get_ieoc(struct eta6953_chip *chip, u32 *ieoc)
{
	int ret = 0;
	u8 regval = 0;

	ret = eta6953_i2c_read_byte(chip, ETA6953_REG_IEOC, &regval);
	if (ret < 0)
		return ret;

	regval = (regval & ETA6953_IEOC_MASK) >> ETA6953_IEOC_SHIFT;
	*ieoc = eta6953_closest_value(ETA6953_IEOC_MIN, ETA6953_IEOC_MAX,
				     ETA6953_IEOC_STEP, regval);

	return ret;
}

static int __eta6953_is_hz_enabled(struct eta6953_chip *chip, bool *en)
{
	return eta6953_i2c_test_bit(chip, ETA6953_REG_HZ,
				   ETA6953_FORCE_HZ_SHIFT, en);
}

static int __eta6953_is_chg_enabled(struct eta6953_chip *chip, bool *en)
{
	return eta6953_i2c_test_bit(chip, ETA6953_REG_CHG_EN,
				   ETA6953_CHG_EN_SHIFT, en);
}

static int __eta6953_enable_shipmode(struct eta6953_chip *chip, bool en)
{
	dev_info(chip->dev, "%s en = %d\n", __func__, en);
	return (en ? eta6953_set_bit : eta6953_clr_bit)
		(chip, ETA6953_REG_BATFETDIS, ETA6953_BATFETDIS_MASK);
}

static int __eta6953_enable_safe_tmr(struct eta6953_chip *chip, bool en)
{
	dev_info(chip->dev, "%s en = %d\n", __func__, en);
	return (en ? eta6953_set_bit : eta6953_clr_bit)
		(chip, ETA6953_REG_SAFETMR_EN, ETA6953_SAFETMR_EN_MASK);
}

static int __eta6953_enable_te(struct eta6953_chip *chip, bool en)
{
	dev_info(chip->dev, "%s en = %d\n", __func__, en);
	return (en ? eta6953_set_bit : eta6953_clr_bit)
		(chip, ETA6953_REG_TE, ETA6953_TE_MASK);
}

static int __eta6953_enable_hz(struct eta6953_chip *chip, bool en)
{
	int ret = 0;

	dev_info(chip->dev, "%s en = %d\n", __func__, en);

	if (ret < 0)
		return ret;

	/* Use force HZ */
	ret = (en ? eta6953_set_bit : eta6953_clr_bit)
		(chip, ETA6953_REG_HZ, ETA6953_FORCE_HZ_MASK);

	return ret;
}

static int __eta6953_enable_otg(struct eta6953_chip *chip, bool en)
{
	dev_info(chip->dev, "%s en = %d\n", __func__, en);
	return (en ? eta6953_set_bit : eta6953_clr_bit)
		(chip, ETA6953_REG_OTG_EN, ETA6953_OTG_EN_MASK);
}

static int __eta6953_set_otgcc(struct eta6953_chip *chip, u32 cc)
{
	dev_info(chip->dev, "%s cc = %d\n", __func__, cc);
	return (cc <= eta6953_otgcc[0] ? eta6953_clr_bit : eta6953_set_bit)
		(chip, ETA6953_REG_OTGCC, ETA6953_OTGCC_MASK);
}

static int __eta6953_enable_chg(struct eta6953_chip *chip, bool en)
{
	int ret = 0;
	struct eta6953_desc *desc = chip->desc;

	dev_info(chip->dev, "%s en = %d, chip_rev = %d\n",
			    __func__, en, chip->chip_rev);

	if (chip->ceb_gpio != U32_MAX)
		gpio_set_value(chip->ceb_gpio, desc->ceb_invert ? en : !en);

	ret = (en ? eta6953_set_bit : eta6953_clr_bit)
		(chip, ETA6953_REG_CHG_EN, ETA6953_CHG_EN_MASK);

	return ret;
}

static int __eta6953_set_vac_ovp(struct eta6953_chip *chip, u32 vac_ovp)
{
	u8 regval = 0;

	regval = eta6953_closest_reg_via_tbl(eta6953_vac_ovp,
					    ARRAY_SIZE(eta6953_vac_ovp),
					    vac_ovp);

	dev_info(chip->dev, "%s vac_ovp = %d(0x%02X)\n",
			    __func__, vac_ovp, regval);

	return eta6953_i2c_update_bits(chip, ETA6953_REG_VAC_OVP,
				      regval << ETA6953_VAC_OVP_SHIFT,
				      ETA6953_VAC_OVP_MASK);
}

static int __eta6953_set_mivr(struct eta6953_chip *chip, u32 mivr)
{
	u8 regval = 0;

	regval = eta6953_closest_reg(ETA6953_MIVR_MIN, ETA6953_MIVR_MAX,
				    ETA6953_MIVR_STEP, mivr);

	dev_info(chip->dev, "%s mivr = %d(0x%02X)\n", __func__, mivr, regval);

	return eta6953_i2c_update_bits(chip, ETA6953_REG_MIVR,
				      regval << ETA6953_MIVR_SHIFT,
				      ETA6953_MIVR_MASK);
}

static int __eta6953_set_aicr(struct eta6953_chip *chip, u32 aicr)
{
	u8 regval = 0;

	regval = eta6953_closest_reg(ETA6953_AICR_MIN, ETA6953_AICR_MAX,
				    ETA6953_AICR_STEP, aicr);
	/* 0 & 1 are both 50mA */
	if (aicr < ETA6953_AICR_MAX)
		regval += 1;

	dev_info(chip->dev, "%s aicr = %d(0x%02X)\n", __func__, aicr, regval);

	return eta6953_i2c_update_bits(chip, ETA6953_REG_AICR,
				      regval << ETA6953_AICR_SHIFT,
				      ETA6953_AICR_MASK);
}

static int __eta6953_set_cv(struct eta6953_chip *chip, u32 cv)
{
	u8 regval = 0;

	regval = eta6953_closest_reg(ETA6953_CV_MIN, ETA6953_CV_MAX,
				    ETA6953_CV_STEP, cv);

	dev_info(chip->dev, "%s cv = %d(0x%02X)\n", __func__, cv, regval);

	return eta6953_i2c_update_bits(chip, ETA6953_REG_CV,
				      regval << ETA6953_CV_SHIFT,
				      ETA6953_CV_MASK);
}

static int __eta6953_set_ichg(struct eta6953_chip *chip, u32 ichg)
{
	u8 regval = 0;

	regval = eta6953_closest_reg(ETA6953_ICHG_MIN, ETA6953_ICHG_MAX,
				    ETA6953_ICHG_STEP, ichg);

	dev_info(chip->dev, "%s ichg = %d(0x%02X)\n", __func__, ichg, regval);

	return eta6953_i2c_update_bits(chip, ETA6953_REG_ICHG, \
				      regval << ETA6953_ICHG_SHIFT, \
				      ETA6953_ICHG_MASK);
}

static int __eta6953_set_ieoc(struct eta6953_chip *chip, u32 ieoc)
{
	u8 regval = 0;

	regval = eta6953_closest_reg(ETA6953_IEOC_MIN, ETA6953_IEOC_MAX,
				    ETA6953_IEOC_STEP, ieoc);

	dev_info(chip->dev, "%s ieoc = %d(0x%02X)\n", __func__, ieoc, regval);

	return eta6953_i2c_update_bits(chip, ETA6953_REG_IEOC,
				      regval << ETA6953_IEOC_SHIFT,
				      ETA6953_IEOC_MASK);
}

static int __eta6953_set_safe_tmr(struct eta6953_chip *chip, u32 hr)
{
	u8 regval = 0;

	regval = eta6953_closest_reg(ETA6953_SAFETMR_MIN, ETA6953_SAFETMR_MAX,
				    ETA6953_SAFETMR_STEP, hr);

	dev_info(chip->dev, "%s time = %d(0x%02X)\n", __func__, hr, regval);

	return eta6953_i2c_update_bits(chip, ETA6953_REG_SAFETMR,
				      regval << ETA6953_SAFETMR_SHIFT,
				      ETA6953_SAFETMR_MASK);
}

static int __eta6953_set_wdt(struct eta6953_chip *chip, u32 sec)
{
	u8 regval = 0;

	/* 40s is the minimum, set to 40 except sec == 0 */
	if (sec <= 40 && sec > 0)
		sec = 40;
	regval = eta6953_closest_reg_via_tbl(eta6953_wdt, ARRAY_SIZE(eta6953_wdt),
					    sec);

	dev_info(chip->dev, "%s time = %d(0x%02X)\n", __func__, sec, regval);

	return eta6953_i2c_update_bits(chip, ETA6953_REG_WDT,
				      regval << ETA6953_WDT_SHIFT,
				      ETA6953_WDT_MASK);
}

static int __eta6953_set_mivrtrack(struct eta6953_chip *chip, u32 mivr_track)
{
	if (mivr_track >= ETA6953_MIVRTRACK_MAX)
		mivr_track = ETA6953_MIVRTRACK_VBAT_300MV;

	dev_info(chip->dev, "%s mivrtrack = %d\n", __func__, mivr_track);

	return eta6953_i2c_update_bits(chip, ETA6953_REG_MIVRTRACK,
				      mivr_track << ETA6953_MIVRTRACK_SHIFT,
				      ETA6953_MIVRTRACK_MASK);
}

static int __eta6953_kick_wdt(struct eta6953_chip *chip)
{
	dev_info(chip->dev, "%s\n", __func__);
	return eta6953_set_bit(chip, ETA6953_REG_WDTCNTRST, ETA6953_WDTCNTRST_MASK);
}
/*
static int __eta6953_set_jeita_cool_iset(struct eta6953_chip *chip, bool en)
{
	dev_info(chip->dev, "%s en = %d\n", __func__, en);
	return (en ? eta6953_set_bit : eta6953_clr_bit)
		(chip, ETA6953_REG_JEITA_COOL_ISET, ETA6953_JEITA_COOL_ISET_MASK);
}

static int __eta6953_set_jeita_cool_vset(struct eta6953_chip *chip, bool en)
{
	dev_info(chip->dev, "%s en = %d\n", __func__, en);
	return (en ? eta6953_set_bit : eta6953_clr_bit)
		(chip, ETA6953_REG_JEITA_COOL_VSET, ETA6953_JEITA_COOL_VSET_MASK);
}
*/
static int __eta6953_dump_registers(struct eta6953_chip *chip)
{
	int ret = 0, i = 0;
	u32 mivr = 0, aicr = 0, cv = 0, ichg = 0, ieoc = 0;
	u8 chrg_stat = 0, vbus_stat = 0;
	bool chg_en = 0;
	u8 regval = 0;
	bool ic_normal = false;
	enum eta6953_ic_stat ic_stat = ETA6953_ICSTAT_SLEEP;
	uint8_t regbuf[ARRAY_SIZE(eta6953_reg_addr)] = {0};

	ret = __eta6953_kick_wdt(chip);

	ret = __eta6953_get_mivr(chip, &mivr);
	ret = __eta6953_get_aicr(chip, &aicr);
	ret = __eta6953_get_cv(chip, &cv);
	ret = __eta6953_get_ichg(chip, &ichg);
	ret = __eta6953_get_ieoc(chip, &ieoc);
	ret = __eta6953_is_chg_enabled(chip, &chg_en);

	ret = __eta6953_get_chrg_stat(chip, &chrg_stat);
	ret = __eta6953_get_vbus_stat(chip, &vbus_stat);
	ret = __eta6953_get_ic_stat(chip, &ic_stat);

	ret = __eta6953_is_fault(chip, &ic_normal);
	if (ret < 0)
		dev_notice(chip->dev, "%s check charger fault fail(%d)\n",
					      __func__, ret);

	if (1){// ic_normal == false) {
		for (i = 0; i < ARRAY_SIZE(eta6953_reg_addr); i++) {
			ret = eta6953_i2c_read_byte(chip, eta6953_reg_addr[i], &regval);
			if (ret < 0)
				continue;
			regbuf[i] = regval;
			//dev_notice(chip->dev, "%s reg0x%02X = 0x%02X\n",
			//		      __func__, eta6953_reg_addr[i], regval);
		}
	}
	printk("%s: start\n",__func__);
	for (i = 0; i < ARRAY_SIZE(eta6953_reg_addr); i++){
		printk(KERN_CONT"%s: reg0x%02X = 0x%02x \n",__func__,eta6953_reg_addr[i],regbuf[i]);
	}
	printk("%s: end\n",__func__);
	dev_info(chip->dev, "%s MIVR = %dmV, AICR = %dmA\n",
		 __func__, mivr / 1000, aicr / 1000);

	dev_info(chip->dev, "%s chg_en = %d, chrg_stat = %d, vbus_stat = %d ic_stat = %d\n",
		 __func__, chg_en, chrg_stat, vbus_stat,ic_stat);

	dev_info(chip->dev, "%s CV = %dmV, ICHG = %dmA, IEOC = %dmA\n",
		 __func__, cv / 1000, ichg / 1000, ieoc / 1000);

	return 0;
}

static irqreturn_t eta6953_irq_handler(int irq, void *data)
{
	int ret = 0;//, i = 0, irqnum = 0, irqbit = 0;
	u8 evt[ETA6953_STREG_MAX] = {0};
	struct eta6953_chip *chip = (struct eta6953_chip *)data;

	dev_info(chip->dev, "%s\n", __func__);

	pm_stay_awake(chip->dev);

	ret = eta6953_i2c_block_read(chip, ETA6953_REG_STATUS, ETA6953_STREG_MAX,
				    evt);
	if (ret < 0) {
		dev_notice(chip->dev, "%s read evt fail(%d)\n", __func__, ret);
		goto out;
	}

	//switch ((evt[ETA6953_ST_STATUS] & ETA6953_CHRGSTAT_MASK) >> ETA6953_CHRGSTAT_SHIFT)
	//{
	//	case 3:
	//		eta6953_chg_done_irq_handler(chip);
	//
	//		break;
	//	default:
	//
	//		break;
	//}
	//
	//if ( evt[ETA6953_ST_STATUS] & ETA6953_PGSTAT_MASK )
	//{
	//	dev_info(chip->dev, "%s %d\n", __func__,__LINE__);
	//	//eta6953_chg_rdy_irq_handler(chip);
	//}
	//
	//switch ((evt[ETA6953_ST_FAULT] & ETA6953_CHRGFAULT_MASK) >> ETA6953_CHRGFAULT_SHIFT)
	//{
	//	case 1:
	//		eta6953_chg_busuv_irq_handler(chip);
	//
	//		break;
	//	case 3:
	//		eta6953_chg_tout_irq_handler(chip);
	//
	//		break;
	//	default:
	//
	//		break;
	//}
	//
	//if ( evt[ETA6953_ST_FAULT] & ETA6953_BATFAULT_MASK )
	//{
	//	eta6953_chg_batov_irq_handler(chip);
	//}
	//

out:
	pm_relax(chip->dev);
	return IRQ_HANDLED;
}

static int eta6953_register_irq(struct eta6953_chip *chip)
{
	int ret = 0;

	dev_info(chip->dev, "%s\n", __func__);

	ret = devm_gpio_request_one(chip->dev, chip->intr_gpio, GPIOF_DIR_IN,
			devm_kasprintf(chip->dev, GFP_KERNEL,
			"eta6953_intr_gpio.%s", dev_name(chip->dev)));
	if (ret < 0) {
		dev_notice(chip->dev, "%s gpio request fail(%d)\n",
				      __func__, ret);
		return ret;
	}
	chip->irq = gpio_to_irq(chip->intr_gpio);
	if (chip->irq < 0) {
		dev_notice(chip->dev, "%s gpio2irq fail(%d)\n",
				      __func__, chip->irq);
		return chip->irq;
	}
	dev_info(chip->dev, "%s irq = %d\n", __func__, chip->irq);

	/* Request threaded IRQ */
if (ret ==111111){
	ret = devm_request_threaded_irq(chip->dev, chip->irq, NULL,
					eta6953_irq_handler,
					IRQF_TRIGGER_FALLING | IRQF_ONESHOT,//IRQF_TRIGGER_FALLING //IRQF_TRIGGER_RISING
					devm_kasprintf(chip->dev, GFP_KERNEL,
					"eta6953_irq.%s", dev_name(chip->dev)),
					chip);
}
	if (ret < 0) {
		dev_notice(chip->dev, "%s request threaded irq fail(%d)\n",
				      __func__, ret);
		return ret;
	}
	device_init_wakeup(chip->dev, true);

	return ret;
}

static int eta6953_parse_dt(struct eta6953_chip *chip)
{
	int ret = 0;//, irqnum = 0;
	struct device_node *parent_np = chip->dev->of_node, *np = NULL;
	struct eta6953_desc *desc = NULL;

	dev_info(chip->dev, "%s\n", __func__);

	chip->desc = &eta6953_default_desc;

	if (!parent_np) {
		dev_notice(chip->dev, "%s no device node\n", __func__);
		return -EINVAL;
	}
	np = of_get_child_by_name(parent_np, "eta6953");
	if (!np) {
		dev_info(chip->dev, "%s no eta6953 device node\n", __func__);
		np = parent_np;
	}

	desc = devm_kzalloc(chip->dev, sizeof(*desc), GFP_KERNEL);
	if (!desc)
		return -ENOMEM;
	memcpy(desc, &eta6953_default_desc, sizeof(*desc));

	ret = of_property_read_string(np, "chg_name", &desc->chg_name);
	if (ret < 0)
		dev_info(chip->dev, "%s no chg_name(%d)\n", __func__, ret);

	ret = of_property_read_string(np, "chg_alias_name",
				      &chip->chg_props.alias_name);
	if (ret < 0) {
		dev_info(chip->dev, "%s no chg_alias_name(%d)\n",
				    __func__, ret);
		chip->chg_props.alias_name = "eta6953_chg";
	}
	dev_info(chip->dev, "%s name = %s, alias name = %s\n", __func__,
			    desc->chg_name, chip->chg_props.alias_name);

	if (strcmp(desc->chg_name, "secondary_chg") == 0)
		chip->enter_shipping_mode = true;

#if !defined(CONFIG_MTK_GPIO) || defined(CONFIG_MTK_GPIOLIB_STAND)
	ret = of_get_named_gpio(parent_np, "rt,intr_gpio", 0);
	if (ret < 0) {
		dev_notice(chip->dev, "%s no rt,intr_gpio(%d)\n",
				      __func__, ret);
		return ret;
	} else
		chip->intr_gpio = ret;

	ret = of_get_named_gpio(parent_np, "rt,ceb_gpio", 0);
	if (ret < 0) {
		dev_info(chip->dev, "%s no rt,ceb_gpio(%d)\n",
				    __func__, ret);
		chip->ceb_gpio = U32_MAX;
	} else
		chip->ceb_gpio = ret;
#else
	ret = of_property_read_u32(parent_np, "rt,intr_gpio_num",
				   &chip->intr_gpio);
	if (ret < 0) {
		dev_notice(chip->dev, "%s no rt,intr_gpio_num(%d)\n",
				      __func__, ret);
		return ret;
	}

	ret = of_property_read_u32(parent_np, "rt,ceb_gpio_num",
				   &chip->ceb_gpio);
	if (ret < 0) {
		dev_info(chip->dev, "%s no rt,ceb_gpio_num(%d)\n",
				    __func__, ret);
		chip->ceb_gpio = U32_MAX;
	}
#endif
	dev_info(chip->dev, "%s intr_gpio = %u, ceb_gpio = %u\n",
			    __func__, chip->intr_gpio, chip->ceb_gpio);

	if (chip->ceb_gpio != U32_MAX) {
		ret = devm_gpio_request_one(
				chip->dev, chip->ceb_gpio, GPIOF_DIR_OUT,
				devm_kasprintf(chip->dev, GFP_KERNEL,
				"eta6953_ceb_gpio.%s", dev_name(chip->dev)));
		if (ret < 0) {
			dev_notice(chip->dev, "%s gpio request fail(%d)\n",
					      __func__, ret);
			return ret;
		}
	}

	/* Charger parameter */
	ret = of_property_read_u32(np, "vac_ovp", &desc->vac_ovp);
	if (ret < 0)
		dev_info(chip->dev, "%s no vac_ovp(%d)\n", __func__, ret);

	ret = of_property_read_u32(np, "mivr", &desc->mivr);
	if (ret < 0)
		dev_info(chip->dev, "%s no mivr(%d)\n", __func__, ret);

	ret = of_property_read_u32(np, "aicr", &desc->aicr);
	if (ret < 0)
		dev_info(chip->dev, "%s no aicr(%d)\n", __func__, ret);

	ret = of_property_read_u32(np, "cv", &desc->cv);
	if (ret < 0)
		dev_info(chip->dev, "%s no cv(%d)\n", __func__, ret);

	ret = of_property_read_u32(np, "ichg", &desc->ichg);
	if (ret < 0)
		dev_info(chip->dev, "%s no ichg(%d)\n", __func__, ret);

	ret = of_property_read_u32(np, "ieoc", &desc->ieoc) < 0;
	if (ret < 0)
		dev_info(chip->dev, "%s no ieoc(%d)\n", __func__, ret);

	ret = of_property_read_u32(np, "safe_tmr", &desc->safe_tmr);
	if (ret < 0)
		dev_info(chip->dev, "%s no safe_tmr(%d)\n", __func__, ret);

	ret = of_property_read_u32(np, "wdt", &desc->wdt);
	if (ret < 0)
		dev_info(chip->dev, "%s no wdt(%d)\n", __func__, ret);

	ret = of_property_read_u32(np, "mivr_track", &desc->mivr_track);
	if (ret < 0)
		dev_info(chip->dev, "%s no mivr_track(%d)\n", __func__, ret);
	if (desc->mivr_track >= ETA6953_MIVRTRACK_MAX)
		desc->mivr_track = ETA6953_MIVRTRACK_VBAT_300MV;

	desc->en_safe_tmr = of_property_read_bool(np, "en_safe_tmr");
	desc->en_te = of_property_read_bool(np, "en_te");
	desc->en_jeita = of_property_read_bool(np, "en_jeita");
	desc->ceb_invert = of_property_read_bool(np, "ceb_invert");
	desc->dis_i2c_tout = of_property_read_bool(np, "dis_i2c_tout");
	desc->en_qon_rst = of_property_read_bool(np, "en_qon_rst");
	desc->auto_aicr = of_property_read_bool(np, "auto_aicr");

	chip->desc = desc;

	desc->en_te = 1;

	return 0;
}

static int eta6953_check_chg(struct eta6953_chip *chip)
{
	int ret = 0;
	u8 regval = 0;

	dev_info(chip->dev, "%s\n", __func__);

	ret = eta6953_i2c_read_byte(chip, ETA6953_REG_VBUSGD, &regval);
	if (ret < 0)
		return ret;

	//if (regval & ETA6953_ST_VBUSGD_MASK)
	//	eta6953_vbus_gd_irq_handler(chip);


	ret = eta6953_i2c_read_byte(chip, ETA6953_REG_CHGDONE, &regval);
	if (ret < 0)
		return ret;

	//if ((regval & ETA6953_ST_CHGDONE_MASK) == (ETA6953_ICSTAT_CHGDONE << ETA6953_ST_CHGDONE_SHIFT))
	//	eta6953_chg_done_irq_handler(chip);
	//else
	//{
	//	ret = eta6953_i2c_read_byte(chip, ETA6953_REG_CHGRDY, &regval);
	//	if (ret < 0)
	//		return ret;
	//
	//	if (regval & ETA6953_ST_CHGRDY_MASK)
	//		eta6953_chg_rdy_irq_handler(chip);
	//}

	return ret;
}


static int eta6953_init_setting(struct eta6953_chip *chip)
{
	int ret = 0;
	struct eta6953_desc *desc = chip->desc;
	u8 evt[ETA6953_STREG_MAX] = {0};
	// unsigned int boot_mode = get_boot_mode();

	dev_info(chip->dev, "%s\n", __func__);

	/* Clear all IRQs */
	ret = eta6953_i2c_block_read(chip, ETA6953_REG_STATUS, ETA6953_STREG_MAX,
				    evt);
	if (ret < 0)
		dev_notice(chip->dev, "%s clear irq fail(%d)\n", __func__, ret);

	ret = __eta6953_set_vac_ovp(chip, desc->vac_ovp);
	if (ret < 0)
		dev_notice(chip->dev, "%s set vac ovp fail(%d)\n",
				      __func__, ret);

	ret = __eta6953_set_mivr(chip, desc->mivr);
	if (ret < 0)
		dev_notice(chip->dev, "%s set mivr fail(%d)\n", __func__, ret);

	ret = __eta6953_set_aicr(chip, desc->aicr);
	if (ret < 0)
		dev_notice(chip->dev, "%s set aicr fail(%d)\n", __func__, ret);

	ret = __eta6953_set_cv(chip, desc->cv);
	if (ret < 0)
		dev_notice(chip->dev, "%s set cv fail(%d)\n", __func__, ret);

	ret = __eta6953_set_ichg(chip, desc->ichg);
	if (ret < 0)
		dev_notice(chip->dev, "%s set ichg fail(%d)\n", __func__, ret);

	ret = __eta6953_set_ieoc(chip, desc->ieoc);
	if (ret < 0)
		dev_notice(chip->dev, "%s set ieoc fail(%d)\n", __func__, ret);

	ret = __eta6953_set_safe_tmr(chip, desc->safe_tmr);
	if (ret < 0)
		dev_notice(chip->dev, "%s set safe tmr fail(%d)\n",
				      __func__, ret);

	ret = __eta6953_set_mivrtrack(chip, desc->mivr_track);
	if (ret < 0)
		dev_notice(chip->dev, "%s set mivrtrack fail(%d)\n",
				      __func__, ret);

	ret = __eta6953_enable_safe_tmr(chip, desc->en_safe_tmr);
	if (ret < 0)
		dev_notice(chip->dev, "%s en safe tmr fail(%d)\n",
				      __func__, ret);

	ret = __eta6953_enable_te(chip, desc->en_te);
	if (ret < 0)
		dev_notice(chip->dev, "%s en te fail(%d)\n", __func__, ret);

	//eta6953_switch_dpdm_event_detection(chip,false);

	/*
	if ((boot_mode == KERNEL_POWER_OFF_CHARGING_BOOT ||
	     boot_mode == LOW_POWER_OFF_CHARGING_BOOT) &&
	     strcmp(desc->chg_name, "primary_chg") == 0) {
		dev_info(chip->dev,
		"%s do not HZ=1 and CHG_EN=0 for primary_chg when KPOC\n",
		__func__);
		return 0;
	}*/

	/*
	 * Customization for MTK platform
	 * Primary charger: CHG_EN=1 at rt9471_plug_in()
	 * Secondary charger: CHG_EN=1 at needed, e.x.: PE10, PE20, etc...
	 */
	ret = __eta6953_enable_chg(chip, false);
	if (ret < 0)
		dev_notice(chip->dev, "%s dis chg fail(%d)\n", __func__, ret);

	/*
	 * Customization for MTK platform
	 * Primary charger: HZ=0 at sink vbus 5V with TCPC enabled
	 * Secondary charger: HZ=0 at needed, e.x.: PE10, PE20, etc...
	 */
//#ifndef CONFIG_TCPC_CLASS
	if (strcmp(desc->chg_name, "secondary_chg") == 0) {
//#endif /* CONFIG_TCPC_CLASS */
		ret = __eta6953_enable_hz(chip, true);
		if (ret < 0)
			dev_notice(chip->dev, "%s en hz fail(%d)\n",
					      __func__, ret);
//#ifndef CONFIG_TCPC_CLASS
	}
//#endif /* CONFIG_TCPC_CLASS */

	return 0;
}

static int eta6953_reset_register(struct eta6953_chip *chip)
{
	int ret = 0;

	dev_info(chip->dev, "%s\n", __func__);

	ret = eta6953_set_bit(chip, ETA6953_REG_REGRST, ETA6953_REGRST_MASK);
	if (ret < 0)
		return ret;

	return __eta6953_set_wdt(chip, 0);
}

static bool eta6953_check_devinfo(struct eta6953_chip *chip)
{
	int ret = 0;

	ret = i2c_smbus_read_byte_data(chip->client, ETA6953_REG_DEVID);
	if (ret < 0) {
		dev_notice(chip->dev, "%s get devinfo fail(%d)\n",
				      __func__, ret);
		return false;
	}
	chip->dev_id = (ret & ETA6953_DEVID_MASK) >> ETA6953_DEVID_SHIFT;
	chip->dev_rev = (ret & ETA6953_DEVREV_MASK) >> ETA6953_DEVREV_SHIFT;
	dev_info(chip->dev, "%s id = 0x%02X, devid rev = 0x%02X\n",
			    __func__, chip->dev_id, chip->dev_rev);
	switch (chip->dev_id) {
	case ETA6963_DEVID:
	case ETA6953_DEVID:
	case SGM41512_DEVID:
		break;
	default:
		dev_notice(chip->dev, "%s incorrect devid 0x%02X\n",
				      __func__, chip->dev_id);
		return false;
	}

	return true;
}

static int eta6953_enable_charging(struct charger_device *chg_dev, bool en);
static int eta6953_plug_in(struct charger_device *chg_dev)
{
	struct eta6953_chip *chip = dev_get_drvdata(&chg_dev->dev);

	dev_info(chip->dev, "%s\n", __func__);

	/* Enable charging */
	return eta6953_enable_charging(chg_dev, true);
}

static int eta6953_plug_out(struct charger_device *chg_dev)
{
	struct eta6953_chip *chip = dev_get_drvdata(&chg_dev->dev);

	dev_info(chip->dev, "%s\n", __func__);

	/* Disable charging */
	return eta6953_enable_charging(chg_dev, false);
}

static int eta6953_enable_charging(struct charger_device *chg_dev, bool en)
{
	int ret = 0;
	struct eta6953_chip *chip = dev_get_drvdata(&chg_dev->dev);

	dev_info(chip->dev, "%s en = %d\n", __func__, en);

	if (en) {
		ret = __eta6953_set_wdt(chip, chip->desc->wdt);
		if (ret < 0) {
			dev_notice(chip->dev, "%s set wdt fail(%d)\n",
					      __func__, ret);
			return ret;
		}
	}

	ret = __eta6953_enable_chg(chip, en);
	if (ret < 0) {
		dev_notice(chip->dev, "%s en chg fail(%d)\n", __func__, ret);
		return ret;
	}

	if (!en) {
		ret = __eta6953_set_wdt(chip, 0);
		if (ret < 0)
			dev_notice(chip->dev, "%s set wdt fail(%d)\n",
					      __func__, ret);
	}

	return ret;
}

static int eta6953_is_charging_enabled(struct charger_device *chg_dev, bool *en)
{
	struct eta6953_chip *chip = dev_get_drvdata(&chg_dev->dev);

	return __eta6953_is_chg_enabled(chip, en);
}

static int eta6953_is_charging_done(struct charger_device *chg_dev, bool *done)
{
	int ret = 0;
	struct eta6953_chip *chip = dev_get_drvdata(&chg_dev->dev);
	enum eta6953_ic_stat ic_stat = ETA6953_ICSTAT_SLEEP;

	ret = __eta6953_get_ic_stat(chip, &ic_stat);
	if (ret < 0)
		return ret;
	*done = (ic_stat == ETA6953_ICSTAT_CHGDONE);

	return ret;
}

static int eta6953_set_vindpm(struct charger_device *chg_dev, u32 uV)
{
	struct eta6953_chip *chip = dev_get_drvdata(&chg_dev->dev);

	return __eta6953_set_mivr(chip, uV);
}

static int eta6953_get_vindpm_state(struct charger_device *chg_dev, bool *in_loop)
{
	struct eta6953_chip *chip = dev_get_drvdata(&chg_dev->dev);

	return eta6953_i2c_test_bit(chip, ETA6953_REG_ST_MIVR,
				   ETA6953_ST_MIVR_SHIFT, in_loop);
}

static int eta6953_get_iindpm(struct charger_device *chg_dev, u32 *uA)
{
	struct eta6953_chip *chip = dev_get_drvdata(&chg_dev->dev);

	return __eta6953_get_aicr(chip, uA);
}

static int eta6953_set_iindpm(struct charger_device *chg_dev, u32 uA)
{
	struct eta6953_chip *chip = dev_get_drvdata(&chg_dev->dev);

	return __eta6953_set_aicr(chip, uA);
}

static int eta6953_get_min_iindpm(struct charger_device *chg_dev, u32 *uA)
{
	*uA = eta6953_closest_value(ETA6953_AICR_MIN, ETA6953_AICR_MAX,
				   ETA6953_AICR_STEP, 0);
	return 0;
}

static int eta6953_get_vreg(struct charger_device *chg_dev, u32 *uV)
{
	struct eta6953_chip *chip = dev_get_drvdata(&chg_dev->dev);

	return __eta6953_get_cv(chip, uV);
}

static int eta6953_set_vreg(struct charger_device *chg_dev, u32 uV)
{
	struct eta6953_chip *chip = dev_get_drvdata(&chg_dev->dev);

	return __eta6953_set_cv(chip, uV);
}

static int eta6953_get_ichg(struct charger_device *chg_dev, u32 *uA)
{
	struct eta6953_chip *chip = dev_get_drvdata(&chg_dev->dev);

	return __eta6953_get_ichg(chip, uA);
}

static int eta6953_set_ichg(struct charger_device *chg_dev, u32 uA)
{
	struct eta6953_chip *chip = dev_get_drvdata(&chg_dev->dev);

	return __eta6953_set_ichg(chip, uA);
}

static int eta6953_get_min_ichg(struct charger_device *chg_dev, u32 *uA)
{
	*uA = eta6953_closest_value(ETA6953_ICHG_MIN, ETA6953_ICHG_MAX,
				   ETA6953_ICHG_STEP, 0);
	return 0;
}

static int eta6953_get_iterm(struct charger_device *chg_dev, u32 *uA)
{
	struct eta6953_chip *chip = dev_get_drvdata(&chg_dev->dev);

	return __eta6953_get_ieoc(chip, uA);
}

static int eta6953_set_iterm(struct charger_device *chg_dev, u32 uA)
{
	struct eta6953_chip *chip = dev_get_drvdata(&chg_dev->dev);

	return __eta6953_set_ieoc(chip, uA);
}

static int eta6953_enable_term(struct charger_device *chg_dev, bool en)
{
	struct eta6953_chip *chip = dev_get_drvdata(&chg_dev->dev);

	return __eta6953_enable_te(chip, en);
}

static int eta6953_kick_wdt(struct charger_device *chg_dev)
{
	struct eta6953_chip *chip = dev_get_drvdata(&chg_dev->dev);

	return __eta6953_kick_wdt(chip);
}

static int eta6953_event(struct charger_device *chg_dev, u32 event, u32 args)
{
	struct eta6953_chip *chip = dev_get_drvdata(&chg_dev->dev);

	dev_info(chip->dev, "%s event = %d\n", __func__, event);

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

static int eta6953_enable_powerpath(struct charger_device *chg_dev, bool en)
{
	struct eta6953_chip *chip = dev_get_drvdata(&chg_dev->dev);

	dev_info(chip->dev, "%s en = %d\n", __func__, en);

	return __eta6953_enable_hz(chip, !en);
}

static int eta6953_enable_chip(struct charger_device *chg_dev, bool en)
{
	struct eta6953_chip *chip = dev_get_drvdata(&chg_dev->dev);

	dev_info(chip->dev, "%s en = %d\n", __func__, en);

	return __eta6953_enable_hz(chip, !en);
}

static int eta6953_is_powerpath_enabled(struct charger_device *chg_dev, bool *en)
{
	int ret = 0;
	struct eta6953_chip *chip = dev_get_drvdata(&chg_dev->dev);

	ret = __eta6953_is_hz_enabled(chip, en);
	*en = !*en;

	return ret;
}

static int eta6953_is_chip_enabled(struct charger_device *chg_dev, bool *en)
{
	int ret = 0;
	struct eta6953_chip *chip = dev_get_drvdata(&chg_dev->dev);

	ret = __eta6953_is_hz_enabled(chip, en);
	*en = !*en;

	return ret;
}

static int eta6953_enable_safety_timer(struct charger_device *chg_dev, bool en)
{
	struct eta6953_chip *chip = dev_get_drvdata(&chg_dev->dev);

	return __eta6953_enable_safe_tmr(chip, en);
}

static int eta6953_is_safety_timer_enabled(struct charger_device *chg_dev,
					  bool *en)
{
	struct eta6953_chip *chip = dev_get_drvdata(&chg_dev->dev);

	return eta6953_i2c_test_bit(chip, ETA6953_REG_SAFETMR_EN,
				   ETA6953_SAFETMR_EN_SHIFT, en);
}

static int eta6953_enable_otg(struct charger_device *chg_dev, bool en)
{
	int ret = 0;
	struct eta6953_chip *chip = dev_get_drvdata(&chg_dev->dev);

	if (en) {
		//ret = __eta6953_set_wdt(chip, chip->desc->wdt);
		ret = __eta6953_set_wdt(chip, 0);
		if (ret < 0) {
			dev_notice(chip->dev, "%s set wdt fail(%d)\n",
					      __func__, ret);
			return ret;
		}

#ifdef CONFIG_TCPC_CLASS
		ret = __eta6953_enable_hz(chip, false);
		if (ret < 0)
			dev_notice(chip->dev, "%s dis hz fail(%d)\n",
					      __func__, ret);
#endif /* CONFIG_TCPC_CLASS */
	}

	ret = __eta6953_enable_otg(chip, en);
	if (ret < 0) {
		dev_notice(chip->dev, "%s en otg fail(%d)\n", __func__, ret);
		return ret;
	}

	if (!en) {
#ifdef CONFIG_TCPC_CLASS
		ret = __eta6953_enable_hz(chip, true);
		if (ret < 0)
			dev_notice(chip->dev, "%s en hz fail(%d)\n",
					      __func__, ret);
#endif /* CONFIG_TCPC_CLASS */

		ret = __eta6953_set_wdt(chip, 0);
		if (ret < 0)
			dev_notice(chip->dev, "%s set wdt fail(%d)\n",
					      __func__, ret);
	}

	return ret;
}

static int eta6953_set_boost_ilim(struct charger_device *chg_dev,
					  u32 uA)
{
	struct eta6953_chip *chip = dev_get_drvdata(&chg_dev->dev);

	return __eta6953_set_otgcc(chip, uA);
}


static int eta6953_dump_registers(struct charger_device *chg_dev)
{
	struct eta6953_chip *chip = dev_get_drvdata(&chg_dev->dev);

	return __eta6953_dump_registers(chip);
}

static int eta6953_run_iind(struct charger_device *chg_dev, u32 *uA)
{
	int ret = 0;
	struct eta6953_chip *chip = dev_get_drvdata(&chg_dev->dev);
	bool chg_mivr = false;
	u32 aicr = 0;

	dev_info(chip->dev, "%s chip_rev = %d\n", __func__, chip->chip_rev);

	ret = eta6953_i2c_test_bit(chip, ETA6953_REG_ST_MIVR, ETA6953_ST_MIVR_SHIFT,
				  &chg_mivr);
	if (ret < 0)
		return ret;
	if (!chg_mivr) {
		dev_info(chip->dev, "%s mivr stat not act\n", __func__);
		return ret;
	}

	/* Backup the aicr */
	ret = __eta6953_get_aicr(chip, &aicr);
	if (ret < 0)
		return ret;




	return ret;
}

static struct charger_ops eta6953_chg_ops = {
	/* cable plug in/out for primary charger */
	.plug_in = eta6953_plug_in,
	.plug_out = eta6953_plug_out,

	/* enable/disable charger */
	.enable = eta6953_enable_charging,
	.is_enabled = eta6953_is_charging_enabled,
	.is_charging_done = eta6953_is_charging_done,

	/* get/set minimun input voltage regulation */
	.set_mivr = eta6953_set_vindpm,
	.get_mivr_state = eta6953_get_vindpm_state,

	/* get/set input current */
	.get_input_current = eta6953_get_iindpm,
	.set_input_current = eta6953_set_iindpm,
	.get_min_input_current = eta6953_get_min_iindpm,

	/* get/set charging voltage */
	.get_constant_voltage = eta6953_get_vreg,
	.set_constant_voltage = eta6953_set_vreg,

	/* get/set charging current*/
	.get_charging_current = eta6953_get_ichg,
	.set_charging_current = eta6953_set_ichg,
	.get_min_charging_current = eta6953_get_min_ichg,

	/* get/set termination current */
	.get_eoc_current = eta6953_get_iterm,
	.set_eoc_current = eta6953_set_iterm,

	/* enable te */
	.enable_termination = eta6953_enable_term,

	/* kick wdt */
	.kick_wdt = eta6953_kick_wdt,

	.event = eta6953_event,

	/* enable/disable powerpath for primary charger */
	.enable_powerpath = eta6953_enable_powerpath,
	.is_powerpath_enabled = eta6953_is_powerpath_enabled,

	/* enable/disable chip for secondary charger */
	.enable_chip = eta6953_enable_chip,
	.is_chip_enabled = eta6953_is_chip_enabled,

	/* enable/disable charging safety timer */
	.enable_safety_timer = eta6953_enable_safety_timer,
	.is_safety_timer_enabled = eta6953_is_safety_timer_enabled,

	/* OTG */
	.enable_otg = eta6953_enable_otg,
	.set_boost_current_limit = eta6953_set_boost_ilim,

	.dump_registers = eta6953_dump_registers,

	/* new features for chip_rev >= 4, AICC */
	.run_aicl = eta6953_run_iind,
};
#if 1
static ssize_t shipping_mode_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	int ret = 0, tmp = 0;
	struct eta6953_chip *chip = dev_get_drvdata(dev);

	ret = kstrtoint(buf, 10, &tmp);
	if (ret < 0) {
		dev_notice(dev, "%s parsing number fail(%d)\n", __func__, ret);
		return -EINVAL;
	}
	if (tmp != 5526789)
		return -EINVAL;
	chip->enter_shipping_mode = true;
	/*
	 * Use kernel_halt() instead of kernel_power_off() to prevent
	 * the system from booting again while cable still plugged-in.
	 * But plug-out cable before AP WDT timeout, please.
	 */
	kernel_halt();

	return count;
}
static const DEVICE_ATTR_WO(shipping_mode);
#endif

static int eta6953_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	int ret = 0;
	struct eta6953_chip *chip = NULL;

	dev_info(&client->dev, "%s (%s)\n", __func__, ETA6953_DRV_VERSION);

	chip = devm_kzalloc(&client->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;
	chip->client = client;
	chip->dev = &client->dev;
	mutex_init(&chip->io_lock);
	chip->hidden_mode_cnt = 0;
	chip->chg_done_once = false;
	wakeup_source_init(&chip->buck_dwork_ws,
			   devm_kasprintf(chip->dev, GFP_KERNEL,
			   "eta6953_buck_dwork_ws.%s", dev_name(chip->dev)));

	chip->enter_shipping_mode = false;
	//init_completion(&chip->aicc_done);
	init_completion(&chip->pe_done);
	i2c_set_clientdata(client, chip);

	if (!eta6953_check_devinfo(chip)) {
		ret = -ENODEV;
		goto err_nodev;
	}

	ret = eta6953_parse_dt(chip);
	if (ret < 0) {
		dev_notice(chip->dev, "%s parse dt fail(%d)\n", __func__, ret);
		goto err_parse_dt;
	}

	ret = eta6953_reset_register(chip);
	if (ret < 0)
		dev_notice(chip->dev, "%s reset register fail(%d)\n",
				      __func__, ret);

	ret = eta6953_init_setting(chip);
	if (ret < 0) {
		dev_notice(chip->dev, "%s init fail(%d)\n", __func__, ret);
		goto err_init;
	}

	chip->chg_dev = charger_device_register(chip->desc->chg_name,
			chip->dev, chip, &eta6953_chg_ops, &chip->chg_props);
	if (IS_ERR_OR_NULL(chip->chg_dev)) {
		ret = PTR_ERR(chip->chg_dev);
		dev_notice(chip->dev, "%s register chg dev fail(%d)\n",
				      __func__, ret);
		goto err_register_chg_dev;
	}

	ret = eta6953_check_chg(chip);
	if (ret < 0) {
		dev_notice(chip->dev, "%s check chg(%d)\n", __func__, ret);
		goto err_check_chg;
	}

	ret = eta6953_register_irq(chip);
	if (ret < 0) {
		dev_notice(chip->dev, "%s register irq fail(%d)\n",
				      __func__, ret);
		goto err_register_irq;
	}

	if (ret < 0) {
		dev_notice(chip->dev, "%s init irq fail(%d)\n", __func__, ret);
		goto err_init_irq;
	}

	ret = device_create_file(chip->dev, &dev_attr_shipping_mode);
	if (ret < 0) {
		dev_notice(chip->dev, "%s create file fail(%d)\n",
				      __func__, ret);
		goto err_create_file;
	}

	__eta6953_dump_registers(chip);
	dev_info(chip->dev, "%s successfully\n", __func__);
	return 0;

err_create_file:
err_init_irq:
err_register_irq:
err_check_chg:
	charger_device_unregister(chip->chg_dev);
err_register_chg_dev:

err_init:

err_parse_dt:
err_nodev:
	mutex_destroy(&chip->io_lock);

	wakeup_source_trash(&chip->buck_dwork_ws);
	devm_kfree(chip->dev, chip);
	return ret;
}

static int eta6953_remove(struct i2c_client *client)
{
	struct eta6953_chip *chip = i2c_get_clientdata(client);

	dev_info(chip->dev, "%s\n", __func__);

	device_remove_file(chip->dev, &dev_attr_shipping_mode);
	charger_device_unregister(chip->chg_dev);
	disable_irq(chip->irq);

	mutex_destroy(&chip->io_lock);

	wakeup_source_trash(&chip->buck_dwork_ws);

	return 0;
}

static void eta6953_shutdown(struct i2c_client *client)
{
	int ret = 0;
	struct eta6953_chip *chip = i2c_get_clientdata(client);

	dev_info(chip->dev, "%s\n", __func__);

	charger_device_unregister(chip->chg_dev);
	disable_irq(chip->irq);

	eta6953_reset_register(chip);

	if (!chip->enter_shipping_mode)
		return;

	ret = __eta6953_enable_shipmode(chip, true);
	if (ret < 0)
		dev_notice(chip->dev, "%s enter shipping mode fail(%d)\n",
				      __func__, ret);
}

static const struct of_device_id eta6953_of_device_id[] = {
	{ .compatible = "eta,eta6953", },
	{ },
};
MODULE_DEVICE_TABLE(of, eta6953_of_device_id);

static const struct i2c_device_id eta6953_i2c_device_id[] = {
	{ "eta6953", 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, eta6953_i2c_device_id);

static struct i2c_driver eta6953_i2c_driver = {
	.driver = {
		.name = "eta6953",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(eta6953_of_device_id),
	},
	.probe = eta6953_probe,
	.remove = eta6953_remove,
	.shutdown = eta6953_shutdown,
	.id_table = eta6953_i2c_device_id,
};
module_i2c_driver(eta6953_i2c_driver);


MODULE_LICENSE("GPL");
MODULE_AUTHOR("Stuart Su <stuart_su@sg-micro.com>");
MODULE_DESCRIPTION("ETA6953 Charger Driver");
MODULE_VERSION(ETA6953_DRV_VERSION);

/*
 * Release Note
 * 1.0.0
 * (1) Initial released
 */
