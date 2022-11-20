#define LOG_TAG "LCM"

#ifndef BUILD_LK
#include <linux/string.h>
#include <linux/kernel.h>
#endif

#include "lcm_drv.h"

#ifdef BUILD_LK
#include <platform/upmu_common.h>
#include <platform/mt_gpio.h>
#include <platform/mt_i2c.h>
#include <platform/mt_pmic.h>
#include <string.h>
#elif defined(BUILD_UBOOT)
#include <asm/arch/mt_gpio.h>
#endif

#ifdef BUILD_LK
#define LCM_LOGI(string, args...)  dprintf(ALWAYS, "[LK/"LOG_TAG"]"string, ##args)
#define LCM_LOGD(string, args...)  dprintf(INFO, "[LK/"LOG_TAG"]"string, ##args)
#else
#define LCM_LOGI(fmt, args...)  pr_notice("[KERNEL/"LOG_TAG"]"fmt, ##args)
#define LCM_LOGD(fmt, args...)  pr_debug("[KERNEL/"LOG_TAG"]"fmt, ##args)
#endif

#define LCM_DSI_CMD_MODE                0
#define FRAME_WIDTH			(720)
#define FRAME_HEIGHT			(1600)

//prize-tangcong modify LCD size-20200331-start
#define LCM_PHYSICAL_WIDTH                  				(68000)
#define LCM_PHYSICAL_HEIGHT                  				(151000)
//prize-tangcong modify LCD size-20200331-end

#define REGFLAG_PORT_SWAP               0xFFFA
#define REGFLAG_DELAY                   0xFFFC
#define REGFLAG_UDELAY                  0xFFFB
#define REGFLAG_END_OF_TABLE            0xFFFD

#ifndef GPIO_LCM_RST
#define GPIO_LCM_RST		(GPIO45 | 0x80000000)
#endif

// ---------------------------------------------------------------------------
//  Local Variable
// ---------------------------------------------------------------------------
static struct LCM_UTIL_FUNCS lcm_util;


#define SET_RESET_PIN(v)	(lcm_util.set_reset_pin((v)))
#define MDELAY(n)		(lcm_util.mdelay(n))
#define UDELAY(n)		(lcm_util.udelay(n))

#define dsi_set_cmdq_V22(cmdq, cmd, count, ppara, force_update) \
	lcm_util.dsi_set_cmdq_V22(cmdq, cmd, count, ppara, force_update)
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update) \
	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update) \
		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd) lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums) \
		lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd) \
		lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size) \
		lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

#ifndef BUILD_LK
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/irq.h>
/* #include <linux/jiffies.h> */
/* #include <linux/delay.h> */
#include <linux/uaccess.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#endif

struct LCM_setting_table {
	unsigned int cmd;
	unsigned char count;
	unsigned char para_list[120];
};


static struct LCM_setting_table lcm_initialization_setting[] = {
/* fae for e11 initial code */
	{0xF0, 2,{0x5A, 0x59}},
	{0xF1, 2,{0xA5, 0xA6}},
	{0xB0, 30,{0x87, 0x86, 0x05, 0x04, 0x8A, 0x8B, 0x04, 0x05, 0x55, 0x55, 0x55, 0x55, 0x1B, 0x00, 0x00, 0x64, 0x00, 0x00, 0x0F, 0x05, 0x04, 0x03, 0x02, 0x01, 0x02, 0x03, 0x04, 0x00, 0x00, 0x00}},
	{0xB1, 29,{0x53, 0x42, 0x85, 0x00, 0x1B, 0x00, 0x00, 0x64, 0x00, 0x00, 0x04, 0x08, 0x54, 0x00, 0x00, 0x00, 0x44, 0x40, 0x02, 0x01, 0x40, 0x02, 0x01, 0x40, 0x02, 0x01, 0x40, 0x02, 0x01}},//OE3.08/DUTY45%
	{0xB2, 17,{0x54, 0x84, 0x85, 0x05, 0x40, 0x02, 0x01, 0x40, 0x02, 0x01, 0x05, 0x05, 0x54, 0x0C, 0x0C, 0x0D, 0x0B}},
	{0xB3, 31,{0x02, 0x0E, 0x0B, 0x0E, 0x0B, 0x26, 0x26, 0x91, 0xA2, 0x33, 0x44, 0x00, 0x26, 0x00, 0x18, 0x01, 0x02, 0x08, 0x20, 0x30, 0x08, 0x09, 0x44, 0x20, 0x40, 0x20, 0x40, 0x08, 0x09, 0x22, 0x33}},
	//, 0xFORWORD, 0xSCAN
	{0xB4, 28,{0x02, 0x02, 0x00, 0x00, 0xE3, 0x09, 0x22, 0x00, 0x22, 0x0D, 0x0F, 0x11, 0x13, 0x05, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0xFF, 0xFF, 0xFC, 0x00, 0x00, 0x00}},
	{0xB5, 28,{0x00, 0x00, 0x00, 0x00, 0xE3, 0x08, 0x22, 0x00, 0x22, 0x0C, 0x0E, 0x10, 0x12, 0x04, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0xFF, 0xFF, 0xFC, 0x00, 0x00, 0x00}},
	//, 0xBACKWORD, 0xSCAN
	//R36, 0x03
	//RB4, 0x02, 0x02, 0x00, 0x00, 0xE3, 0x04, 0x00, 0x22, 0x22, 0x12, 0x10, 0x0E, 0x0C, 0x08, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0xFF, 0xFF, 0xFC, 0x00, 0x00, 0x00
	//RB5, 0x00, 0x00, 0x00, 0x00, 0xE3, 0x05, 0x00, 0x22, 0x22, 0x13, 0x11, 0x0F, 0x0D, 0x09, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0xFF, 0xFF, 0xFC, 0x00, 0x00, 0x00
	{0xB8, 24,{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xA0, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xA0}},
	//RB9, 0x99, 0x11, 0x01, 0xFF, 0x//ID
	//RBA, 0x62, 0x62//OTPVCOM需要屏蒿	
	{0xBB, 13,{0x01, 0x05, 0x09, 0x11, 0x0D, 0x19, 0x1D, 0x55, 0x25, 0x69, 0x00, 0x21, 0x25}},
	{0xBC, 14,{0x00, 0x00, 0x00, 0x00, 0x02, 0x20, 0xFF, 0x00, 0x03, 0x24, 0x01, 0x73, 0x33, 0x00}},
	{0xBD, 10,{0xE9, 0x02, 0x4E, 0xCF, 0x72, 0xA4, 0x08, 0x44, 0xAE, 0x15}},
	{0xBE, 10,{0x68, 0x7C, 0x64, 0x3C, 0x0C, 0x77, 0x43, 0x07, 0x0E, 0x0E}},//VOP5.6/VGH13/VGL-13
	{0xBF, 8,{0x07, 0x25, 0x07, 0x25, 0x7F, 0x00, 0x11, 0x04}},
	{0xC0, 9,{0x10, 0x00, 0x1F, 0xFF, 0xFF, 0xFF, 0x00, 0xFF, 0x00}},
	{0xC1, 19,{0xC0, 0x20, 0x20, 0x96, 0x04, 0x30, 0x30, 0x04, 0x2A, 0x40, 0x36, 0x00, 0x07, 0xCF, 0xFF, 0xFF, 0xC0, 0x00, 0xC0}},//LONGV
	{0xC2, 1,{0x00}},
	{0xC3, 9,{0x06, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0x00, 0x81, 0x01}},
	{0xC4, 10,{0x84, 0x01, 0x2B, 0x41, 0x00, 0x3C, 0x00, 0x03, 0x03, 0x2E}},
	{0xC5, 11,{0x03, 0x1C, 0xC0, 0xC0, 0x40, 0x10, 0x42, 0x44, 0x0F, 0x0F, 0x14}},
	{0xC6, 10,{0x87, 0x96, 0x2A, 0x29, 0x29, 0x33, 0x7F, 0x34, 0x08, 0x04}},
	//GAMMA2.2
	{0xC7, 22,{0xFC, 0xC5, 0xA2, 0x88, 0x58, 0x38, 0x06, 0x57, 0x1F, 0xF1, 0xC4, 0x91, 0xEA, 0xBD, 0xA2, 0x77, 0x60, 0x3C, 0x1A, 0x7F, 0xC0, 0x00}},
	{0xC8, 22,{0xFC, 0xC5, 0xA2, 0x88, 0x58, 0x38, 0x06, 0x57, 0x1F, 0xF1, 0xC4, 0x91, 0xEA, 0xBD, 0xA2, 0x77, 0x60, 0x3C, 0x1A, 0x7F, 0xC0, 0x00}},
	{0xCB, 1,{0x00}},
	{0xD0, 5,{0x80, 0x0D, 0xFF, 0x0F, 0x62}},
	{0xD2, 1,{0x42}},
	{0xF6, 1,{0x3F}},    /* 回读Rf6后再加0x10后写值给回F6, Temp_F6, lcm_initialization_setting[26].para_list[0] */
	{0xFA, 3,{0x45, 0x93, 0x01}},
	{0xFE, 4,{0xFF, 0xFF, 0xFF, 0x40}},//64M
	{0xE0, 26,{0x30, 0x00, 0x80, 0x88, 0x11, 0x3F, 0x22, 0x62, 0xDF, 0xA0, 0x04, 0xCC, 0x01, 0xFF, 0xF6, 0xFF, 0xF0, 0xFD, 0xFF, 0xFD, 0xF8, 0xF5, 0xFC, 0xFC, 0xFD, 0xFF}},
	{0xE1, 23,{0xEF, 0xFE, 0xFE, 0xFE, 0xFE, 0xEE, 0xF0, 0x20, 0x33, 0xFF, 0x00, 0x00, 0x6A, 0x90, 0xC0, 0x0D, 0x6A, 0xF0, 0x3E, 0xFF, 0x00, 0x06, 0x40}},//20KHZ
	{0xF1, 2,{0x5A, 0x59}},
	{0xF0, 2,{0xA5, 0xA6}},
//	{0x53, 1,{0x2C}},				/* fae */
	//R55, 0x00
	{0x35, 1,{0x00}},
	{0x11, 0,{}},
	{REGFLAG_DELAY, 120,{}},
	{0x29, 0,{}},
	{REGFLAG_DELAY, 20, {}},
	{0x26,1,{0x01}},//鍑虹潯鐪犲鍔犵殑浠ｇ爜
	
	{REGFLAG_END_OF_TABLE, 0x00, {} }
};

static struct LCM_setting_table lcm_deep_sleep_mode_in_setting[] = {

{0x26, 1, {0x08}},
	// Display off sequence
	{0x28, 0, {0x00}},
	{REGFLAG_DELAY, 10, {}},
	// Sleep Mode On
	{0x10, 0, {0x00}},
	{REGFLAG_DELAY, 120, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}},
};

static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
	unsigned int i;
	for(i = 0; i < count; i++) {
		unsigned int cmd;
		cmd = table[i].cmd;
		switch (cmd) {
			case REGFLAG_DELAY :
				MDELAY(table[i].count);
				break;
			case REGFLAG_END_OF_TABLE :
				break;
			default:
				dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
		}
	}
}

// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------
static void lcm_set_util_funcs(const struct LCM_UTIL_FUNCS *util)
{
	memcpy(&lcm_util, util, sizeof(struct LCM_UTIL_FUNCS));
}
/*prize by anhengxuan,add 90HZ function,20220415,begin*/
#ifdef CONFIG_MTK_HIGH_FRAME_RATE
static void lcm_dfps_int(struct LCM_DSI_PARAMS *dsi)
{
	struct dfps_info *dfps_params = dsi->dfps_params;

	dsi->dfps_enable = 1;
	dsi->dfps_default_fps = 6000;/*real fps * 100, to support float*/
	dsi->dfps_def_vact_tim_fps = 9000;/*real vact timing fps * 100*/

	/*traversing array must less than DFPS_LEVELS*/
	/*DPFS_LEVEL0*/
	dfps_params[0].level = DFPS_LEVEL0;
	dfps_params[0].fps = 6000;/*real fps * 100, to support float*/
	dfps_params[0].vact_timing_fps = 9000;/*real vact timing fps * 100*/
	/*if mipi clock solution*/
	/*dfps_params[0].PLL_CLOCK = xx;*/
	/*dfps_params[0].data_rate = xx; */
	/*if HFP solution*/
	/*dfps_params[0].horizontal_frontporch = xx;*/
	dfps_params[0].vertical_frontporch = 1060;
	//dfps_params[0].vertical_frontporch_for_low_power = 1270;

	/*if need mipi hopping params add here*/
	dfps_params[0].dynamic_switch_mipi = 1;
	dfps_params[0].PLL_CLOCK_dyn = 425;
	dfps_params[0].horizontal_frontporch_dyn = 48;
	dfps_params[0].vertical_frontporch_dyn = 1060;
	//dfps_params[0].vertical_frontporch_for_low_power_dyn = 1270;

	/*DPFS_LEVEL1*/
	dfps_params[1].level = DFPS_LEVEL1;
	dfps_params[1].fps = 9000;/*real fps * 100, to support float*/
	dfps_params[1].vact_timing_fps = 9000;/*real vact timing fps * 100*/
	/*if mipi clock solution*/
	/*dfps_params[1].PLL_CLOCK = xx;*/
	/*dfps_params[1].data_rate = xx; */
	/*if HFP solution*/
	/*dfps_params[1].horizontal_frontporch = xx;*/
	dfps_params[1].vertical_frontporch = 150;
	//dfps_params[1].vertical_frontporch_for_low_power = 1270;

	/*if need mipi hopping params add here*/
	dfps_params[1].dynamic_switch_mipi = 0;
	dfps_params[1].PLL_CLOCK_dyn = 425;
	dfps_params[1].horizontal_frontporch_dyn = 48;
	dfps_params[1].vertical_frontporch_dyn = 150;
	//dfps_params[1].vertical_frontporch_for_low_power_dyn = 1270;

	dsi->dfps_num = 2;
}
#endif
static void lcm_get_params(struct LCM_PARAMS *params)
{
	memset(params, 0, sizeof(struct LCM_PARAMS));

	params->type = LCM_TYPE_DSI;

	params->width = FRAME_WIDTH;
	params->height = FRAME_HEIGHT;

//	params->virtual_width = VIRTUAL_WIDTH;
//	params->virtual_height = VIRTUAL_HEIGHT;

#if (LCM_DSI_CMD_MODE)
	params->dsi.mode            = CMD_MODE;
#else
	params->dsi.mode            = BURST_VDO_MODE;
#endif

	/* The following defined the fomat for data coming from LCD engine. */
	params->dsi.data_format.color_order	= LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.trans_seq	= LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding		= LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format		= LCM_DSI_FORMAT_RGB888;

	/* Highly depends on LCD driver capability */
	/* video mode timing */
	params->dsi.PS                                  = LCM_PACKED_PS_24BIT_RGB888;

	params->dsi.vertical_sync_active                = 4;
	params->dsi.vertical_backporch                  = 32;
	params->dsi.vertical_frontporch                 = 150;
	params->dsi.vertical_active_line                = FRAME_HEIGHT;
	
	params->dsi.horizontal_sync_active              = 4;
	params->dsi.horizontal_backporch                = 48;
	params->dsi.horizontal_frontporch               = 48;
	params->dsi.horizontal_active_pixel             = FRAME_WIDTH;
	params->dsi.LANE_NUM                            = LCM_FOUR_LANE;


	params->dsi.PLL_CLOCK                           = 279; // 425;//250

    //prize-tangcong modify LCD size-20200331-start
	params->physical_width = LCM_PHYSICAL_WIDTH/1000;
	params->physical_height = LCM_PHYSICAL_HEIGHT/1000;
	//prize-tangcong modify LCD size-20200331-end
	params->dsi.ssc_disable                         = 1;
	params->dsi.ssc_range                           = 4;

	params->dsi.HS_TRAIL                            = 15;
	params->dsi.noncont_clock                       = 1;
	params->dsi.noncont_clock_period                = 1;

#if 1
	/* ESD check function */
	params->dsi.esd_check_enable                    = 1;
	params->dsi.customization_esd_check_enable      = 1;
	//params->dsi.clk_lp_per_line_enable              = 1;
	params->dsi.lcm_esd_check_table[0].cmd          = 0x0A;
	params->dsi.lcm_esd_check_table[0].count        = 1;
	params->dsi.lcm_esd_check_table[0].para_list[0] = 0x9C;
#endif

#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
	params->round_corner_en = 1;
	params->full_content = 0;
	params->corner_pattern_width = 1080;
	params->corner_pattern_height = 32;
	params->corner_pattern_height_bot = 32;
#endif
/*prize by anhengxuan,add 90HZ function,20220415,begin*/
#ifdef CONFIG_MTK_HIGH_FRAME_RATE
	/****DynFPS start****/
	lcm_dfps_int(&(params->dsi));
	/****DynFPS end****/
#endif
/*prize by anhengxuan,add 90HZ function,20220415,end*/
}

static void lcm_init_power(void)
{
	//display_bias_enable();
	printk("%s\n",__func__);
}

static void lcm_suspend_power(void)
{
	//display_bias_disable();
	printk("%s\n",__func__);
}

static void lcm_resume_power(void)
{
	//SET_RESET_PIN(0);
	//display_bias_enable();
	printk("%s\n",__func__);
}

//绠�鍗曠殑鎶婂瓧绗︿覆杞负鏁板瓧
static unsigned char char1Tonum(unsigned char ch)
{
    if((ch>='0')&&(ch<='9'))
        return ch - '0';
    else if ((ch>='a')&&(ch<='f'))
        return ch - 'a' + 10; 
    else if ((ch>='A')&&(ch<='F'))
        return ch - 'A' + 10; 
    else
     return 0xff;
}

static unsigned char char2Tonum(unsigned char hch, unsigned char lch)
{
    return ((char1Tonum(hch) << 4) | char1Tonum(lch));
}

static void kernel_F6_reg_update(void)
{
	
	char *r = NULL;
	unsigned char kernel_F6_buf[3] = {0};
	unsigned char kernel_F6_real = 0x0;
	static bool update = false;
	if(update)
		return;
	update = true;
    r = strstr(saved_command_line,"androidboot.Temp_F6=0x");
    //printk("lcm_resume r = %s\n",r);//r = androidboot.Temp_F6=0x30 ramoops.mem_address=0x47c90000 ramoops.mem_size=0xe0000
    snprintf(kernel_F6_buf, 3, "%s",(r+22));
    //printk("lcm_resume kernel_F6_buf = %s\n",kernel_F6_buf); //lcm_resume kernel_F6_buf = 0x30
    //printk("lcm_resume kernel_F6_buf = %x\n",kernel_F6_buf[0]); //kernel_F6_buf = 33 3
    //printk("lcm_resume kernel_F6_buf = %x\n",kernel_F6_buf[1]); //kernel_F6_buf = 30 0
    //printk("lcm_resume kernel_F6_buf = %x\n",kernel_F6_buf[2]); //kernel_F6_buf = 0  0
    kernel_F6_real = char2Tonum(kernel_F6_buf[0], kernel_F6_buf[1]);
    //printk("lcm_resume kernel_F6_buf = %x\n",kernel_F6_real);
	printk("%s, update=%d, old=0x%x,kernel_F6_real=0x%x\n",__func__,update,lcm_initialization_setting[26].para_list[0],kernel_F6_real);
    lcm_initialization_setting[26].para_list[0] = kernel_F6_real;
}

static void lcm_init(void)
{
	display_ldo18_enable(1);
	display_bias_enable_v(6000);
	MDELAY(10);
	printk("%s\n",__func__);


	mt_dsi_pinctrl_set(LCM_RESET_PIN_NO, 1);
	//SET_RESET_PIN(1);
	MDELAY(10);
	mt_dsi_pinctrl_set(LCM_RESET_PIN_NO, 0);
	//SET_RESET_PIN(0);
	MDELAY(10);
	mt_dsi_pinctrl_set(LCM_RESET_PIN_NO, 1);
	//SET_RESET_PIN(1);
	MDELAY(30);

	kernel_F6_reg_update();
	push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
}

static void lcm_suspend(void)
{
	printk("%s\n",__func__);

	push_table(lcm_deep_sleep_mode_in_setting, sizeof(lcm_deep_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);
	//MDELAY(5);
	//SET_RESET_PIN(0);
    //MDELAY(10);
	display_bias_disable();
	MDELAY(5);
	display_ldo18_enable(0);
}

static void lcm_resume(void)
{
	printk("%s\n",__func__);

	lcm_init();
}

static unsigned int lcm_compare_id(void)
{
	unsigned int id = 0;
	unsigned char buffer[2];
	unsigned int array[16];

    mt_dsi_pinctrl_set(LCM_RESET_PIN_NO, 1);
	//SET_RESET_PIN(1);
	MDELAY(10);
	mt_dsi_pinctrl_set(LCM_RESET_PIN_NO, 0);
	//SET_RESET_PIN(0);
	MDELAY(10);
   mt_dsi_pinctrl_set(LCM_RESET_PIN_NO, 1);
	//SET_RESET_PIN(1);
	MDELAY(10);

	array[0] = 0x00023700;	/* read id return two byte,version and id */
	dsi_set_cmdq(array, 1, 1);

	read_reg_v2(0xA1, buffer, 2);
	id = (buffer[0] << 8) + buffer[1];     /* we only need ID */

	LCM_LOGI("%s,nl9911_id=0x%x\n", __func__, id);

	if (id == 0x9911)
		return 1;
	else
		return 0;

}

static unsigned int lcm_ata_check(unsigned char *buffer)
{
#ifndef BUILD_LK
#if 1
	unsigned int ret = 0;
	unsigned int x0 = FRAME_WIDTH / 4;
	unsigned int x1 = FRAME_WIDTH * 3 / 4;

	unsigned char x0_MSB = ((x0 >> 8) & 0xFF);
	unsigned char x0_LSB = (x0 & 0xFF);
	unsigned char x1_MSB = ((x1 >> 8) & 0xFF);
	unsigned char x1_LSB = (x1 & 0xFF);

	unsigned int data_array[3];
	unsigned char read_buf[4];
	LCM_LOGI("ATA check size = 0x%x,0x%x,0x%x,0x%x\n", x0_MSB, x0_LSB, x1_MSB, x1_LSB);
	
	data_array[0] = 0x0003390A; /* HS packet */
	data_array[1] = 0x00595AF0;
	dsi_set_cmdq(data_array, 2, 1);

	data_array[0] = 0x0003390A; /* HS packet */
	data_array[1] = 0x00A6A5F1;
	dsi_set_cmdq(data_array, 2, 1);
	
	data_array[0] = 0x0004390A; /* HS packet */
	data_array[1] = (x1_LSB << 24) | (x1_MSB << 16) | (x0_LSB << 8) | 0xB9;
	dsi_set_cmdq(data_array, 2, 1);

	data_array[0] = 0x00033700; /* read id return two byte,version and id */
	dsi_set_cmdq(data_array, 1, 1);
	read_reg_v2(0x04, read_buf, 3);
	printk("%s read[0x04]= %x  %x %x %x\n", __func__, read_buf[0], read_buf[1],read_buf[2],read_buf[3]);

//	MDELAY(10);
	data_array[0] = 0x00033700; /* read id return two byte,version and id */
	dsi_set_cmdq(data_array, 1, 1);
	read_reg_v2(0xB9, read_buf, 3);
	printk("%s read[0xB9]= %x %x %x %x \n", __func__, read_buf[0],read_buf[1],read_buf[2],read_buf[3]);

	if ((read_buf[0] == x0_LSB) && (read_buf[1] == x1_MSB)
	        && (read_buf[2] == x1_LSB))
		ret = 1;
	else
		ret = 0;

	return ret;
#endif
	return 1;
#else
	return 0;
#endif
}

struct LCM_DRIVER icnl9911c_hdp_dsi_vdo_auo6517_lcm_drv = {
	.name		= "icnl9911c_hdp_dsi_vdo_auo6517",
    	//prize-lixuefeng-20150512-start
	#if defined(CONFIG_PRIZE_HARDWARE_INFO) && !defined (BUILD_LK)
	.lcm_info = {
		.chip	= "icnl9911c",
		.vendor	= "LD",
		.id		= "0x11",
		.more	= "1600*720",
	},
	#endif
	//prize-lixuefeng-20150512-end	
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.compare_id 	= lcm_compare_id,
	.init_power	= lcm_init_power,
	.ata_check	= lcm_ata_check,
#ifndef BUILD_LK
	.resume_power	= lcm_resume_power,
	.suspend_power	= lcm_suspend_power,
    #endif
};

