/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#ifndef BUILD_LK
#include <linux/string.h>
#include <linux/kernel.h>
#else
#include <platform/mt_gpio.h>
#include <platform/mt_pmic.h>
#endif
#include "lcm_drv.h"

/* --------------------------------------------------------------------------- */
/* Local Constants */
/* --------------------------------------------------------------------------- */

#define FRAME_WIDTH  										(720)
#define FRAME_HEIGHT 										(1520)

#define REGFLAG_DELAY             							0XFE
#define REGFLAG_END_OF_TABLE      							0xFFF   // END OF REGISTERS MARKER

#define LCM_DSI_CMD_MODE									0
/* --------------------------------------------------------------------------- */
/* Local Variables */
/* --------------------------------------------------------------------------- */

static struct LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)    								(lcm_util.set_reset_pin((v)))
#define MDELAY(n) 											(lcm_util.mdelay(n))


// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)										lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)					lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg											lcm_util.dsi_read_reg()
#define read_reg_v2(cmd, buffer, buffer_size)				lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

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
	unsigned cmd;
	unsigned char count;
	unsigned char para_list[64];
};


static struct LCM_setting_table lcm_initialization_setting[] = {
   {0XF0,2,{0X5A,0X59}},
    {0XF1,2,{0XA5,0XA6}},
    {0XB0,30,{0X87,0X86,0X85,0X84,0X02,0X03,0X04,0X05,0X33,0X33,0X33,0X33,0X00,0X00,0X00,0X78,0X00,0X00,0X0F,0X05,0X04,0X03,0X02,0X01,0X02,0X03,0X04,0X00,0X00,0X00}},
    //{0XB1,0X53,0X43,0X85,0X80,0X00,0X00,0X00,0X7F,0X00,0X00,0X04,0X08,0X54,0X00,0X00,0X00,0X44,0X40,0X02,0X01,0X40,0X02,0X01,0X40,0X02,0X01,0X40,0X02,0X01},//OE=1.95US
    {0XB1,29,{0X53,0X43,0X85,0X80,0X00,0X00,0X00,0X7E,0X00,0X00,0X04,0X08,0X54,0X00,0X00,0X00,0X44,0X40,0X02,0X01,0X40,0X02,0X01,0X40,0X02,0X01,0X40,0X02,0X01}},//OE=2.01US
    {0XB2,17,{0X54,0XC4,0X82,0X05,0X40,0X02,0X01,0X40,0X02,0X01,0X05,0X05,0X54,0X0C,0X0C,0X0D,0X0B}},
    {0XB3,31,{0X02,0X0C,0X06,0X0C,0X06,0X26,0X26,0X91,0XA2,0X33,0X44,0X00,0X26,0X00,0X18,0X01,0X02,0X08,0X20,0X30,0X08,0X09,0X44,0X20,0X40,0X20,0X40,0X08,0X09,0X22,0X33}},
    {0XB4,28,{0X00,0X00,0X00,0X00,0X04,0X06,0X01,0X01,0X10,0X12,0X0C,0X0E,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0XFF,0XFF,0XFC,0X60,0X30,0X00}},
    {0XB5,28,{0X00,0X00,0X00,0X00,0X05,0X07,0X01,0X01,0X11,0X13,0X0D,0X0F,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0XFF,0XFF,0XFC,0X60,0X30,0X00}},
    {0XB8,24,{0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00}},
    //{0XBA,0XA4,0XA4},
    {0XBB,13,{0X01,0X05,0X09,0X11,0X0D,0X19,0X1D,0X55,0X25,0X69,0X00,0X21,0X25}},
    {0XBC,14,{0X00,0X00,0X00,0X00,0X02,0X20,0XFF,0X00,0X03,0X33,0X01,0X73,0X33,0X00}},
    {0XBD,10,{0XE9,0X02,0X4F,0XCF,0X72,0XA4,0X08,0X44,0XAE,0X15}},
    {0XBE,10,{0X7D,0X7D,0X46,0X5A,0X0C,0X77,0X43,0X07,0X0E,0X0E}},//OP=5.5VGH=14VGL=-14
    {0XBF,8,{0X07,0X25,0X07,0X25,0X7F,0X00,0X11,0X04}},
    {0XC0,9,{0X10,0XFF,0XFF,0XFF,0XFF,0XFF,0X00,0XFF,0X00}},
    {0XC1,19,{0XC0,0X0C,0X20,0X96,0X04,0X30,0X30,0X04,0X2A,0XF0,0X35,0X00,0X07,0XCF,0XFF,0XFF,0XC0,0X00,0XC0}},
    {0XC2,1,{0X00}},
    {0XC3,11,{0X06,0X00,0XFF,0X00,0XFF,0X00,0X00,0X81,0X01,0X00,0X00}},
    {0XC4,10,{0X84,0X01,0X2B,0X41,0X00,0X3C,0X00,0X03,0X03,0X2E}},
    {0XC5,11,{0X03,0X1C,0XB8,0XB8,0X30,0X10,0X42,0X44,0X08,0X09,0X14}},
    {0XC6,10,{0X87,0X9B,0X2A,0X29,0X29,0X33,0X64,0X34,0X08,0X04}},
    {0XC7,22,{0XF7,0XDE,0XCA,0XB9,0X9A,0X7D,0X4F,0XA2,0X66,0X32,0XFF,0XC5,0X12,0XE1,0XBF,0X91,0X76,0X4F,0X1A,0X7F,0XE4,0X00}},
    {0XC8,22,{0XF7,0XDE,0XCA,0XB9,0X9A,0X7D,0X4F,0XA2,0X66,0X32,0XFF,0XC5,0X12,0XE1,0XBF,0X91,0X76,0X4F,0X1A,0X7F,0XE4,0X00}},
    {0XCB,1,{0X00}},
    {0XD0,5,{0X80,0X0D,0XFF,0X0F,0X61}},
    {0XD2,1,{0X42}},
    {0XFE,4,{0XFF,0XFF,0XFF,0X40}},
    //{0XE0,0X30,0X00,0X00,0X08,0X11,0X3F,0X22,0X62,0XDF,0XA0,0X04,0XCC,0X01,0XFF,0XF6,0XFF,0XF0,0XFD,0XFF,0XFD,0XF8,0XF5,0XFC,0XFC,0XFD,0XFF},
    //{0XE1,0XEF,0XFE,0XFE,0XFE,0XFE,0XEE,0XF0,0X20,0X33,0XFF,0X00,0X00,0X6A,0X90,0XC0,0X0D,0X6A,0XF0,0X3E,0XFF,0X00,0X05,0XFF//20KHZ},
    {0XF1,2,{0X5A,0X59}},
    {0XF0,2,{0XA5,0XA6}},
    {0X35,1,{0X00}},
    {0x11,0,{}},
    //{0x11,1,{0x00}},
    {REGFLAG_DELAY, 120, {}},
    {0x29,0,{}},
    //{0x29,1,{0x00}},
    {REGFLAG_DELAY, 10, {}},
    {0X26,1,{0X01}}//出睡眠增加的代码

};


static struct LCM_setting_table lcm_deep_sleep_mode_in_setting[] = {
	{0x26, 1, {0x08}},
        {0x28,0,{}},
        //{0x28,1,{0x00}},
	{REGFLAG_DELAY, 10, {}},
	{0x10,0, {}},
	//{0x10,1, {0x00}},
	{REGFLAG_DELAY, 120, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
	unsigned int i;

    for(i = 0; i < count; i++) {
		
        unsigned cmd;
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



/* --------------------------------------------------------------------------- */
/* LCM Driver Implementations */
/* --------------------------------------------------------------------------- */

static void lcm_set_util_funcs(const struct LCM_UTIL_FUNCS *util)
{
	memcpy(&lcm_util, util, sizeof(struct LCM_UTIL_FUNCS));
}

static void lcm_get_params(struct LCM_PARAMS *params)
{

		memset(params, 0, sizeof(struct LCM_PARAMS));
	
		params->type   = LCM_TYPE_DSI;

		params->width  = FRAME_WIDTH;
		params->height = FRAME_HEIGHT;
		params->density = 320;//LCM_DENSITY;


#if (LCM_DSI_CMD_MODE)
		params->dsi.mode   = CMD_MODE;
#else
		params->dsi.mode   = BURST_VDO_MODE;
#endif
		params->dsi.vertical_sync_active = 4;//10;
		params->dsi.vertical_backporch = 12;//20;
		params->dsi.vertical_frontporch = 150;//20;
		params->dsi.vertical_active_line = FRAME_HEIGHT; 

		params->dsi.horizontal_sync_active				= 4;//40;	
		params->dsi.horizontal_backporch				= 48;//50;
		params->dsi.horizontal_frontporch				= 48;//30;  //80
		params->dsi.horizontal_active_pixel				= FRAME_WIDTH;

		params->dsi.LANE_NUM = LCM_FOUR_LANE;
		
		params->dbi.te_mode 				= LCM_DBI_TE_MODE_VSYNC_ONLY;
		params->dbi.te_edge_polarity		= LCM_POLARITY_RISING;
		params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
		params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST;
		params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;
		params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;
		
		
		params->dsi.packet_size=256;
		params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;
		// Video mode setting		
		params->dsi.intermediat_buffer_num = 2;


		// params->dsi.HS_TRAIL=20; 
		params->dsi.PLL_CLOCK = 296;  //240=4db  230=1.5db
/***********************    esd  check   ***************************/
#ifndef BUILD_LK
		params->dsi.esd_check_enable = 1;
		params->dsi.customization_esd_check_enable = 1;
		params->dsi.lcm_esd_check_table[0].cmd          = 0x0a;
		params->dsi.lcm_esd_check_table[0].count        = 1;
		params->dsi.lcm_esd_check_table[0].para_list[0] = 0x9C;
#endif

}


static void lcm_init(void)
{


        SET_RESET_PIN(1);    
        MDELAY(10);                             
        SET_RESET_PIN(0);    
        MDELAY(10);                             
        SET_RESET_PIN(1);    
        MDELAY(35);                             
        SET_RESET_PIN(0);    
        MDELAY(10);                             
        SET_RESET_PIN(1);    
        MDELAY(10);                             
        SET_RESET_PIN(0);    
        MDELAY(10);                             
        SET_RESET_PIN(1);    
        MDELAY(10);                             
        SET_RESET_PIN(0);    
        MDELAY(10);                             
        SET_RESET_PIN(1);    
        MDELAY(30);

#ifdef BUILD_LK
        printf("%s:%d",__func__,__LINE__);
#else
        printk("%s:%d",__func__,__LINE__);
#endif
	push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
}


static void lcm_suspend(void)
{

	push_table(lcm_deep_sleep_mode_in_setting, sizeof(lcm_deep_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);
	MDELAY(50);
}


static void lcm_resume(void)
{
//	lcm_initialization_setting[141].para_list[0] = lcm_initialization_setting[141].para_list[0] + 1; 
	lcm_init();
	
//	push_table(lcm_sleep_out_setting, sizeof(lcm_sleep_out_setting) / sizeof(struct LCM_setting_table), 1);
}

static unsigned int lcm_compare_id(void)
{
		return 0;
}


struct LCM_DRIVER icnl9911c_boe_lcm_drv = {
	.name		= "icnl9911c_boe",
	#if defined(CONFIG_PRIZE_HARDWARE_INFO) && !defined (BUILD_LK)
	.lcm_info = {
		.chip	= "icnl9911c",
		.vendor	= "rouxian",
		.id		= "0x9911",
		.more	= "1520*720",
	},
	#endif
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.compare_id 	= lcm_compare_id,
};

