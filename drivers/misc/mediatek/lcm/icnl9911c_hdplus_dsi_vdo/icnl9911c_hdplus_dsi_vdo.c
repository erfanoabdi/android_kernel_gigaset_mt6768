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
#define FRAME_HEIGHT 										(1600)

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
{0xF0,2,{0x5A,0x59}},
{0xF1,2,{0xA5,0xA6}},
{0xB0,30,{0x87,0x86,0x05,0x04,0x8A,0x8B,0x04,0x05,0x55,0x55,0x55,0x55,0x1B,0x00,0x00,0x64,0x00,0x00,0x0F,0x05,0x04,0x03,0x02,0x01,0x02,0x03,0x04,0x00,0x00,0x00}},
{0xB1,29,{0x53,0x42,0x85,0x00,0x1B,0x00,0x00,0x64,0x00,0x00,0x04,0x08,0x54,0x00,0x00,0x00,0x44,0x40,0x02,0x01,0x40,0x02,0x01,0x40,0x02,0x01,0x40,0x02,0x01}},
{0xB2,17,{0x54,0x84,0x85,0x05,0x40,0x02,0x01,0x40,0x02,0x01,0x05,0x05,0x54,0x0C,0x0C,0x0D,0x0B}},
{0xB3,31,{0x02,0x0E,0x0B,0x0E,0x0B,0x26,0x26,0x91,0xA2,0x33,0x44,0x00,0x26,0x00,0x18,0x01,0x02,0x08,0x20,0x30,0x08,0x09,0x44,0x20,0x40,0x20,0x40,0x08,0x09,0x22,0x33}},
{0xB4,28,{0x02,0x02,0x00,0x00,0xE3,0x09,0x22,0x00,0x22,0x0D,0x0F,0x11,0x13,0x05,0x02,0x02,0x02,0x02,0x02,0x02,0x02,0x02,0xFF,0xFF,0xFC,0x00,0x00,0x00}},
{0xB5,28,{0x00,0x00,0x00,0x00,0xE3,0x08,0x22,0x00,0x22,0x0C,0x0E,0x10,0x12,0x04,0x02,0x02,0x02,0x02,0x02,0x02,0x02,0x02,0xFF,0xFF,0xFC,0x00,0x00,0x00}},
{0xB8,24,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
{0xBB,13,{0x01,0x05,0x09,0x11,0x0D,0x19,0x1D,0x55,0x25,0x69,0x00,0x21,0x25}},
{0xBC,14,{0x00,0x00,0x00,0x00,0x02,0x20,0xFF,0x00,0x03,0x24,0x01,0x73,0x33,0x02}},
{0xBD,10,{0xE9,0x02,0x4F,0xCF,0x72,0xA4,0x08,0x44,0xAE,0x15}},
{0xBE,10,{0x73,0x87,0x64,0x3C,0x0C,0x77,0x43,0x07,0x0E,0x0E}},
{0xBF,8,{0x07,0x25,0x07,0x25,0x7F,0x00,0x11,0x04}},
{0xC0,9,{0x10,0x00,0x1F,0xFF,0xFF,0xFF,0x00,0xFF,0x00}},
{0xC1,19,{0xC0,0x20,0x20,0x96,0x04,0x30,0x30,0x04,0x2A,0x40,0x36,0x00,0x07,0xCF,0xFF,0xFF,0xC0,0x00,0xC0}},
{0xC2,1,{0x00}},
{0xC3,9,{0x06,0x00,0xFF,0x00,0xFF,0x00,0x00,0x81,0x01}},
{0xC4,10,{0x84,0x01,0x2B,0x41,0x00,0x3C,0x00,0x03,0x03,0x2E}},
{0xC5,11,{0x03,0x1C,0xC0,0xC0,0x40,0x10,0x42,0x44,0x0F,0x0F,0x14}},
{0xC6,10,{0x87,0x96,0x2A,0x29,0x29,0x33,0x7F,0x34,0x08,0x04}},
{0xC7,22,{0xFC,0xC5,0xA2,0x88,0x58,0x38,0x06,0x57,0x1F,0xF1,0xC4,0x91,0xEA,0xBD,0xA2,0x77,0x60,0x3C,0x1A,0x7F,0xC0,0x00}},
{0xC8,22,{0xFC,0xC5,0xA2,0x88,0x58,0x38,0x06,0x57,0x1F,0xF1,0xC4,0x91,0xEA,0xBD,0xA2,0x77,0x60,0x3C,0x1A,0x7F,0xC0,0x00}},
{0xCB,1,{0x00}},
{0xD0,8,{0x80,0x0D,0xFF,0x0F,0x61,0x0B,0x08,0x04}},
{0xD2,1,{0x42}},
{0xFE,4,{0xFF,0xFF,0xFF,0x40}},
{0xE0,26,{0x30,0x00,0x80,0x88,0x11,0x3F,0x22,0x62,0xDF,0xA0,0x04,0xCC,0x01,0xFF,0xF6,0xFF,0xF0,0xFD,0xFF,0xFD,0xF8,0xF5,0xFC,0xFC,0xFD,0xFF}},
{0xE1,23,{0xEF,0xFE,0xFE,0xFE,0xFE,0xEE,0xF0,0x20,0x33,0xFF,0x00,0x00,0x6A,0x90,0xC0,0x0D,0x6A,0xF0,0x3E,0xFF,0x00,0x06,0x40}},
{0xF1,2,{0x5A,0x59}},
{0xF0,2,{0xA5,0xA6}},
{0x53,1,{0x2C}},
{0x35,1,{0x00}},
{0x11,0,{}},
{REGFLAG_DELAY, 120, {}},
{0x29,0,{}},
{REGFLAG_DELAY, 20, {}},
{0x26,1,{0x01}},
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
		params->dsi.vertical_backporch = 48;//20;
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
#ifdef BUILD_LK
        printf("%s:%d",__func__,__LINE__);
#else
        printk("%s:%d",__func__,__LINE__);
#endif
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


struct LCM_DRIVER icnl9911c_hdplus_dsi_vdo_lcm_drv = {
	.name		= "icnl9911c_hdplus_dsi_vdo",
	#if defined(CONFIG_PRIZE_HARDWARE_INFO) && !defined (BUILD_LK)
	.lcm_info = {
		.chip	= "icnl9911c",
		.vendor	= "rouxian",
		.id		= "0x9911",
		.more	= "1600*720",
	},
	#endif
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.compare_id 	= lcm_compare_id,
};

