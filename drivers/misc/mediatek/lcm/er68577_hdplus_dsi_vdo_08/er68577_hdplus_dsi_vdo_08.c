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
#define FRAME_HEIGHT 										(1440)

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

struct LCM_setting_table {
	unsigned cmd;
	unsigned char count;
	unsigned char para_list[64];
};


static struct LCM_setting_table lcm_initialization_setting[] = {
	
{0xE0,2,{0xAB,0xBA}},
{0xE1,2,{0xBA,0xAB}},
{0xB1,4,{0x10,0x01,0x47,0xFF}},
{0xB2,6,{0x0C,0x14,0x04,0x50,0x50,0x14}},
{0xB3,3,{0x56,0x52,0xA0}},
{0xB4,3,{0x33,0x30,0x04}},
{0xB6,7,{0xB0,0x00,0x00,0x10,0x00,0x10,0x00}},
{0xB8,5,{0x05,0x12,0x29,0x49,0x48}},  
{0xB9,38,{0x7C,0x37,0x29,0x1E,0x1D,0x10,0x19,0x06,0x23,0x25,0x27,0x47,0x38,0x42,0x35,0x34,0x26,0x15,0x06,0x7C,0x37,0x29,0x1E,0x1D,0x10,0x19,0x06,0x23,0x25,0x27,0x47,0x38,0x42,0x35,0x34,0x26,0x15,0x06}},
{0xC0,16,{0x54,0x56,0x12,0x34,0x44,0x44,0x33,0x33,0x90,0x04,0x70,0x04,0x3F,0x00,0x00,0xC1}},
{0xC1,10,{0x14,0x94,0x02,0x89,0x90,0x04,0x70,0x04,0x54,0x00}},        
{0xC2,12,{0x37,0x09,0x08,0x89,0x08,0x11,0x22,0x20,0x44,0xBB,0x18,0x00}},
{0xC3,22,{0x92,0x52,0x10,0x10,0x0E,0x0E,0x0C,0x0C,0x04,0x02,0x00,0x00,0x02,0x1C,0x06,0x1D,0x02,0x02,0x02,0x02,0x02,0x02}},
{0xC4,22,{0x13,0x13,0x11,0x11,0x0F,0x0F,0x0D,0x0D,0x05,0x02,0x00,0x00,0x02,0x1C,0x07,0x1D,0x02,0x02,0x02,0x02,0x02,0x02}},
{0xC6,2,{0x2A,0x2A}},                          
{0xC8,6,{0x11,0x00,0x31,0x42,0x34,0x16}},      
{0xC9,5,{0xA1,0x22,0xFF,0xC4,0x23}},           
{0xCA,2,{0xCB,0x43}},                          
{0xCD,8,{0x0E,0x4F,0x4F,0x30,0x14,0x6B,0x06,0xB3}},
{0xD2,4,{0xE3,0x2B,0x38,0x00}},                
{0xD4,11,{0x00,0x01,0x00,0x0E,0x04,0x44,0x08,0x10,0x00,0x00,0x00}},
{0xE6,8,{0x00,0x01,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF}},
{0xF0,5,{0x12,0x03,0x20,0x00,0xFF}},
{0xF3,1,{0x00}},
     

		 //sleep out 
		{0x11,1,{0x00}},       
		{REGFLAG_DELAY, 150, {}},
		// Display ON            
		{0x29, 1, {0x00}},       
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


//#ifndef BUILD_LK
__maybe_unused static struct LCM_setting_table lcm_deep_sleep_mode_in_setting[] = {
	
//	{0xFF,0x03,{0x98,0x81,0x00}},
	// Display off sequence
//	{0x28, 1, {0x00}},
//	{REGFLAG_DELAY, 50, {}},
    // Sleep Mode On
//	{0x10, 1, {0x00}},
	//{REGFLAG_DELAY, 120, {}},
	//{REGFLAG_END_OF_TABLE, 0x00, {}}
		// Display off sequence
	{0x28, 1, {0x00}},
	{REGFLAG_DELAY, 50, {}},

    // Sleep Mode On
	{0x10, 1, {0x00}},
	{REGFLAG_DELAY, 120, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};
//__maybe_unused static struct LCM_setting_table lcm_deep_sleep_mode_out_setting[] = {
	
//	{0xFF,0x03,{0x98,0x81,0x00}},
	// Display off sequence
//	{0x11, 1, {0x00}},
//	{REGFLAG_DELAY, 120, {}},

    // Sleep Mode On
//	{0x29, 1, {0x00}},
//	{REGFLAG_DELAY, 120, {}},
//	{REGFLAG_END_OF_TABLE, 0x00, {}}
//};
//#endif
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
		params->dsi.vertical_sync_active = 4;
		params->dsi.vertical_backporch = 12;
		params->dsi.vertical_frontporch = 20;
		params->dsi.vertical_active_line = FRAME_HEIGHT; 

		params->dsi.horizontal_sync_active				= 20;	
		params->dsi.horizontal_backporch				= 80;
		params->dsi.horizontal_frontporch				= 80;  //80
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
		params->dsi.PLL_CLOCK = 240;  //240=4db  230=1.5db
/***********************    esd  check   ***************************/
#ifndef BUILD_LK
		params->dsi.esd_check_enable = 0;
		params->dsi.customization_esd_check_enable = 0;
		params->dsi.lcm_esd_check_table[0].cmd          = 0x0a;
		params->dsi.lcm_esd_check_table[0].count        = 1;
		params->dsi.lcm_esd_check_table[0].para_list[0] = 0x1C;
#endif

}


static void lcm_init(void)
{
    SET_RESET_PIN(1);
    MDELAY(10);
    SET_RESET_PIN(0);
    MDELAY(20);
    SET_RESET_PIN(1);
    MDELAY(120);

	push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
}


static void lcm_suspend(void)
{
//#ifndef BUILD_LK
		//printk("[PRIZE KERNEL] %s, line =%d\n",__FUNCTION__,__LINE__);
//		push_table(lcm_deep_sleep_mode_in_setting, sizeof(lcm_deep_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);
//		SET_RESET_PIN(0);
//#else
//		printf("[PRIZE LK] %s, line =%d\n",__FUNCTION__,__LINE__);
//#endif
    push_table(lcm_deep_sleep_mode_in_setting, sizeof(lcm_deep_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);
	SET_RESET_PIN(1);
	MDELAY(1);
	SET_RESET_PIN(0);
	MDELAY(1);
	SET_RESET_PIN(1);
	MDELAY(120);
}


static void lcm_resume(void)
{
#ifndef BUILD_LK
		//printk("[PRIZE KERNEL] %s, line =%d\n",__FUNCTION__,__LINE__);
		lcm_init();
#else
		printf("[PRIZE LK] %s, line =%d\n",__FUNCTION__,__LINE__);
#endif
}

static unsigned int lcm_compare_id(void)
{
		return 0;
}


struct LCM_DRIVER er68577_hdplus_dsi_vdo_08_lcm_drv = {
	.name		= "er68577_hdplus_dsi_vdo_08",
	#if defined(CONFIG_PRIZE_HARDWARE_INFO)
	.lcm_info = {
		.chip	= "er68577",
		.vendor	= "mizan",
		.id		= "0xe8/800",
		.more	= "lcm_1440*720",
	},
	#endif
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.compare_id 	= lcm_compare_id,
};

