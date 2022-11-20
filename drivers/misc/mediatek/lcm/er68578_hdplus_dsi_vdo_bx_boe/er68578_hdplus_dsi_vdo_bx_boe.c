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

#define REGFLAG_DELAY             							0XFEE
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
/*************************************************/
{0xFE, 1,{0x01}},

{0x0E, 1,{0x21}},
{0x24, 1,{0xC0}},
{0x25, 1,{0x53}},
{0x26, 1,{0x00}},
{0x2B, 1,{0xE5}},
{0x16, 1,{0x52}},
{0x27, 1,{0x0A}},
{0x29, 1,{0x0A}},
{0x2F, 1,{0x40}},
{0x34, 1,{0x6D}},
{0x1B, 1,{0x00}},
{0x12, 1,{0x08}},
{0x1A, 1,{0x06}},
{0x46, 1,{0x8A}},//vcom flick
{0x52, 1,{0x80}},
{0x53, 1,{0x00}},
{0x54, 1,{0x80}},
{0x55, 1,{0x00}},
{0x0F, 1,{0xF0}},
{0x5F, 1,{0x13}},//0x12 3lane 0x13 4lane

{0xFE, 1,{0x03}},
{0x00, 1,{0x05}},
{0x01, 1,{0x14}},
{0x02, 1,{0x06}},
{0x03, 1,{0xA8}},
{0x04, 1,{0x00}},
{0x05, 1,{0x00}},
{0x06, 1,{0x50}},
{0x07, 1,{0x05}},
{0x08, 1,{0x14}},
{0x09, 1,{0x06}},
{0x0A, 1,{0xA8}},
{0x0B, 1,{0x00}},
{0x0C, 1,{0x00}},
{0x0D, 1,{0x50}},
{0x0E, 1,{0x0C}},
{0x0F, 1,{0x0D}},
{0x10, 1,{0x0E}},
{0x11, 1,{0x0F}},
{0x12, 1,{0x00}},
{0x13, 1,{0x26}},
{0x14, 1,{0xD0}},
{0x15, 1,{0xC4}},
{0x16, 1,{0x08}},
{0x17, 1,{0x08}},
{0x18, 1,{0x09}},
{0x19, 1,{0x0A}},
{0x1A, 1,{0x0B}},
{0x1B, 1,{0x00}},
{0x1C, 1,{0x26}},
{0x1D, 1,{0xD0}},
{0x1E, 1,{0x84}},
{0x1F, 1,{0x08}},
{0x20, 1,{0x00}},
{0x21, 1,{0x00}},
{0x22, 1,{0x08}},
{0x23, 1,{0xB2}},
{0x24, 1,{0xB2}},
{0x25, 1,{0x2D}},
{0x26, 1,{0x00}},
{0x27, 1,{0xAE}},
{0x28, 1,{0xAE}},
{0x29, 1,{0x2D}},
{0x2A, 1,{0x00}},
{0x2B, 1,{0x00}},
{0x2D, 1,{0x00}},
{0x2F, 1,{0x00}},
{0x30, 1,{0x00}},
{0x31, 1,{0x00}},
{0x32, 1,{0x00}},
{0x33, 1,{0x00}},
{0x34, 1,{0x00}},
{0x35, 1,{0x00}},
{0x36, 1,{0x00}},
{0x37, 1,{0x00}},
{0x38, 1,{0x00}},
{0x39, 1,{0x00}},
{0x3A, 1,{0x00}},
{0x3B, 1,{0x00}},
{0x3D, 1,{0x00}},
{0x3F, 1,{0x00}},
{0x40, 1,{0x00}},
{0x41, 1,{0x00}},
{0x42, 1,{0x00}},
{0x43, 1,{0x00}},
{0x44, 1,{0x00}},
{0x45, 1,{0x00}},
{0x46, 1,{0x00}},
{0x47, 1,{0x00}},
{0x48, 1,{0x00}},
{0x49, 1,{0x00}},
{0x4A, 1,{0x00}},
{0x4B, 1,{0x00}},
{0x4C, 1,{0x00}},
{0x4D, 1,{0x00}},
{0x4E, 1,{0x00}},
{0x4F, 1,{0x00}},
{0x50, 1,{0x00}},
{0x51, 1,{0x00}},
{0x52, 1,{0x00}},
{0x53, 1,{0x00}},
{0x54, 1,{0x00}},
{0x55, 1,{0x00}},
{0x56, 1,{0x00}},
{0x57, 1,{0x00}},
{0x58, 1,{0x00}},
{0x59, 1,{0x00}},
{0x5A, 1,{0x00}},
{0x5B, 1,{0x00}},
{0x5C, 1,{0x00}},
{0x5D, 1,{0x00}},
{0x5E, 1,{0x00}},
{0x5F, 1,{0x00}},
{0x60, 1,{0x00}},
{0x61, 1,{0x00}},
{0x62, 1,{0x00}},
{0x63, 1,{0x00}},
{0x64, 1,{0x00}},
{0x65, 1,{0x00}},
{0x66, 1,{0x00}},
{0x67, 1,{0x00}},
{0x68, 1,{0x00}},
{0x69, 1,{0x00}},
{0x6A, 1,{0x00}},
{0x6B, 1,{0x00}},
{0x6C, 1,{0x00}},
{0x6D, 1,{0x00}},
{0x6E, 1,{0x00}},
{0x6F, 1,{0x00}},
{0x70, 1,{0x00}},
{0x71, 1,{0x00}},
{0x72, 1,{0x00}},
{0x73, 1,{0x00}},
{0x74, 1,{0x00}},
{0x75, 1,{0x00}},
{0x76, 1,{0x00}},
{0x77, 1,{0x00}},
{0x78, 1,{0x00}},
{0x79, 1,{0x00}},
{0x7A, 1,{0x00}},
{0x7B, 1,{0x00}},
{0x7C, 1,{0x00}},
{0x7D, 1,{0x00}},
{0x7E, 1,{0x1C}},
{0x7F, 1,{0x1C}},
{0x80, 1,{0x03}},
{0x81, 1,{0x01}},
{0x82, 1,{0x09}},
{0x83, 1,{0x0B}},
{0x84, 1,{0x0D}},
{0x85, 1,{0x0F}},
{0x86, 1,{0x3F}},
{0x87, 1,{0x3F}},
{0x88, 1,{0x3F}},
{0x89, 1,{0x3F}},
{0x8A, 1,{0x3F}},
{0x8B, 1,{0x3F}},
{0x8C, 1,{0x3F}},
{0x8D, 1,{0x3F}},
{0x8E, 1,{0x3F}},
{0x8F, 1,{0x3F}},
{0x90, 1,{0x3F}},
{0x91, 1,{0x3F}},
{0x92, 1,{0x1D}},
{0x93, 1,{0x1D}},
{0x94, 1,{0x1D}},
{0x95, 1,{0x1D}},
{0x96, 1,{0x3F}},
{0x97, 1,{0x3F}},
{0x98, 1,{0x3F}},
{0x99, 1,{0x3F}},
{0x9A, 1,{0x3F}},
{0x9B, 1,{0x3F}},
{0x9C, 1,{0x3F}},
{0x9D, 1,{0x3F}},
{0x9E, 1,{0x3F}},
{0x9F, 1,{0x3F}},
{0xA0, 1,{0x3F}},
{0xA2, 1,{0x3F}},
{0xA3, 1,{0x0E}},
{0xA4, 1,{0x0C}},
{0xA5, 1,{0x0A}},
{0xA6, 1,{0x08}},
{0xA7, 1,{0x00}},
{0xA9, 1,{0x02}},
{0xAA, 1,{0x1C}},
{0xAB, 1,{0x1C}},
{0xAC, 1,{0x1C}},
{0xAD, 1,{0x1D}},
{0xAE, 1,{0x00}},
{0xAF, 1,{0x02}},
{0xB0, 1,{0x0E}},
{0xB1, 1,{0x0C}},
{0xB2, 1,{0x0A}},
{0xB3, 1,{0x08}},
{0xB4, 1,{0x3F}},
{0xB5, 1,{0x3F}},
{0xB6, 1,{0x3F}},
{0xB7, 1,{0x3F}},
{0xB8, 1,{0x3F}},
{0xB9, 1,{0x3F}},
{0xBA, 1,{0x3F}},
{0xBB, 1,{0x3F}},
{0xBC, 1,{0x3F}},
{0xBD, 1,{0x3F}},
{0xBE, 1,{0x3F}},
{0xBF, 1,{0x3F}},
{0xC0, 1,{0x1D}},
{0xC1, 1,{0x1C}},
{0xC2, 1,{0x1C}},
{0xC3, 1,{0x1D}},
{0xC4, 1,{0x3F}},
{0xC5, 1,{0x3F}},
{0xC6, 1,{0x3F}},
{0xC7, 1,{0x3F}},
{0xC8, 1,{0x3F}},
{0xC9, 1,{0x3F}},
{0xCA, 1,{0x3F}},
{0xCB, 1,{0x3F}},
{0xCC, 1,{0x3F}},
{0xCD, 1,{0x3F}},
{0xCE, 1,{0x3F}},
{0xCF, 1,{0x3F}},
{0xD0, 1,{0x09}},
{0xD1, 1,{0x0B}},
{0xD2, 1,{0x0D}},
{0xD3, 1,{0x0F}},
{0xD4, 1,{0x03}},
{0xD5, 1,{0x01}},
{0xD6, 1,{0x1D}},
{0xD7, 1,{0x1C}},
{0xDC, 1,{0x02}},
{0xDE, 1,{0x0F}},
{0xDF, 1,{0x00}},
{0xFE, 1,{0x0E}},
{0x01, 1,{0x75}},
{0x54, 1,{0x01}},

{0xFE, 1,{0x04}},
{0x60, 1,{0x01}},
{0x61, 1,{0x05}},
{0x62, 1,{0x09}},
{0x63, 1,{0x0C}},
{0x64, 1,{0x04}},
{0x65, 1,{0x0E}},
{0x66, 1,{0x0D}},
{0x67, 1,{0x09}},
{0x68, 1,{0x18}},
{0x69, 1,{0x0D}},
{0x6A, 1,{0x11}},
{0x6B, 1,{0x05}},
{0x6C, 1,{0x0D}},
{0x6D, 1,{0x0F}},
{0x6E, 1,{0x09}},
{0x6F, 1,{0x00}},
{0x70, 1,{0x01}},
{0x71, 1,{0x05}},
{0x72, 1,{0x09}},
{0x73, 1,{0x0C}},
{0x74, 1,{0x04}},
{0x75, 1,{0x0E}},
{0x76, 1,{0x0D}},
{0x77, 1,{0x09}},
{0x78, 1,{0x18}},
{0x79, 1,{0x0D}},
{0x7A, 1,{0x11}},
{0x7B, 1,{0x05}},
{0x7C, 1,{0x0D}},
{0x7D, 1,{0x0F}},
{0x7E, 1,{0x09}},
{0x7F, 1,{0x00}},

{0xFE, 1,{0x00}},
{0x58, 1,{0x00}},
{0x35, 0,{0x00}},

{0x11, 0,{0x00}},
{REGFLAG_DELAY, 120, {}},
{0x29, 0,{0x00}},
{REGFLAG_DELAY, 20, {}},     
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
		params->dsi.vertical_backporch = 36;
		params->dsi.vertical_frontporch = 16;
		params->dsi.vertical_active_line = FRAME_HEIGHT; 

		params->dsi.horizontal_sync_active				= 2 ;	
		params->dsi.horizontal_backporch				= 36;
		params->dsi.horizontal_frontporch				= 26;  //80
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
		params->dsi.PLL_CLOCK = 221;  //240=4db  230=1.5db
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

static unsigned int lcm_compare_id(void)
{
	unsigned int id[3];
	unsigned char buffer[4];
	unsigned int array[16];

	SET_RESET_PIN(1);
	MDELAY(15);
	SET_RESET_PIN(0);
	MDELAY(15);

	SET_RESET_PIN(1);
	MDELAY(20);

	array[0] = 0x01fe1500;	/* read id return two byte,version and id */
	dsi_set_cmdq(array, 1, 1);

	read_reg_v2(0xDE, buffer, 3);
	id[0] = buffer[0];		/* we only need ID */
	id[1] = buffer[1];		/* we only need ID */
	id[2] = buffer[2];		/* we only need ID */
	printk("%s,wangmd debug: nt35695 id = 0x%08x 0x%08x 0x%08x\n", __func__, id[0],id[1],id[2]);
	array[0] = 0x00fe1500;	/* read id return two byte,version and id */
	dsi_set_cmdq(array, 1, 1);
		return 0;
}
static void lcm_resume(void)
{
	
//		unsigned int array[16];
//lcm_compare_id();
#ifndef BUILD_LK
		//printk("[PRIZE KERNEL] %s, line =%d\n",__FUNCTION__,__LINE__);
		lcm_init();
#else
		printf("[PRIZE LK] %s, line =%d\n",__FUNCTION__,__LINE__);
#endif
}




struct LCM_DRIVER er68578_hdplus_dsi_vdo_bx_boe_lcm_drv = {
	.name		= "er68578_hdplus_dsi_vdo_bx_boe",
	#if defined(CONFIG_PRIZE_HARDWARE_INFO)
	.lcm_info = {
		.chip	= "er68578",
		.vendor	= "bx",
		.id		= "0x68",
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

