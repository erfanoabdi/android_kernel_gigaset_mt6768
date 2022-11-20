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

#define LOG_TAG "LCM"

#ifndef BUILD_LK
#  include <linux/string.h>
#  include <linux/kernel.h>
#endif

#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
#include "data_hw_roundedpattern.h"
#endif

#include "lcm_drv.h"

#ifdef BUILD_LK
#  include <platform/upmu_common.h>
#  include <platform/mt_gpio.h>
#  include <platform/mt_i2c.h>
#  include <platform/mt_pmic.h>
#  include <string.h>
#elif defined(BUILD_UBOOT)
#  include <asm/arch/mt_gpio.h>
#endif

#ifdef BUILD_LK
#  define LCM_LOGI(string, args...)  dprintf(0, "[LK/"LOG_TAG"]"string, ##args)
#  define LCM_LOGD(string, args...)  dprintf(1, "[LK/"LOG_TAG"]"string, ##args)
#else
#  define LCM_LOGI(fmt, args...)  pr_notice("[KERNEL/"LOG_TAG"]"fmt, ##args)
#  define LCM_LOGD(fmt, args...)  pr_debug("[KERNEL/"LOG_TAG"]"fmt, ##args)
#endif

#define LCM_ID_NT36672C 0x10
static struct LCM_UTIL_FUNCS lcm_util;
static unsigned int lcm_compare_id(void);
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
#define read_reg(cmd)	lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size) \
		lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

#ifndef BUILD_LK
#  include <linux/kernel.h>
#  include <linux/module.h>
#  include <linux/fs.h>
#  include <linux/slab.h>
#  include <linux/init.h>
#  include <linux/list.h>
#  include <linux/i2c.h>
#  include <linux/irq.h>
#  include <linux/uaccess.h>
#  include <linux/interrupt.h>
#  include <linux/io.h>
#  include <linux/platform_device.h>
#endif

#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include "lcm_i2c.h"

#define FRAME_WIDTH				(1080)
#define FRAME_HEIGHT			(2460)

/* physical size in um */
#define LCM_PHYSICAL_WIDTH		(69200)
#define LCM_PHYSICAL_HEIGHT		(157600)
#define LCM_DENSITY						(480)

#define REGFLAG_DELAY					0xFFFC
#define REGFLAG_UDELAY				0xFFFB
#define REGFLAG_END_OF_TABLE	0xFFFD
#define REGFLAG_RESET_LOW			0xFFFE
#define REGFLAG_RESET_HIGH		0xFFFF

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

struct LCM_setting_table {
	unsigned int cmd;
	unsigned char count;
	unsigned char para_list[500];
};

static struct LCM_setting_table lcm_suspend_setting[] = {
	{0x28, 0, {} },
	{REGFLAG_DELAY, 20, {} },
	{0x10, 0, {} },
	{REGFLAG_DELAY, 120, {} },
};

static struct LCM_setting_table init_setting[] = {
	{0xff,1,{0x10}},
	{0xfb,1,{0x01}},
//	{0xb0,1,{0x00}},
	/* DSC on */
	{0xC0, 1, {0x03} },
	{0xC2, 2, {0x1B, 0xA0} },		

	{0xff,1,{0x20}},
	{0xfb,1,{0x01}},
	{0x01,1,{0x66}},
	{0x06,1,{0x5b}},
	{0x07,1,{0x27}},

	{0x1b,1,{0x01}},
	{0x5c,1,{0x90}},
	{0x5e,1,{0xe6}},
	{0x69,1,{0xd0}},
	{0x95,1,{0xe0}},
	{0x96,1,{0xe0}},
	{0xf2,1,{0x64}},
	{0xf4,1,{0x64}},
	{0xf6,1,{0x64}},
	{0xf8,1,{0x64}},
	
	{0xff,1,{0x21}},
	{0xfb,1,{0x01}},
	
	{0xff,1,{0x24}},
	{0xfb,1,{0x01}},
	{0x00,1,{0x00}},
	{0x07,1,{0x18}},
	{0x08,1,{0x16}},
	{0x09,1,{0x14}},
	{0x0a,1,{0x17}},
	{0x0b,1,{0x15}},
	{0x0c,1,{0x13}},
	{0x0d,1,{0x0f}},
	{0x0e,1,{0x2c}},
	{0x0f,1,{0x32}},
	{0x10,1,{0x30}},
	{0x11,1,{0x2e}},
	{0x12,1,{0x01}},
	{0x13,1,{0x10}},
	{0x14,1,{0x10}},
	{0x15,1,{0x10}},
	{0x16,1,{0x10}},
	{0x17,1,{0x10}},
	{0x18,1,{0x00}},
	{0x1f,1,{0x18}},
	{0x20,1,{0x16}},
	{0x21,1,{0x14}},
	{0x22,1,{0x17}},
	{0x23,1,{0x15}},
	{0x24,1,{0x13}},
	{0x25,1,{0x0f}},
	{0x26,1,{0x2d}},
	{0x27,1,{0x33}},
	{0x28,1,{0x31}},
	{0x29,1,{0x2f}},
	{0x2a,1,{0x01}},
	{0x2b,1,{0x10}},
	{0x2d,1,{0x10}},
	{0x2f,1,{0x10}},
	{0x30,1,{0x10}},
	{0x31,1,{0x10}},
	{0x32,1,{0x05}},
	{0x35,1,{0x01}},
	{0x36,1,{0x4c}},
	{0x36,1,{0x4c}},
	{0x4d,1,{0x03}},
	{0x4e,1,{0x46}},
	{0x4f,1,{0x46}},
	{0x53,1,{0x41}},
	{0x7a,1,{0x83}},
	{0x7b,1,{0x12}},
	{0x7c,1,{0x00}},
	{0x7d,1,{0x03}},
	{0x80,1,{0x03}},
	{0x81,1,{0x03}},
	{0x82,1,{0x13}},
	{0x84,1,{0x31}},
	{0x85,1,{0x00}},
	{0x86,1,{0x00}},
	{0x87,1,{0x00}},
	{0x90,1,{0x13}},
	{0x92,1,{0x31}},
	{0x93,1,{0x00}},
	{0x94,1,{0x00}},
	{0x95,1,{0x00}},
	{0x9c,1,{0xf4}},
	{0x9d,1,{0x01}},
	{0xa0,1,{0x12}},
	{0xa2,1,{0x12}},
	{0xa3,1,{0x03}},
	{0xa4,1,{0x03}},
	{0xa5,1,{0x03}},
	{0xc4,1,{0x80}},
	{0xc6,1,{0xc0}},
	{0xc9,1,{0x00}},
	{0xd9,1,{0x80}},
	{0xe9,1,{0x03}},
	
	{0xff,1,{0x25}},
	{0xfb,1,{0x01}},
	{0x0f,1,{0x1b}},
	{0x19,1,{0xe4}},
	{0x68,1,{0x58}},
	{0x69,1,{0x10}},
	{0x6b,1,{0x00}},
	{0x77,1,{0x72}},
	{0x78,1,{0x15}},
	{0x81,1,{0x00}},
	{0x84,1,{0x2d}},
	{0x8e,1,{0x10}},
	{0xc4,1,{0x11}},
	{0xc5,1,{0x11}},
	{0xc6,1,{0x11}},
	
	{0xff,1,{0x26}},
	{0xfb,1,{0x01}},
	{0x14,1,{0x06}},
	{0x15,1,{0x01}},
	{0x74,1,{0xaf}},
	{0x81,1,{0x12}},
	{0x83,1,{0x03}},
	{0x84,1,{0x02}},
	{0x85,1,{0x01}},
	{0x86,1,{0x02}},
	{0x87,1,{0x01}},
	{0x88,1,{0x0c}},
	{0x8a,1,{0x1a}},
	{0x8b,1,{0x11}},
	{0x8c,1,{0x24}},
	{0x8e,1,{0x42}},
	{0x8f,1,{0x11}},
	{0x90,1,{0x11}},
	{0x91,1,{0x11}},
	{0x9a,1,{0x80}},
	{0x9b,1,{0x04}},
	{0x9c,1,{0x00}},
	{0x9d,1,{0x00}},
	{0x9e,1,{0x00}},
	
	{0xff,1,{0x27}},
	{0xfb,1,{0x01}},
	{0x01,1,{0x9c}},
	{0x20,1,{0x81}},
	{0x21,1,{0xdf}},
	{0x25,1,{0x82}},
	{0x26,1,{0x13}},
	{0x6e,1,{0x12}},
	{0x6f,1,{0x00}},
	{0x70,1,{0x00}},
	{0x71,1,{0x00}},
	{0x72,1,{0x00}},
	{0x73,1,{0x76}},
	{0x74,1,{0x10}},
	{0x75,1,{0x32}},
	{0x76,1,{0x54}},
	{0x77,1,{0x00}},
	{0x7d,1,{0x09}},
	{0x7e,1,{0xa1}},
	{0x80,1,{0x27}},
	{0x82,1,{0x09}},
	{0x83,1,{0xa1}},
	{0x88,1,{0x01}},
	{0x89,1,{0x01}},
	
	{0xff,1,{0x2a}},
	{0xfb,1,{0x01}},
	{0x00,1,{0x91}},
	{0x03,1,{0x20}},
	{0x07,1,{0x50}},
	{0x0a,1,{0x60}},
	{0x0d,1,{0x40}},
	{0x0e,1,{0x02}},
	{0x11,1,{0xf6}},
	{0x15,1,{0x0f}},
	{0x16,1,{0x86}},
	{0x19,1,{0x0f}},
	{0x1a,1,{0x5a}},
	{0x1b,1,{0x14}},
	{0x1d,1,{0x36}},
	{0x1e,1,{0x4d}},
	{0x1f,1,{0x4d}},
	{0x20,1,{0x4d}},
	{0x28,1,{0xcb}},
	{0x29,1,{0x0a}},
	{0x2a,1,{0x1c}},
	{0x2d,1,{0x04}},
	{0x2f,1,{0x02}},
	{0x30,1,{0x0a}},
	{0x31,1,{0xbd}},
	{0x33,1,{0xe2}},
	{0x34,1,{0xcd}},
	{0x35,1,{0x2a}},
	{0x36,1,{0xda}},
	{0x36,1,{0xda}},
	{0x37,1,{0xc9}},
	{0x38,1,{0x2d}},
	{0x39,1,{0xd7}},
	{0x3a,1,{0x0a}},
	
	{0xff,1,{0x2c}},
	{0xfb,1,{0x01}},
	{0xff,1,{0xe0}},
	{0xfb,1,{0x01}},
	{0x35,1,{0x82}},
	
	{0xff,1,{0xf0}},
	{0xff,1,{0xf0}},
	{0xfb,1,{0x01}},
	{0x1c,1,{0x01}},
	{0x33,1,{0x01}},
	{0x5a,1,{0x00}},
	
	{0xff,1,{0xd0}},
	{0xfb,1,{0x01}},
	{0x53,1,{0x22}},
	{0x54,1,{0x02}},
	
	{0xff,1,{0xc0}},
	{0xfb,1,{0x01}},
	{0x9c,1,{0x11}},
	{0x9d,1,{0x11}},
	
	{0xff,1,{0x2b}},
	{0xfb,1,{0x01}},
	{0xb7,1,{0x12}},
	{0xb8,1,{0x02}},
	{0xc0,1,{0x01}},
/*
	{0xff,1,{0x20}},
	{0xfb,1,{0x01}},	
	{0x86,1,{0x03}},
*/	
	{0xff,1,{0x10}},
	{0xfb,1,{0x01}},
	{0x11, 0, {} },
	{REGFLAG_DELAY, 120, {} },
	// Display On
	{0x29, 0, {} },
	
};
static struct LCM_setting_table
__maybe_unused lcm_deep_sleep_mode_in_setting[] = {
	{0x28, 1, {0x00} },
	{REGFLAG_DELAY, 50, {} },
	{0x10, 1, {0x00} },
	{REGFLAG_DELAY, 150, {} },
};

static struct LCM_setting_table __maybe_unused lcm_sleep_out_setting[] = {
	{0x11, 1, {0x00} },
	{REGFLAG_DELAY, 120, {} },
	{0x29, 1, {0x00} },
	{REGFLAG_DELAY, 50, {} },
};

static void push_table(void *cmdq, struct LCM_setting_table *table,
		       unsigned int count, unsigned char force_update)
{
	unsigned int i;
	unsigned int cmd;

	for (i = 0; i < count; i++) {
		cmd = table[i].cmd;
		switch (cmd) {
		case REGFLAG_DELAY:
			if (table[i].count <= 10)
				MDELAY(table[i].count);
			else
				MDELAY(table[i].count);
			break;
		case REGFLAG_UDELAY:
			UDELAY(table[i].count);
			break;
		case REGFLAG_END_OF_TABLE:
			break;
		default:
			dsi_set_cmdq_V22(cmdq, cmd, table[i].count,
					 table[i].para_list, force_update);
			if (table[i].count > 1)
				MDELAY(1);
			break;
		}
	}
}

static void lcm_set_util_funcs(const struct LCM_UTIL_FUNCS *util)
{
	memcpy(&lcm_util, util, sizeof(struct LCM_UTIL_FUNCS));
}

#ifdef CONFIG_MTK_HIGH_FRAME_RATE
static void lcm_dfps_int(struct LCM_DSI_PARAMS *dsi)
{

	struct dfps_info *dfps_params = dsi->dfps_params;
	LCM_LOGI("%s:%d\n", __func__, __LINE__);
	dsi->dfps_enable = 1;
	dsi->dfps_default_fps = 9000;/*real fps * 100, to support float*/
	dsi->dfps_def_vact_tim_fps = 9000;/*real vact timing fps * 100*/

	/* DPFS_LEVEL0 */
	dfps_params[0].level = DFPS_LEVEL0;
	dfps_params[0].fps = 6000;/*real fps * 100, to support float*/
	dfps_params[0].vact_timing_fps = 9000;/*real vact timing fps * 100*/
	/* if mipi clock solution */
	/* if vfp solution */
	dfps_params[0].vertical_frontporch = 1200;
	//dfps_params[0].vertical_frontporch_for_low_power = 1200;

	/* DPFS_LEVEL1 */
	dfps_params[1].level = DFPS_LEVEL1;
	dfps_params[1].fps = 9000;/*real fps * 100, to support float*/
	dfps_params[1].vact_timing_fps = 9000;/*real vact timing fps * 100*/
	/* if mipi clock solution */
	dfps_params[1].vertical_frontporch = 54;
//	dfps_params[1].vertical_frontporch_for_low_power = 54;//1321;

	dsi->dfps_num = 2;
}
#endif

static void lcm_get_params(struct LCM_PARAMS *params)
{
	LCM_LOGI("%s:%d\n", __func__, __LINE__);
	memset(params, 0, sizeof(struct LCM_PARAMS));

	params->type = LCM_TYPE_DSI;

	params->width = FRAME_WIDTH;
	params->height = FRAME_HEIGHT;
	params->physical_width = LCM_PHYSICAL_WIDTH / 1000;
	params->physical_height = LCM_PHYSICAL_HEIGHT / 1000;
	params->physical_width_um = LCM_PHYSICAL_WIDTH;
	params->physical_height_um = LCM_PHYSICAL_HEIGHT;
	params->density = LCM_DENSITY;
	
	params->dsi.mode = SYNC_PULSE_VDO_MODE;//BURST_VDO_MODE;
	params->dsi.switch_mode = CMD_MODE;
	params->dsi.switch_mode_enable = 0;

	/* DSI */
	/* Command mode setting */
	params->dsi.LANE_NUM = LCM_FOUR_LANE;
	/* The following defined the fomat for data coming from LCD engine. */
	params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.trans_seq = LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding = LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format = LCM_DSI_FORMAT_RGB888;

	/* Highly depends on LCD driver capability. */
	params->dsi.packet_size = 256;
	/* video mode timing */
	params->dsi.PS = LCM_PACKED_PS_24BIT_RGB888;

	params->dsi.vertical_sync_active = 10;
	params->dsi.vertical_backporch = 10;
	params->dsi.vertical_frontporch = 1200;//54 90hz ;//1321 60hz 142 25 10; ////1200 60hz 165 22 22;
	params->dsi.vertical_frontporch_for_low_power = 1200;
	params->dsi.vertical_active_line = FRAME_HEIGHT;
	params->dsi.horizontal_sync_active = 22;//10;
	params->dsi.horizontal_backporch = 22;//25;
	params->dsi.horizontal_frontporch = 165;//142;
	params->dsi.horizontal_active_pixel = FRAME_WIDTH;  //FRAME_WIDTH
	/*with dsc*/
	params->dsi.bdg_dsc_enable = 1;
	params->dsi.bdg_ssc_disable = 1;
	params->dsi.dsc_params.ver = 17;
	params->dsi.dsc_params.slice_mode = 1;
	params->dsi.dsc_params.rgb_swap = 0;
	params->dsi.dsc_params.dsc_cfg = 34;
	params->dsi.dsc_params.rct_on = 1;
	params->dsi.dsc_params.bit_per_channel = 8; //P1
	params->dsi.dsc_params.dsc_line_buf_depth = 9; //P2
	params->dsi.dsc_params.bp_enable = 1; //P2 
	params->dsi.dsc_params.bit_per_pixel = 128; //P2
	params->dsi.dsc_params.pic_height = 2460;
	params->dsi.dsc_params.pic_width = 1080;
	params->dsi.dsc_params.slice_height = 20;//20;
	params->dsi.dsc_params.slice_width = 540;
	params->dsi.dsc_params.chunk_size = 540;
	params->dsi.dsc_params.xmit_delay = 170; //p6
	params->dsi.dsc_params.dec_delay = 526; //p7 p8
	params->dsi.dsc_params.scale_value = 32;
	params->dsi.dsc_params.increment_interval = 113;//113;//43; //P9-P10 
	params->dsi.dsc_params.decrement_interval = 7; //P11 P12
	params->dsi.dsc_params.line_bpg_offset = 12;
	params->dsi.dsc_params.nfl_bpg_offset = 1294;//1294;//3511; //P13-P14
	params->dsi.dsc_params.slice_bpg_offset = 1302;//1302;//3255; //P15-P16
	params->dsi.dsc_params.initial_offset = 6144;
	params->dsi.dsc_params.final_offset = 7072;//C2
	params->dsi.dsc_params.flatness_minqp = 3;
	params->dsi.dsc_params.flatness_maxqp = 12; 
	params->dsi.dsc_params.rc_model_size = 8192;
	params->dsi.dsc_params.rc_edge_factor = 6;
	params->dsi.dsc_params.rc_quant_incr_limit0 = 11;
	params->dsi.dsc_params.rc_quant_incr_limit1 = 11;
	params->dsi.dsc_params.rc_tgt_offset_hi = 3;
	params->dsi.dsc_params.rc_tgt_offset_lo = 3;
	params->dsi.dsc_enable = 0;
	
	params->dsi.PLL_CLOCK = 760/2;
	params->dsi.data_rate = 760;
	params->dsi.ssc_disable = 1;
	params->dsi.ssc_range = 1;
	params->dsi.cont_clock = 1;

	params->dsi.clk_lp_per_line_enable = 0;
	params->dsi.esd_check_enable = 1;
	params->dsi.customization_esd_check_enable = 1;
	params->dsi.lcm_esd_check_table[0].cmd = 0x0A;
	params->dsi.lcm_esd_check_table[0].count = 1;
	params->dsi.lcm_esd_check_table[0].para_list[0] = 0x9c;
	
	params->dsi.lane_swap_en = 0;
#ifdef MTK_RUNTIME_SWITCH_FPS_SUPPORT
	params->dsi.fps = 60;
#endif

#ifdef MTK_ROUND_CORNER_SUPPORT
	params->round_corner_params.round_corner_en = 1;
	params->round_corner_params.full_content = 0;
	params->round_corner_params.h = ROUND_CORNER_H_TOP;
	params->round_corner_params.h_bot = ROUND_CORNER_H_BOT;
	params->round_corner_params.tp_size = sizeof(top_rc_pattern);
	params->round_corner_params.lt_addr = (void *)top_rc_pattern;
#endif

#ifdef CONFIG_MTK_HIGH_FRAME_RATE
	/****DynFPS start****/
	lcm_dfps_int(&(params->dsi));
	/****DynFPS end****/
#endif
}
/* turn on gate ic & control voltage to 5.5V */
/* equle display_bais_enable ,mt6768 need +/-5.5V */
static void lcm_init_power(void)
{
	LCM_LOGI("%s:%d\n", __func__, __LINE__);
	display_ldo18_enable(1);
	display_bias_enable();
}

static void lcm_suspend_power(void)
{
	LCM_LOGI("%s:%d\n", __func__, __LINE__);
	//prize add by lvyuanchuan for deep sleep mode,20220922
	SET_RESET_PIN(1);
	display_bias_disable();
}

/* turn on gate ic & control voltage to 5.5V */
static void lcm_resume_power(void)
{
	LCM_LOGI("%s:%d\n", __func__, __LINE__);
	SET_RESET_PIN(0);
	lcm_init_power();
}

static void lcm_init(void)
{
	SET_RESET_PIN(0);
	MDELAY(15);
	SET_RESET_PIN(1);
	MDELAY(10);
	SET_RESET_PIN(0);
	MDELAY(10);
	SET_RESET_PIN(1);
	MDELAY(10);

	push_table(NULL, init_setting, ARRAY_SIZE(init_setting), 1);
}

static void lcm_suspend(void)
{
	LCM_LOGI("%s:%d\n", __func__, __LINE__);
	push_table(NULL, lcm_suspend_setting,
		   ARRAY_SIZE(lcm_suspend_setting), 1);
}

static void lcm_resume(void)
{
	LCM_LOGI("%s:%d\n", __func__, __LINE__);
	lcm_init();
}

static unsigned int lcm_ata_check(unsigned char *buffer)
{
#ifndef BUILD_LK
	unsigned int ret = 0;
	unsigned int id[3] = {0x83, 0x11, 0x2B};
	unsigned int data_array[3];
	unsigned char read_buf[3];

	data_array[0] = 0x00033700; /* set max return size = 3 */
	dsi_set_cmdq(data_array, 1, 1);

	read_reg_v2(0x04, read_buf, 3); /* read lcm id */

	LCM_LOGI("ATA read = 0x%x, 0x%x, 0x%x\n",
		 read_buf[0], read_buf[1], read_buf[2]);

	if ((read_buf[0] == id[0]) &&
	    (read_buf[1] == id[1]) &&
	    (read_buf[2] == id[2]))
		ret = 1;
	else
		ret = 0;

	return ret;
#else
	return 0;
#endif
}


static void lcm_update(unsigned int x, unsigned int y, unsigned int width,
	unsigned int height)
{
	unsigned int x0 = x;
	unsigned int y0 = y;
	unsigned int x1 = x0 + width - 1;
	unsigned int y1 = y0 + height - 1;

	unsigned char x0_MSB = ((x0 >> 8) & 0xFF);
	unsigned char x0_LSB = (x0 & 0xFF);
	unsigned char x1_MSB = ((x1 >> 8) & 0xFF);
	unsigned char x1_LSB = (x1 & 0xFF);
	unsigned char y0_MSB = ((y0 >> 8) & 0xFF);
	unsigned char y0_LSB = (y0 & 0xFF);
	unsigned char y1_MSB = ((y1 >> 8) & 0xFF);
	unsigned char y1_LSB = (y1 & 0xFF);

	unsigned int data_array[16];

#ifdef LCM_SET_DISPLAY_ON_DELAY
	lcm_set_display_on();
#endif

	data_array[0] = 0x00053902;
	data_array[1] = (x1_MSB << 24) | (x0_LSB << 16) | (x0_MSB << 8) | 0x2a;
	data_array[2] = (x1_LSB);
	dsi_set_cmdq(data_array, 3, 1);

	data_array[0] = 0x00053902;
	data_array[1] = (y1_MSB << 24) | (y0_LSB << 16) | (y0_MSB << 8) | 0x2b;
	data_array[2] = (y1_LSB);
	dsi_set_cmdq(data_array, 3, 1);

	data_array[0] = 0x002c3909;
	dsi_set_cmdq(data_array, 1, 0);
}

static unsigned int lcm_compare_id(void)
{
	unsigned int id = 0;
	unsigned char buffer[2];
	unsigned int array[16];

	MDELAY(20);

	array[0] = 0x00013700;  /* read id return 1byte */
	dsi_set_cmdq(array, 1, 1);

	read_reg_v2(0xDA, buffer, 1);
	id = buffer[0];     /* we only need ID */

	LCM_LOGI("%s,LCM_ID_NT36672C=0x%08x\n", __func__, id);

	if (id == LCM_ID_NT36672C)
		return 1;
	else
		return 0;

}


/* return TRUE: need recovery */
/* return FALSE: No need recovery */
static unsigned int lcm_esd_check(void)
{
#ifndef BUILD_LK
	char buffer[3];
	int array[4];

	array[0] = 0x00013700;
	dsi_set_cmdq(array, 1, 1);

	read_reg_v2(0x0A, buffer, 1);

	if (buffer[0] != 0x9C) {
		LCM_LOGI("[LCM ERROR] [0x0A]=0x%02x\n", buffer[0]);
		return TRUE;
	}
	LCM_LOGI("[LCM NORMAL] [0x0A]=0x%02x\n", buffer[0]);
	return FALSE;
#else
	return FALSE;
#endif

}
struct LCM_DRIVER nt36672c_fhdp_dsi_vdo_90hz_6382_lcm_drv = {
	.name = "nt36672c_fhdp_dsi_vdo_90hz_6382_lcm_drv",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params = lcm_get_params,
	.init = lcm_init,
	.suspend = lcm_suspend,
	.resume = lcm_resume,
	.compare_id = lcm_compare_id,
	.init_power = lcm_init_power,
	.resume_power = lcm_resume_power,
	.suspend_power = lcm_suspend_power,
	.esd_check = lcm_esd_check,
	.ata_check = lcm_ata_check,
	.update = lcm_update,
#if defined(CONFIG_PRIZE_HARDWARE_INFO) && !defined (BUILD_LK)
	.lcm_info = {
		.chip	= "nt36672c",
		.vendor	= "novatek",
		.id	= "0x10",
		.more	= "1080*2460",
	},
#endif
};
