#ifndef BUILD_LK
#include <linux/string.h>
#include <linux/kernel.h>
#else
#include <platform/mt_gpio.h>
#include <platform/mt_pmic.h>
#endif
#include "lcm_drv.h"

// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

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


{0xFF,0x01,{0x30}},
{0xFF,0x01,{0x52}},
{0xFF,0x01,{0x01}},
{0xE3,0x01,{0x00}},
{0x03,0x01,{0x40}},
{0x04,0x01,{0x00}},
{0x05,0x01,{0x03}},
{0x25,0x01,{0x0A}},
{0x28,0x01,{0x50}},//47
{0x29,0x01,{0xc5}},
{0x2a,0x01,{0x9f}},
{0x37,0x01,{0x74}}, 
{0x38,0x01,{0x7f}}, 
{0x39,0x01,{0x2C}},
{0x40,0x01,{0x12}},
{0x44,0x01,{0x00}}, 
{0x49,0x01,{0x2C}},
{0x59,0x01,{0xfe}}, 
{0x5C,0x01,{0x00}},
{0x6D,0x01,{0x00}},
{0x6E,0x01,{0x00}},
{0x80,0x01,{0x20}}, 
{0x91,0x01,{0x67}},
{0x92,0x01,{0x67}},
{0xA0,0x01,{0x55}},
{0xA1,0x01,{0x50}},
{0xA3,0x01,{0x58}},
{0xA4,0x01,{0x9C}},
{0xA7,0x01,{0x02}},
{0xA8,0x01,{0x01}},
{0xA9,0x01,{0x21}},  //01
{0xAA,0x01,{0xFC}},
{0xAB,0x01,{0x28}},
{0xAC,0x01,{0x06}},
{0xAD,0x01,{0x06}},
{0xAE,0x01,{0x06}},
{0xAF,0x01,{0x03}},
{0xB0,0x01,{0x08}},
{0xB1,0x01,{0x26}},
{0xB2,0x01,{0x28}},
{0xB3,0x01,{0x28}},
{0xB4,0x01,{0x03}},
{0xB5,0x01,{0x08}},
{0xB6,0x01,{0x26}},
{0xB7,0x01,{0x08}},
{0xB8,0x01,{0x26}},
{0xF0,0x01,{0x00}},
{0xF6,0x01,{0xC0}},
{0xFF,0x01,{0x30}},
{0xFF,0x01,{0x52}},
{0xFF,0x01,{0x02}},
{0x28,0x01,{0x0A}},
{0x29,0x01,{0x21}},//01
{0x2A,0x01,{0x1f}},
{0xB1,0x01,{0x14}},
{0xD1,0x01,{0x0A}},
{0xB4,0x01,{0x33}},
{0xD4,0x01,{0x2D}},
{0xB2,0x01,{0x15}},
{0xD2,0x01,{0x0B}},
{0xB3,0x01,{0x35}},
{0xD3,0x01,{0x2F}},
{0xB6,0x01,{0x26}},
{0xD6,0x01,{0x20}},
{0xB7,0x01,{0x42}},
{0xD7,0x01,{0x3C}},
{0xC1,0x01,{0x09}},
{0xE1,0x01,{0x09}},
{0xB8,0x01,{0x0D}},
{0xD8,0x01,{0x0D}},
{0xB9,0x01,{0x03}},
{0xD9,0x01,{0x05}},
{0xBD,0x01,{0x12}},
{0xDD,0x01,{0x14}},
{0xBC,0x01,{0x10}},
{0xDC,0x01,{0x12}},
{0xBB,0x01,{0x0F}},
{0xDB,0x01,{0x11}},
{0xBA,0x01,{0x11}},
{0xDA,0x01,{0x11}},
{0xBE,0x01,{0x1A}},
{0xDE,0x01,{0x1A}},
{0xBF,0x01,{0x11}},
{0xDF,0x01,{0x11}},
{0xC0,0x01,{0x18}},
{0xE0,0x01,{0x18}},
{0xB5,0x01,{0x37}},
{0xD5,0x01,{0x32}},
{0xB0,0x01,{0x02}},
{0xD0,0x01,{0x05}},
{0xFF,0x01,{0x30}},
{0xFF,0x01,{0x52}},
{0xFF,0x01,{0x03}},
{0x04,0x01,{0x51}},
{0x05,0x01,{0xc0}},
{0x06,0x01,{0xc0}},
{0x08,0x01,{0x87}},
{0x09,0x01,{0x86}},
{0x0a,0x01,{0x85}},
{0x0b,0x01,{0x84}},
{0x24,0x01,{0x51}},
{0x25,0x01,{0xc0}},
{0x26,0x01,{0xc0}},
{0x2a,0x01,{0x9f}},
{0x2b,0x01,{0xa0}},
{0x2c,0x01,{0xa1}},
{0x2d,0x01,{0xa2}},
{0x34,0x01,{0xa1}},
{0x35,0x01,{0x53}},
{0x36,0x01,{0x53}},
{0x37,0x01,{0x13}},
{0x40,0x01,{0x85}},
{0x41,0x01,{0x84}},
{0x42,0x01,{0x83}},
{0x43,0x01,{0x82}},
{0x45,0x01,{0xa0}},
{0x46,0x01,{0xa1}},
{0x48,0x01,{0xa2}},
{0x49,0x01,{0xa3}},
{0x50,0x01,{0x81}}, 
{0x51,0x01,{0x80}},
{0x52,0x01,{0x01}},
{0x53,0x01,{0x02}},
{0x55,0x01,{0xa4}},
{0x56,0x01,{0xa5}},
{0x58,0x01,{0xa6}},
{0x59,0x01,{0xa7}},
{0x80,0x01,{0x03}}, 
{0x81,0x01,{0x02}}, 
{0x82,0x01,{0x07}}, 
{0x83,0x01,{0x06}}, 
{0x84,0x01,{0x05}}, 
{0x85,0x01,{0x04}}, 
{0x92,0x01,{0x00}}, 
{0x93,0x01,{0x01}}, 
{0x96,0x01,{0x03}}, 
{0x97,0x01,{0x02}}, 
{0x98,0x01,{0x07}}, 
{0x99,0x01,{0x06}}, 
{0x9a,0x01,{0x05}}, 
{0x9b,0x01,{0x04}}, 
{0xa8,0x01,{0x00}}, 
{0xa9,0x01,{0x01}}, 
{0xFF,0x01,{0x30}},
{0xFF,0x01,{0x52}},
{0xFF,0x01,{0x00}},
{0x36,0x01,{0x02}},

		{0x11,1,{0x00}},       
		{REGFLAG_DELAY, 200, {}},
		// Display ON            
		{0x29, 1, {0x00}}, 
		{REGFLAG_DELAY, 10, {}},
	//{0x35,0x01,{0x00}}, 	
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


// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------

static void lcm_set_util_funcs(const struct LCM_UTIL_FUNCS *util)
{
	memcpy(&lcm_util, util, sizeof(struct LCM_UTIL_FUNCS));
}


#ifndef BUILD_LK
__maybe_unused static struct LCM_setting_table lcm_deep_sleep_mode_in_setting[] = {
	
	// Display off sequence
	{0x28, 1, {0x00}},
	{REGFLAG_DELAY, 50, {}},
    // Sleep Mode On
	{0x10, 1, {0x00}},
	{REGFLAG_DELAY, 120, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};
__maybe_unused static struct LCM_setting_table lcm_deep_sleep_mode_out_setting[] = {
	
	// Display off sequence
	{0x11, 1, {0x00}},
	{REGFLAG_DELAY, 120, {}},

    // Sleep Mode On
	{0x29, 1, {0x00}},
	{REGFLAG_DELAY, 120, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};
#endif

static void lcm_get_params(struct LCM_PARAMS *params)
{
		memset(params, 0, sizeof(struct LCM_PARAMS));
	
		params->type   = LCM_TYPE_DSI;

		params->width  = FRAME_WIDTH;
		params->height = FRAME_HEIGHT;
			//5.5 18:9 62476 124951,16:9 68489 121759
	params->physical_width = 62;//LCM_PHYSICAL_WIDTH/1000;
	params->physical_height = 125;//LCM_PHYSICAL_HEIGHT/1000;
	params->physical_width_um = 62476;//LCM_PHYSICAL_WIDTH;	= sqrt((size*25.4)^2/(18^2+9^2))*9*1000
	params->physical_height_um = 124951;//LCM_PHYSICAL_HEIGHT; = sqrt((size*25.4)^2/(18^2+9^2))*18*1000
	params->density = 320;//LCM_DENSITY;

#if (LCM_DSI_CMD_MODE)
		params->dsi.mode   = CMD_MODE;
#else
		params->dsi.mode   = BURST_VDO_MODE;
#endif
		params->dsi.vertical_sync_active = 2;
		params->dsi.vertical_backporch = 8;
		params->dsi.vertical_frontporch = 14;
		params->dsi.vertical_active_line = FRAME_HEIGHT; 

		params->dsi.horizontal_sync_active				= 2;	
		params->dsi.horizontal_backporch				= 30;//50
		params->dsi.horizontal_frontporch				= 24;  //30 
		params->dsi.horizontal_active_pixel				= FRAME_WIDTH;
		params->dsi.LANE_NUM = LCM_FOUR_LANE;
		params->dbi.te_mode 				= LCM_DBI_TE_MODE_VSYNC_ONLY;
		params->dbi.te_edge_polarity		= LCM_POLARITY_RISING;
		params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
		params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST;
		params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;
		params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;
		params->dsi.ssc_disable				=1;
		
		params->dsi.packet_size=256;
		params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;
		// Video mode setting		
		params->dsi.intermediat_buffer_num = 2;


		// params->dsi.HS_TRAIL=20; 
		params->dsi.PLL_CLOCK = 212; //320 20200424 //240=4db  230=1.5db
/***********************    esd  check   ***************************/
//#ifndef BUILD_LK
	//	params->dsi.esd_check_enable = 1;
	//	params->dsi.customization_esd_check_enable = 1;
	//	params->dsi.lcm_esd_check_table[0].cmd          = 0x0a;
	//	params->dsi.lcm_esd_check_table[0].count        = 1;
	//	params->dsi.lcm_esd_check_table[0].para_list[0] = 0x9C;
//#endif

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
#ifndef BUILD_LK
		//printk("[PRIZE KERNEL] %s, line =%d\n",__FUNCTION__,__LINE__);
		push_table(lcm_deep_sleep_mode_in_setting, sizeof(lcm_deep_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);
		SET_RESET_PIN(0);
#else
		printf("[PRIZE LK] %s, line =%d\n",__FUNCTION__,__LINE__);
#endif
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
		return 1;
}


struct LCM_DRIVER nv3051d_hdplus_dsi_vdo_hq_lcm_drv = {
	.name		= "nv3051d_hdplus_dsi_vdo_hq",
    	//prize-lixuefeng-20150512-start
	#if defined(CONFIG_PRIZE_HARDWARE_INFO) && !defined (BUILD_LK)
	.lcm_info = {
		.chip	= "nv3051d_huaqi",
		.vendor	= "dezhixin_12v",
		.id		= "0x9366",
		.more	= "lcm_1440*720",
	},
	#endif
	//prize-lixuefeng-20150512-end	
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.compare_id 	= lcm_compare_id,
	
#if (LCM_DSI_CMD_MODE)
	.set_backlight	= lcm_setbacklight,
    .update         = lcm_update,
#endif
};

