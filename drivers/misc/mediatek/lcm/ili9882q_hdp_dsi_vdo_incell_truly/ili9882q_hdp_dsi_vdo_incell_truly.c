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
#else
/*#include <mach/mt_pm_ldo.h>*/
#ifdef CONFIG_MTK_LEGACY
#include <mach/mt_gpio.h>
#endif
#endif

#ifdef CONFIG_MTK_LEGACY
#include <cust_gpio_usage.h>
#endif
#ifndef CONFIG_FPGA_EARLY_PORTING
#if defined(CONFIG_MTK_LEGACY)
#include <cust_i2c.h>
#endif
#endif

#ifdef BUILD_LK
#define LCM_LOGI(string, args...)  dprintf(0, "[LK/"LOG_TAG"]"string, ##args)
#define LCM_LOGD(string, args...)  dprintf(1, "[LK/"LOG_TAG"]"string, ##args)
#else
#define LCM_LOGI(fmt, args...)  pr_debug("[KERNEL/"LOG_TAG"]"fmt, ##args)
#define LCM_LOGD(fmt, args...)  pr_debug("[KERNEL/"LOG_TAG"]"fmt, ##args)
#endif

static  struct  LCM_UTIL_FUNCS lcm_util;

#define SET_RESET_PIN(v)            (lcm_util.set_reset_pin((v)))
#define MDELAY(n)                   (lcm_util.mdelay(n))
#define UDELAY(n)		(lcm_util.udelay(n))

/* --------------------------------------------------------------------------- */
/* Local Functions */
/* --------------------------------------------------------------------------- */
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)     lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)        lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)                                       lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)                   lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)                                        lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)                lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

/* --------------------------------------------------------------------------- */
/* Local Constants */
/* --------------------------------------------------------------------------- */
#define LCM_DSI_CMD_MODE	0
#define FRAME_WIDTH  										 (720)
#define FRAME_HEIGHT 										 (1600)

#define REGFLAG_DELAY             							 0xFFA
#define REGFLAG_UDELAY             							 0xFFB
#define REGFLAG_PORT_SWAP									 0xFFC
#define REGFLAG_END_OF_TABLE      							 0xFFD   // END OF REGISTERS MARKER

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

/* --------------------------------------------------------------------------- */
/* Local Variables */
/* --------------------------------------------------------------------------- */

struct LCM_setting_table
{
    unsigned int cmd;
    unsigned char count;
    unsigned char para_list[64];
};

static struct LCM_setting_table lcm_suspend_setting[] =
{
	{0x28,1, {0x00}},
	{REGFLAG_DELAY,40,{}},   
	{0x10,1, {0x00}},
	{REGFLAG_DELAY,120,{}},        
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_initialization_setting[] = {
	// GIP Setting}},
	{0xFF,0x03,{0x98,0x82,0x01}},
	
	{0x00,0x01,{0x44  }},
	{0x01,0x01,{0x13}},
	{0x02,0x01,{0x10}},
	{0x03,0x01,{0x20}},
	{0x04,0x01,{0xCA}},
	{0x05,0x01,{0x13}},
	{0x06,0x01,{0x10}},
	{0x07,0x01,{0x20}},
	{0x08,0x01,{0x82  }},
	{0x09,0x01,{0x09  }},
	{0x0a,0x01,{0xb3}},
	{0x0b,0x01,{0x00}},
	{0x0c,0x01,{0x17}},
	{0x0d,0x01,{0x17}},
	{0x0e,0x01,{0x04  }},
	{0x0f,0x01,{0x04}},
	{0x10,0x01,{0x0A}},
	{0x11,0x01,{0x0A}},
	{0x12,0x01,{0x09}},
	{0x1E,0x01,{0x0A}},
	{0x1F,0x01,{0x0A  }},
	{0x16,0x01,{0x82  }},
	{0x17,0x01,{0x09  }},
	{0x18,0x01,{0x33}},
	{0x19,0x01,{0x00}},
	{0x1a,0x01,{0x17}},
	{0x1b,0x01,{0x17}},
	{0x1c,0x01,{0x04  }},
	{0x1d,0x01,{0x04}},
	{0x20,0x01,{0x09}},
	{0x24,0x01,{0x02  }},
	{0x25,0x01,{0x0b}},
	{0x26,0x01,{0x10}},
	{0x27,0x01,{0x20}},
	{0x2c,0x01,{0x34}},
	{0x31,0x01,{0x07  }},
	{0x32,0x01,{0x2a}},
	{0x33,0x01,{0x2a}},
	{0x34,0x01,{0x0D}},
	{0x35,0x01,{0x28}},
	{0x36,0x01,{0x29}},
	{0x37,0x01,{0x11}},
	{0x38,0x01,{0x13}},
	{0x39,0x01,{0x15}},
	{0x3a,0x01,{0x17}},
	{0x3b,0x01,{0x19}},
	{0x3c,0x01,{0x1b}},
	{0x3d,0x01,{0x09}},
	{0x3e,0x01,{0x07}},
	{0x3f,0x01,{0x07}},
	{0x40,0x01,{0x07}},
	{0x41,0x01,{0x07}},
	{0x42,0x01,{0x07}},
	{0x43,0x01,{0x07}},
	{0x44,0x01,{0x07}},
	{0x45,0x01,{0x07}},
	{0x46,0x01,{0x07}},
	{0x47,0x01,{0x07}},
	{0x48,0x01,{0x2a}},
	{0x49,0x01,{0x2a}},
	{0x4a,0x01,{0x0C}},
	{0x4b,0x01,{0x28}},
	{0x4c,0x01,{0x29}},
	{0x4d,0x01,{0x10  }},
	{0x4e,0x01,{0x12}},
	{0x4f,0x01,{0x14}},
	{0x50,0x01,{0x16}},
	{0x51,0x01,{0x18}},
	{0x52,0x01,{0x1a}},
	{0x53,0x01,{0x08  }},
	{0x54,0x01,{0x07}},
	{0x55,0x01,{0x07}},
	{0x56,0x01,{0x07}},
	{0x57,0x01,{0x07}},
	{0x58,0x01,{0x07  }},
	{0x59,0x01,{0x07  }},
	{0x5a,0x01,{0x07}},
	{0x5b,0x01,{0x07}},
	{0x5c,0x01,{0x07}},
	{0x61,0x01,{0x07  }},
	{0x62,0x01,{0x2a}},
	{0x63,0x01,{0x2a}},
	{0x64,0x01,{0x0D}},
	{0x65,0x01,{0x28}},
	{0x66,0x01,{0x29}},
	{0x67,0x01,{0x11}},
	{0x68,0x01,{0x13}},
	{0x69,0x01,{0x15}},
	{0x6a,0x01,{0x17}},
	{0x6b,0x01,{0x19}},
	{0x6c,0x01,{0x1b}},
	{0x6d,0x01,{0x09}},
	{0x6e,0x01,{0x07}},
	{0x6f,0x01,{0x07}},
	{0x70,0x01,{0x07}},
	{0x71,0x01,{0x07}},
	{0x72,0x01,{0x07}},
	{0x73,0x01,{0x07}},
	{0x74,0x01,{0x07}},
	{0x75,0x01,{0x07}},
	{0x76,0x01,{0x07}},
	{0x77,0x01,{0x07}},
	{0x78,0x01,{0x2a}},
	{0x79,0x01,{0x2a}},
	{0x7a,0x01,{0x0C}},
	{0x7b,0x01,{0x28}},
	{0x7c,0x01,{0x29}},
	{0x7d,0x01,{0x10  }},
	{0x7e,0x01,{0x12}},
	{0x7f,0x01,{0x14}},
	{0x80,0x01,{0x16  }},
	{0x81,0x01,{0x18}},
	{0x82,0x01,{0x1a}},
	{0x83,0x01,{0x08  }},
	{0x84,0x01,{0x07}},
	{0x85,0x01,{0x07}},
	{0x86,0x01,{0x07}},
	{0x87,0x01,{0x07}},
	{0x88,0x01,{0x07  }},
	{0x89,0x01,{0x07  }},
	{0x8a,0x01,{0x07}},
	{0x8b,0x01,{0x07}},
	{0x8c,0x01,{0x07}},
	{0xA0,0x01,{0x01}},
	{0xA2,0x01,{0x00}},
	{0xA3,0x01,{0x00}},
	{0xA4,0x01,{0x00}},
	{0xA5,0x01,{0x00}},
	{0xA6,0x01,{0x00}},
	{0xA7,0x01,{0x00}},   // modify for sensor mura by shawn_20200515
	{0xA8,0x01,{0x00}},   // modify for sensor mura by shawn_20200515
	{0xA9,0x01,{0x04}},
	{0xAA,0x01,{0x04}},
	{0xAB,0x01,{0x00}},   // modify for sensor mura by shawn_20200515
	{0xAC,0x01,{0x00}},   // modify for sensor mura by shawn_20200515
	{0xAD,0x01,{0x04}},
	{0xAE,0x01,{0x04}},
	{0xB0,0x01,{0x33}},
	{0xB1,0x01,{0x33}},
	{0xB2,0x01,{0x00}},
	{0xB9,0x01,{0x40}},
	{0xC3,0x01,{0xFF}},
	{0xCA,0x01,{0x44}},
	{0xD0,0x01,{0x01}},
	{0xD1,0x01,{0x00}},
	{0xDC,0x01,{0x37}},
	{0xDD,0x01,{0x42}},
	{0xE2,0x01,{0x00}},
	{0xE6,0x01,{0x23}},
	{0xE7,0x01,{0x54}},
	{0xED,0x01,{0x00}},
	
	// RTN. Internal VBP,0x Internal VFP
	{0xFF,0x03,{0x98,0x82,0x02}},
	{0xF1,0x01,{0x1C }},   // Tcon ESD option
	{0x4B,0x01,{0x5A }},   // line_chopper
	{0x50,0x01,{0xCA }},   // line_chopper
	{0x51,0x01,{0x00}}, 	// line_chopper
	{0x06,0x01,{0x8F}}, 	// Internal Line Time (RTN)
	{0x0B,0x01,{0xA0 }},	// Internal VFP[9]
	{0x0C,0x01,{0x00 }},	// Internal VFP[8]
	{0x0D,0x01,{0x12 }},	// Internal VBP ( ¨Q 255 )
	{0x0E,0x01,{0xE4 }},	// Internal VFP  ( ¨Q 1023 )
	{0x4E,0x01,{0x11 }},	// SRC BIAS
	{0x4D,0x01,{0xCE }},	// Power Saving Off
	{0xF2,0x01,{0x4A }},	// Reset Option
	
	
	// Power Setting
	{0xFF,0x03,{0x98,0x82,0x05}},
	{0x03,0x01,{0x01}},  //VCOM
	{0x04,0x01,{0x13}},  //FC //VCOM  -1.4v
	//{0x46,0x01,0x00	  // LVD HVREG option}},
	{0x47,0x01,{0x0A}},
	{0x85,0x01,{0x77 }},	// HW RESET option
	{0x63,0x01,{0x9C }}, //GVDDN -5.6v
	{0x64,0x01,{0x9C}},  //GVDDP +5.6v
	{0x68,0x01,{0x8D }},	// VGHO = 14V
	{0x69,0x01,{0x93 }},	// VGH = 15V
	{0x6A,0x01,{0xC9 }},	// VGLO = -14V
	{0x6B,0x01,{0xBB }},	// VGL = -15V
	{0xC8,0x01,{0x8D }},	// VGHO_C = 14V
	{0xC9,0x01,{0x8D }},	// VGHO_L = 14V
	{0xCA,0x01,{0x8D }},	// VGHO_M = 14V
	{0xCB,0x01,{0x4D }},	// VGHO_H = 10.8V
	{0xD0,0x01,{0x93 }},	// VGH_C = 15V
	{0xD1,0x01,{0x93 }},	// VGH_L = 15V
	{0xD2,0x01,{0x93 }},	// VGH_M = 15V
	{0xD3,0x01,{0x53 }},	// VGH_H = 11.8V
	
	// Resolution}},
	{0xFF,0x03,{0x98,0x82,0x06}},
	{0xD9,0x01,{0x1F }},	// 4Lane
	{0xC0,0x01,{0x40 }},	// NL = 1600
	{0xC1,0x01,{0x16 }},	// NL = 1600
	{0x80,0x01,{0x09}},
	
	{0xFF,0x03,{0x98,0x82,0x07}},
	{0xC0,0x01,{0x01 }},	// TS Enable
	{0xCB,0x01,{0xCF }},	// TS_TH0  (-25 C)
	{0xCC,0x01,{0xB0 }},	// TS_TH1  (5 C)
	{0xCD,0x01,{0x9D }},	// TS_TH2 (25 C)
	{0xCE,0x01,{0x7E }},	// TS_TH3  (56 C)
	
	
	//Gamma Register}},
	{0xFF,0x03,{0x98,0x82,0x08}},
	{0xE0,0x27,{0x00,0x24,0x52,0x76,0xA7,0x50,0xD3,0xF7,0x24,0x48,0x95,0x83,0xB4,0xE0,0x0A,0xAA,0x36,0x6C,0x8F,0xBC,0xFE,0xE2,0x15,0x54,0x89,0x03,0xEC}},
	{0xE1,0x27,{0x00,0x24,0x52,0x76,0xA7,0x50,0xD3,0xF7,0x24,0x48,0x95,0x83,0xB4,0xE0,0x0A,0xAA,0x36,0x6C,0x8F,0xBC,0xFE,0xE2,0x15,0x54,0x89,0x03,0xEC}},
	
	
	// OSC Auto Trim Setting}},
	{0xFF,0x03,{0x98,0x82,0x0B}},
	{0x9A,0x01,{0x89}},
	{0x9B,0x01,{0x05}},
	{0x9C,0x01,{0x06}},
	{0x9D,0x01,{0x06}},
	{0x9E,0x01,{0xE1}},
	{0x9F,0x01,{0xE1}},
	{0xAA,0x01,{0x22}},
	{0xAB,0x01,{0xE0}}, 	// AutoTrimType
	{0xAC,0x01,{0x7F }},	// trim_osc_max
	{0xAD,0x01,{0x3F }},	// trim_osc_min
	
	// TP Setting}},
	{0xFF,0x03,{0x98,0x82,0x0E}},
	{0x11,0x01,{0x10 }},	//TSVD Rise Poisition
	{0x12,0x01,{0x08 }},	// LV mode TSHD Rise position
	{0x13,0x01,{0x10 }},	// LV mode TSHD Rise position
	{0x00,0x01,{0xA0 }},	// LV mode
	
	
	{0xFF,0x03,{0x98,0x82,0x00}},
	{0x35,0x01,{0x00}}, //TE enable
	{0x11,0x01,{0x00}},
	{REGFLAG_DELAY,120,{}},
	{0x29,0x01,{0x00}},

	{REGFLAG_DELAY,20,{}},

	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
    unsigned int i;
    LCM_LOGI("nt35695----tps6132-lcm_init   push_table++++++++++++++===============================devin----\n");
    for (i = 0; i < count; i++)
    {
        unsigned cmd;
        cmd = table[i].cmd;

        switch (cmd)
        {
        case REGFLAG_DELAY:
            if (table[i].count <= 10)
                MDELAY(table[i].count);
            else
                MDELAY(table[i].count);
            break;

        case REGFLAG_END_OF_TABLE:
            break;

        default:
            dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
        }
    }
}

/* --------------------------------------------------------------------------- */
/* LCM Driver Implementations */
/* --------------------------------------------------------------------------- */

static void lcm_set_util_funcs(const  struct  LCM_UTIL_FUNCS *util)
{
    memcpy(&lcm_util, util, sizeof( struct  LCM_UTIL_FUNCS));
}

static void lcm_get_params( struct  LCM_PARAMS *params)
{
    memset(params, 0, sizeof( struct  LCM_PARAMS));

    params->type                         = LCM_TYPE_DSI;
    params->width                        = FRAME_WIDTH;
    params->height                       = FRAME_HEIGHT;

#ifndef BUILD_LK
	params->physical_width               = 68;     //LCM_PHYSICAL_WIDTH/1000;
	params->physical_height              = 151;    //LCM_PHYSICAL_HEIGHT/1000;
	params->physical_width_um            = 67928;  //LCM_PHYSICAL_WIDTH; = sqrt((size*25.4)^2/(18^2+9^2))*9*1000
	params->physical_height_um           = 150952; //LCM_PHYSICAL_HEIGHT; = sqrt((size*25.4)^2/(18^2+9^2))*18*1000
#endif

    // enable tearing-free
    params->dbi.te_mode                  = LCM_DBI_TE_MODE_DISABLED;
    params->dbi.te_edge_polarity         = LCM_POLARITY_RISING;

#if (LCM_DSI_CMD_MODE)
    params->dsi.mode                     = CMD_MODE;
    params->dsi.switch_mode              = SYNC_PULSE_VDO_MODE;
#else
    params->dsi.mode                     = SYNC_PULSE_VDO_MODE;//SYNC_EVENT_VDO_MODE;//BURST_VDO_MODE;////
#endif

    // DSI
    /* Command mode setting */
    //1 Three lane or Four lane
    params->dsi.LANE_NUM                 = LCM_FOUR_LANE;
    //The following defined the fomat for data coming from LCD engine.
    params->dsi.data_format.color_order  = LCM_COLOR_ORDER_RGB;
    params->dsi.data_format.trans_seq    = LCM_DSI_TRANS_SEQ_MSB_FIRST;
    params->dsi.data_format.padding      = LCM_DSI_PADDING_ON_LSB;
    params->dsi.data_format.format       = LCM_DSI_FORMAT_RGB888;

    params->dsi.PS                       = LCM_PACKED_PS_24BIT_RGB888;

#if (LCM_DSI_CMD_MODE)
    params->dsi.intermediat_buffer_num   = 0;//because DSI/DPI HW design change, this parameters should be 0 when video mode in MT658X; or memory leakage
    params->dsi.word_count               = FRAME_WIDTH * 3; //DSI CMD mode need set these two bellow params, different to 6577
#else
    params->dsi.intermediat_buffer_num   = 2;	//because DSI/DPI HW design change, this parameters should be 0 when video mode in MT658X; or memory leakage
#endif

    // Video mode setting
    params->dsi.packet_size              = 256;

	params->dsi.vertical_sync_active				= 2;//2;
	params->dsi.vertical_backporch					= 16;//8;
	params->dsi.vertical_frontporch                 = 228; // 230;
	params->dsi.vertical_active_line				= FRAME_HEIGHT;

	params->dsi.horizontal_sync_active              = 16; // 8;
	params->dsi.horizontal_backporch				= 20;//20;
	params->dsi.horizontal_frontporch				= 20;//40;
	params->dsi.horizontal_active_pixel             = FRAME_WIDTH;

    params->dsi.esd_check_enable = 1;
    params->dsi.customization_esd_check_enable = 1;
    params->dsi.lcm_esd_check_table[0].cmd          = 0x0a;
    params->dsi.lcm_esd_check_table[0].count        = 1;
    params->dsi.lcm_esd_check_table[0].para_list[0] = 0x9c;  
	params->dsi.PLL_CLOCK = 275;
	
	// params->dsi.ssc_disable = 1;
}

static unsigned int lcm_compare_id(void)
{
    #define LCM_ID 0x0098820d
    int array[4];
    char buffer[4]={0,0,0,0};
    int id = 0;

	display_ldo18_enable(1);
	MDELAY(5);
	display_bias_vpos_set(5800);
	display_bias_vneg_set(5800);
	//display_bias_enable();
	mt_dsi_pinctrl_set(LCM_RESET_PIN_NO, 1);
    //SET_RESET_PIN(1);
    MDELAY(10);
	mt_dsi_pinctrl_set(LCM_RESET_PIN_NO, 1);
    //SET_RESET_PIN(0);
    MDELAY(20);
	mt_dsi_pinctrl_set(LCM_RESET_PIN_NO, 1);
    //SET_RESET_PIN(1);
    MDELAY(60);

	//{0x39, 0xFF, 5, { 0xFF,0x98,0x06,0x04,0x01}}, // Change to Page 1 CMD
	array[0]=0x00043902;
    array[1]=0x068298ff;
    dsi_set_cmdq(array, 2, 1);

    MDELAY(10);
    array[0] = 0x00083700;
    dsi_set_cmdq(array, 1, 1);

    MDELAY(10);
    read_reg_v2(0xf0, &buffer[0], 1);//    NC 0x00  0x98 0x16
    MDELAY(10);
    read_reg_v2(0xf1, &buffer[1], 1);//    NC 0x00  0x98 0x16
    MDELAY(10);
    read_reg_v2(0xf2, &buffer[2], 1);//    NC 0x00  0x98 0x16

	id = (buffer[0]<<16) | (buffer[1]<<8) | buffer[2];

#ifdef BUILD_LK
	printf("cjx:%s, LK debug: ili9881h id = 0x%08x\n", __func__, id);
#else
	printk("cjx:%s: ili9881h id = 0x%08x \n", __func__, id);
#endif

    return (LCM_ID == id)?1:0;
}


static void lcm_init(void)
{
    LCM_LOGI("cjx : lcm_init +++++++++++++++++++++++++++++\n");
    
    display_bias_enable_v(6000);
	MDELAY(15);
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
    
    push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
}

static void lcm_suspend(void)
{
	printk("%s\n",__func__);

	push_table(lcm_suspend_setting, sizeof(lcm_suspend_setting) / sizeof(struct LCM_setting_table), 1);
	//MDELAY(5);
	//SET_RESET_PIN(0);
    //MDELAY(10);
	display_bias_disable();
}

static void lcm_resume(void)
{
    lcm_init();
}

#if (LCM_DSI_CMD_MODE)
static void lcm_update(unsigned int x, unsigned int y, unsigned int width, unsigned int height)
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
#endif
struct LCM_DRIVER ili9882q_hdp_dsi_vdo_incell_truly_lcm_drv = 
{
  .name			= "ili9882q_hdp_dsi_vdo_incell_truly",
	#if defined(CONFIG_PRIZE_HARDWARE_INFO) && !defined (BUILD_LK)
	.lcm_info = {
		.chip	= "ili9882q",
		.vendor	= "truly",
		.id		= "0x98820q",
		.more	= "1600*720",
	},
	#endif
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.compare_id = lcm_compare_id,
#if (LCM_DSI_CMD_MODE)
    .update         = lcm_update,
#endif
};

