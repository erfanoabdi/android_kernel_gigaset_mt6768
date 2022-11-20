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
{0xFF,3,{0x98,0x82,0x01}},
{0x00,1,{0x46}},
{0x01,1,{0x33}},
{0x02,1,{0x00}},
{0x03,1,{0x00}},
{0x04,1,{0x42}},
{0x05,1,{0x33}},
{0x06,1,{0x00}},
{0x07,1,{0x00}},
{0x08,1,{0x83}},
{0x09,1,{0x03}},
{0x0A,1,{0x72}},
{0x0B,1,{0x00}},
{0x0C,1,{0x00}},
{0x0D,1,{0x00}},
{0x0E,1,{0x00}},
{0x0F,1,{0x00}},
{0x31,1,{0x02}},
{0x32,1,{0x02}},
{0x33,1,{0x02}},
{0x34,1,{0x0E}},
{0x35,1,{0x0C}},
{0x36,1,{0x16}},
{0x37,1,{0x14}},
{0x38,1,{0x12}},
{0x39,1,{0x10}},
{0x3A,1,{0x08}},
{0x3B,1,{0x0A}},
{0x3C,1,{0x07}},
{0x3D,1,{0x07}},
{0x3E,1,{0x07}},
{0x3F,1,{0x07}},
{0x40,1,{0x07}},
{0x41,1,{0x07}},
{0x42,1,{0x07}},
{0x43,1,{0x07}},
{0x44,1,{0x07}},
{0x45,1,{0x07}},
{0x46,1,{0x07}},
{0x47,1,{0x02}},
{0x48,1,{0x02}},
{0x49,1,{0x02}},
{0x4A,1,{0x0F}},
{0x4B,1,{0x0F}},
{0x4C,1,{0x17}},
{0x4D,1,{0x15}},
{0x4E,1,{0x13}},
{0x4F,1,{0x11}},
{0x50,1,{0x09}},
{0x51,1,{0x0B}},
{0x52,1,{0x07}},
{0x53,1,{0x07}},
{0x54,1,{0x07}},
{0x55,1,{0x07}},
{0x56,1,{0x07}},
{0x57,1,{0x07}},
{0x58,1,{0x07}},
{0x59,1,{0x07}},
{0x5A,1,{0x07}},
{0x5B,1,{0x07}},
{0x5C,1,{0x07}},
{0x61,1,{0x02}},
{0x62,1,{0x02}},
{0x63,1,{0x02}},
{0x64,1,{0x09}},
{0x65,1,{0x0B}},
{0x66,1,{0x15}},
{0x67,1,{0x17}},
{0x68,1,{0x11}},
{0x69,1,{0x13}},
{0x6A,1,{0x0F}},
{0x6B,1,{0x0D}},
{0x6C,1,{0x07}},
{0x6D,1,{0x07}},
{0x6E,1,{0x07}},
{0x6F,1,{0x07}},
{0x70,1,{0x07}},
{0x71,1,{0x07}},
{0x72,1,{0x07}},
{0x73,1,{0x07}},
{0x74,1,{0x07}},
{0x75,1,{0x07}},
{0x76,1,{0x07}},
{0x77,1,{0x02}},  //VGLO
{0x78,1,{0x02}},  //VGLO
{0x79,1,{0x02}},  //VGLO
{0x7A,1,{0x08}},  //STV4
{0x7B,1,{0x0A}},  //STV3
{0x7C,1,{0x14}},  //CLK4
{0x7D,1,{0x16}},  //CLK3
{0x7E,1,{0x10}},  //CLK2
{0x7F,1,{0x12}},  //CLK1
{0x80,1,{0x0E}},  //STV1
{0x81,1,{0x0C}},  //STV2
{0x82,1,{0x07}},
{0x83,1,{0x07}},
{0x84,1,{0x07}},
{0x85,1,{0x07}},
{0x86,1,{0x07}},
{0x87,1,{0x07}},
{0x88,1,{0x07}},
{0x89,1,{0x07}},
{0x8A,1,{0x07}},
{0x8B,1,{0x07}},
{0x8C,1,{0x07}},
{0xE7,1,{0x54}},  //V-porch SRC=V0
// RTN. Internal VBP, Internal VFP
{0xFF,3,{0x98,0x82,0x02}},
{0xF1,1,{0x1C}},    // Tcon ESD option
{0x4B,1,{0x5A}},    // line_chopper
{0x50,1,{0xCA}},    // line_chopper
{0x51,1,{0x00}},     // line_chopper
{0x06,1,{0x8D}},     // Internal Line Time (RTN)
{0x0B,1,{0xA0}},     // Internal VFP[9]
{0x0C,1,{0x00}},     // Internal VFP[8]
{0x0D,1,{0x1A}},     // Internal VBP
{0x0E,1,{0xFA}},     // Internal VFP
{0x4E,1,{0x11}},    // SRC BIAS
// {0x4D,01,{0xCE}},     // Power Saving Off
{0x01,1,{0x14}},     // MIPI timeout 160us 
{0x02,1,{0x0A}},     // MIPI timeout 160us

// Power Setting
{0xFF,3,{0x98,0x82,0x05}},
{0x03,1,{0x01}}, //00     // VCOM
{0x04,1,{0x25}}, //D8     // VCOM
{0x63,1,{0x7E}},     // GVDDN = -5V
{0x64,1,{0x7E}},     // GVDDP = 5V
{0x68,1,{0xA1}},     // VGHO = 15V
{0x69,1,{0xA7}},     // VGH = 16V
{0x6A,1,{0x8D}},     // VGLO = -11V
{0x6B,1,{0x7F}},     // VGL = -12V
{0x85,1,{0x37}},      // HW RESET option
{0x46,1,{0x00}},      // LVD HVREG option

// Resolution
{0xFF,3,{0x98,0x82,0x06}},
{0xD9,1,{0x1F}},     // 4Lane
//{0x08,01,{0x00}},     // PLL
{0xC0,1,{0x40}},     // NL = 1600
{0xC1,1,{0x16}},     // NL = 1600

// Gamma Register
{0xFF,3,{0x98,0x82,0x08}},
{0xE0,27,{0x00,0x24,0x49,0x6A,0x97,0x50,0xC1,0xE4,0x0F,0x33,0x55,0x6F,0xA2,0xD0,0xFC,0xAA,0x2A,0x62,0x86,0xB4,0xFE,0xDB,0x0F,0x50,0x86,0x03,0xEC}},
{0xE1,27,{0x00,0x24,0x49,0x6A,0x97,0x50,0xC1,0xE4,0x0F,0x33,0x55,0x6F,0xA2,0xD0,0xFC,0xAA,0x2A,0x62,0x86,0xB4,0xFE,0xDB,0x0F,0x50,0x86,0x03,0xEC}},
// OSC Auto Trim Setting
{0xFF,3,{0x98,0x82,0x0B}},
{0x9A,1,{0x44}},
{0x9B,1,{0x81}},
{0x9C,1,{0x03}},
{0x9D,1,{0x03}},
{0x9E,1,{0x70}},
{0x9F,1,{0x70}},
{0xAB,1,{0xE0}},     // AutoTrimType
								
{0xFF,3,{0x98,0x82,0x0E}},
{0x11,1,{0x10}},     // TSVD Rise position
{0x13,1,{0x10}},     // LV mode TSHD Rise position
{0x00,1,{0xA0}},      // LV mode

{0xFF,3,{0x98,0x82,0x00}},

{0x11,1,{0x00}},                  
{REGFLAG_DELAY,120,{}},	          
{0x29,1,{0x00}},                  
{REGFLAG_DELAY,50,{}},            
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
	params->dsi.vertical_frontporch					= 245;  // rom Q driver
	params->dsi.vertical_active_line				= FRAME_HEIGHT;

	params->dsi.horizontal_sync_active				= 18;//10;
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

#ifndef BUILD_LK
extern atomic_t ESDCheck_byCPU;
#endif

static unsigned int lcm_ata_check(unsigned char *buffer)
{
#if 1
#ifndef BUILD_LK 
unsigned int ret = 0 ,ret1=2; 
//unsigned int x0 = FRAME_WIDTH/4; 
//unsigned int x1 = FRAME_WIDTH*3/4; 
//unsigned int y0 = 0;
//unsigned int y1 = y0 + FRAME_HEIGHT - 1;
unsigned char x0_MSB = 0x5;//((x0>>8)&0xFF); 
unsigned char x0_LSB = 0x2;//(x0&0xFF); 
unsigned char x1_MSB = 0x1;//((x1>>8)&0xFF); 
unsigned char x1_LSB = 0x4;//(x1&0xFF); 
	//unsigned char y0_MSB = ((y0>>8)&0xFF);
	//unsigned char y0_LSB = (y0&0xFF);
	//unsigned char y1_MSB = ((y1>>8)&0xFF);
	//unsigned char y1_LSB = (y1&0xFF);
	
unsigned int data_array[6]; 
unsigned char read_buf[4]; 
unsigned char read_buf1[4]; 
unsigned char read_buf2[4]; 
unsigned char read_buf3[4]; 
#ifdef BUILD_LK 
printf("ATA check kernel size = 0x%x,0x%x,0x%x,0x%x\n",x0_MSB,x0_LSB,x1_MSB,x1_LSB); 
#else 
printk("ATA check kernel size = 0x%x,0x%x,0x%x,0x%x\n",x0_MSB,x0_LSB,x1_MSB,x1_LSB); 
#endif 
//write page frist lhr
//data_array[0]= 0x0002150A;//HS packet 
//data_array[1]= (x1_MSB<<24)|(x0_LSB<<16)|(x0_MSB<<8)|0x51; 
//data_array[2]= (x1_LSB); 
//dsi_set_cmdq(data_array, 3, 1); 
   
data_array[0]= 0x0002390A;//HS packet 
data_array[1]= 0x00002453; 
//data_array[2]= (x1_LSB); 
dsi_set_cmdq(data_array, 2, 1); 
    
 data_array[0]= 0x0002390A;//HS packet 
data_array[1]= 0x0000F05e; 
dsi_set_cmdq(data_array, 2, 1); 
data_array[0]= 0x0002390A;//HS packet 
data_array[1]= 0x00000151; 
dsi_set_cmdq(data_array, 2, 1); 
data_array[0]= 0x0002390A;//HS packet 
data_array[1]= 0x00000355; 
//data_array[2]= (x1_LSB); 
dsi_set_cmdq(data_array, 2, 1); 
 
//data_array[0]= 0x0002150A;//HS packet 
//data_array[1]= (x1_MSB<<24)|(x0_LSB<<16)|(x0_MSB<<8)|0x51; 
//data_array[2]= (x1_LSB); 
//dsi_set_cmdq(data_array, 3, 1); 
 
data_array[0] = 0x00013700; 
dsi_set_cmdq(data_array, 1, 1); 
atomic_set(&ESDCheck_byCPU, 1);
read_reg_v2(0X52, read_buf, 1); 
read_reg_v2(0X56, read_buf1, 1); 
read_reg_v2(0X54, read_buf2, 1); 
read_reg_v2(0X5F, read_buf3, 1);
atomic_set(&ESDCheck_byCPU, 0);
if((read_buf1[0] == 0x03) && (read_buf2[0] == 0x24) /*&& (read_buf3[0] == 0xf0)*/) 
   ret = 1; 
else 
    ret = 0; 
#ifdef BUILD_LK 
printf("ATA read buf kernel size = 0x%x,0x%x,0x%x,0x%x,ret= %d\n",read_buf[0],read_buf[1],read_buf[2],read_buf[3],ret); 
#else 
printk("ATA read buf kernel size = 0x%x,0x%x,0x%x,0x%x,ret= %d ret1= %d\n",read_buf[0],read_buf1[0],read_buf2[0],read_buf3[0],ret,ret1); 
printk("ATA read buf new kernel size = 0x%x,0x%x,0x%x,0x%x,ret= %d ret1= %d\n",read_buf1[0],read_buf1[1],read_buf1[2],read_buf1[3],ret,ret1); 
//printk("ATA read buf new kernel size = 0x%x,0x%x,0x%x,0x%x,ret= %d ret1= %d\n",read_buf1,read_buf1,read_buf1,read_buf1,ret,ret1); 
#endif 
return ret; 
#endif 
#endif
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
	display_bias_enable();
    SET_RESET_PIN(1);
    MDELAY(10);
    SET_RESET_PIN(0);
    MDELAY(20);
    SET_RESET_PIN(1);
    MDELAY(120);

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
    
	SET_RESET_PIN(0);
    display_ldo18_enable(1);
    display_bias_vpos_enable(1);
    display_bias_vneg_enable(1);
    MDELAY(10);
    display_bias_vpos_set(5800);
	display_bias_vneg_set(5800);
	MDELAY(10);
	SET_RESET_PIN(1);
	MDELAY(10);
    SET_RESET_PIN(0);
    MDELAY(20);
    SET_RESET_PIN(1);
    MDELAY(60);
    
    push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
}

static void lcm_suspend(void)
{

	push_table(lcm_suspend_setting, sizeof(lcm_suspend_setting) / sizeof(struct LCM_setting_table), 1);
	SET_RESET_PIN(1);
	MDELAY(1);
	SET_RESET_PIN(0);
	MDELAY(1);
	display_ldo18_enable(0);
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
struct LCM_DRIVER ili9882n_hj_dsi_vdo_ivo_hl_ata_lcm_drv = 
{
  .name			= "ili9882n_hj_dsi_vdo_ivo_hl_ata",
	#if defined(CONFIG_PRIZE_HARDWARE_INFO) && !defined (BUILD_LK)
	.lcm_info = {
		.chip	= "ili9882n",
		.vendor	= "hl",
		.id		= "0x98820d",
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
	.ata_check	= lcm_ata_check
};

