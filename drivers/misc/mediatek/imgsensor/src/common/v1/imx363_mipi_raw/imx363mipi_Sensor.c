/*
* Copyright (C) 2016 MediaTek Inc.
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

/*****************************************************************************
 *
 * Filename:
 * ---------
 *      IMX363mipi_Sensor.c
 *
 * Project:
 * --------
 *      ALPS
 *
 * Description:
 * ------------
 *      Source code of Sensor driver
 *
 *
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/

#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
#include <linux/types.h>

#include "kd_camera_typedef.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"

#include "imx363mipi_Sensor.h"

/****************************Modify Following Strings for Debug****************************/
#define PFX "IMX363_camera_sensor"
#define LOG_1 LOG_INF("IMX363,MIPI 1LANE\n")
#define LOG_2 LOG_INF("preview 1600*1200@30fps,600Mbps/lane; video 1600*1200@30fps,600Mbps/lane; 1600*1200@30fps,600Mbps/lane\n")
/****************************   Modify end    *******************************************/
#define LOG_INF(format, args...) pr_err(PFX "[%s] " format, __FUNCTION__, ##args)
#define LOGE(format, args...)   pr_err(PFX "[%s] " format, __FUNCTION__, ##args)




extern  int imx363_otp_read_pdaf(void);


static DEFINE_SPINLOCK(imgsensor_drv_lock);
typedef struct tag_i2c_write_array{
	kal_uint32 addr;
	kal_uint32 para;
}i2c_write_array;

static i2c_write_array init_setting_array[]={
		 {0x0100, 0x00},
         {0x0136, 0x18},
         {0x0137, 0x00},
         {0x31A3, 0x00},
         {0x64D4, 0x01},
         {0x64D5, 0xAA},
         {0x64D6, 0x01},
         {0x64D7, 0xA9},
         {0x64D8, 0x01},
         {0x64D9, 0xA5},
         {0x64DA, 0x01},
         {0x64DB, 0xA1},
         {0x720A, 0x24},
         {0x720B, 0x89},
         {0x720C, 0x85},
         {0x720D, 0xA1},
         {0x720E, 0x6E},
         {0x729C, 0x59},
         {0x817C, 0xFF},
         {0x817D, 0x80},
         {0x9348, 0x96},
         {0x934B, 0x8C},
         {0x934C, 0x82},
         {0x9353, 0xAA},
         {0x9354, 0xAA},
         {0x4073, 0x30},

};
//write_cmos_sensor(0x0340,0x0C);//FRM_LENGTH_LINES[15:8]
//write_cmos_sensor(0x0341,0xE0);//FRM_LENGTH_LINES[7:0]0x0CE0=3296
//write_cmos_sensor(0x0342,0x21);//LINE_LENGTH_PCK[15:8]
//write_cmos_sensor(0x0343,0x28);//LINE_LENGTH_PCK[7:0]0x2128=8488
static i2c_write_array preview_setting_array[]={
	{0x0112, 0x0A},
	{0x0113, 0x0A},
	{0x0114, 0x03},
	{0x0220, 0x00},
	{0x0221, 0x11},
	{0x0340, 0x0C},
	{0x0341, 0x62},
	{0x0342, 0x22},
	{0x0343, 0x80},
	{0x0381, 0x01},
	{0x0383, 0x01},
	{0x0385, 0x01},
	{0x0387, 0x01},
	{0x0900, 0x00},
	{0x0901, 0x11},
	{0x30F4, 0x02},
	{0x30F5, 0x80},
	{0x30F6, 0x00},
	{0x30F7, 0xc8},
	{0x31A0, 0x00},
	{0x31A5, 0x00},
	{0x31A6, 0x00},
	{0x560F, 0xbe},
	{0x5856, 0x08},
	{0x58D0, 0x10},
	{0x734A, 0x01},
	{0x734F, 0x2b},
	{0x7441, 0x55},
	{0x7914, 0x03},
	{0x7928, 0x04},
	{0x7929, 0x04},
	{0x793F, 0x03},
	{0xBC7B, 0x18},
	{0x0344, 0x00},
	{0x0345, 0x00},
	{0x0346, 0x00},
	{0x0347, 0x00},
	{0x0348, 0x0F},
	{0x0349, 0xBF},
	{0x034A, 0x0B},
	{0x034B, 0xCF},
	{0x034C, 0x0F},
	{0x034D, 0xC0},
	{0x034E, 0x0B},
	{0x034F, 0xD0},
	{0x0408, 0x00},
	{0x0409, 0x00},
	{0x040A, 0x00},
	{0x040B, 0x00},
	{0x040C, 0x0F},
	{0x040D, 0xC0},
	{0x040E, 0x0B},
	{0x040F, 0xD0},
	{0x0301, 0x03},
	{0x0303, 0x02},
	{0x0305, 0x04},
	{0x0306, 0x00},
	{0x0307, 0xd2},
	{0x0309, 0x0A},
	{0x030B, 0x01},
	{0x030D, 0x04},
	{0x030E, 0x00},
	{0x030F, 0xdf},
	{0x0310, 0x01},
	{0x0202, 0x0C},
	{0x0203, 0x52},
	{0x0224, 0x06},
	{0x0225, 0x29},
	{0x0204, 0x00},
	{0x0205, 0x00},
	{0x0216, 0x00},
	{0x0217, 0x00},
	{0x020E, 0x01},
	{0x020F, 0x00},
	{0x0226, 0x01},
	{0x0227, 0x00},
};
#if 0
static i2c_write_array capture_setting_array[]={
	{0x0112, 0x0A},
	{0x0113, 0x0A},
	{0x0114, 0x03},
	{0x0220, 0x00},
	{0x0221, 0x11},
	{0x0340, 0x0C},
	{0x0341, 0x62},
	{0x0342, 0x22},
	{0x0343, 0x80},
	{0x0381, 0x01},
	{0x0383, 0x01},
	{0x0385, 0x01},
	{0x0387, 0x01},
	{0x0900, 0x00},
	{0x0901, 0x11},
	{0x30F4, 0x02},
	{0x30F5, 0x80},
	{0x30F6, 0x00},
	{0x30F7, 0xc8},
	{0x31A0, 0x00},
	{0x31A5, 0x00},
	{0x31A6, 0x00},
	{0x560F, 0xbe},
	{0x5856, 0x08},
	{0x58D0, 0x10},
	{0x734A, 0x01},
	{0x734F, 0x2b},
	{0x7441, 0x55},
	{0x7914, 0x03},
	{0x7928, 0x04},
	{0x7929, 0x04},
	{0x793F, 0x03},
	{0xBC7B, 0x18},
	{0x0344, 0x00},
	{0x0345, 0x00},
	{0x0346, 0x00},
	{0x0347, 0x00},
	{0x0348, 0x0F},
	{0x0349, 0xBF},
	{0x034A, 0x0B},
	{0x034B, 0xCF},
	{0x034C, 0x0F},
	{0x034D, 0xC0},
	{0x034E, 0x0B},
	{0x034F, 0xD0},
	{0x0408, 0x00},
	{0x0409, 0x00},
	{0x040A, 0x00},
	{0x040B, 0x00},
	{0x040C, 0x0F},
	{0x040D, 0xC0},
	{0x040E, 0x0B},
	{0x040F, 0xD0},
	{0x0301, 0x03},
	{0x0303, 0x02},
	{0x0305, 0x04},
	{0x0306, 0x00},
	{0x0307, 0xd2},
	{0x0309, 0x0A},
	{0x030B, 0x01},
	{0x030D, 0x04},
	{0x030E, 0x00},
	{0x030F, 0xdf},
	{0x0310, 0x01},
	{0x0202, 0x0C},
	{0x0203, 0x52},
	{0x0224, 0x06},
	{0x0225, 0x29},
	{0x0204, 0x00},
	{0x0205, 0x00},
	{0x0216, 0x00},
	{0x0217, 0x00},
	{0x020E, 0x01},
	{0x020F, 0x00},
	{0x0226, 0x01},
	{0x0227, 0x00},
};
#endif
static imgsensor_info_struct imgsensor_info = {
	.sensor_id = IMX363_SENSOR_ID, //IMX363MIPI_SENSOR_ID,  /*sensor_id = 0x326*/ //record sensor id defined in Kd_imgsensor.h
	.checksum_value = 0xfa71879b, //checksum value for Camera Auto Test
	.pre = {
		.pclk = 840000000,                           //record different mode's pclk
		.linelength = 8832,                            //record different mode's linelength
		.framelength = 3170,                       //record different mode's framelength
		.startx = 0,                                          //record different mode's startx of grabwindow
		.starty = 0,                                          //record different mode's starty of grabwindow
		.grabwindow_width = 4032,          //record different mode's width of grabwindow
		.grabwindow_height =3024,          //record different mode's height of grabwindow
		/*     following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario         */
		.mipi_data_lp2hs_settle_dc = 85,//unit , ns
		/*     following for GetDefaultFramerateByScenario()         */
		.max_framerate = 300,
	},
	.cap = {/*normal capture*/
		.pclk = 840000000,//OPPXCLK
		.linelength = 8832,
		.framelength = 3170,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4032,
		.grabwindow_height = 3024,
		.mipi_data_lp2hs_settle_dc = 85,//unit , ns
		.max_framerate = 300,//300,
	},
	.cap1 = {/*PIP capture*/ //capture for PIP 24fps relative information, capture1 mode must use same framelength, linelength with Capture mode for shutter calculate
		.pclk = 840000000,
		.linelength = 8832,
		.framelength = 3170,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4032,
		.grabwindow_height = 3024,
		.mipi_data_lp2hs_settle_dc = 85,//unit , ns
		.max_framerate = 150, //less than 13M(include 13M),cap1 max framerate is 24fps,16M max framerate is 20fps, 20M max framerate is 15fps
	},
	.normal_video = {
		.pclk = 840000000,
		.linelength = 8832,
		.framelength = 3170,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4032,
		.grabwindow_height = 3024,
		.mipi_data_lp2hs_settle_dc = 85,//unit , ns
		.max_framerate = 300,//modify
	},
	.hs_video = {/*slow motion*/
		.pclk = 840000000,//OPPXCLK
		.linelength = 8832,
		.framelength = 3170,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4032,
		.grabwindow_height = 3024,
		.mipi_data_lp2hs_settle_dc = 85,//unit , ns
		.max_framerate = 300,//300,
	},
	.slim_video = {/*VT Call*/
		.pclk = 840000000,//OPPXCLK
		.linelength = 8832,
		.framelength = 3170,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4032,
		.grabwindow_height = 3024,
		.mipi_data_lp2hs_settle_dc = 85,//unit , ns
		.max_framerate = 300,//300,
	},
	.margin = 20,                    //sensor framelength & shutter margin
	.min_shutter = 4, //1,              //min shutter
	.max_frame_length = 0xff00,//max framelength by sensor register's limitation
	.ae_shut_delay_frame = 0,    //shutter delay frame for AE cycle, 2 frame with ispGain_delay-shut_delay=2-0=2
	.ae_sensor_gain_delay_frame = 0,//sensor gain delay frame for AE cycle,2 frame with ispGain_delay-sensor_gain_delay=2-0=2
	.ae_ispGain_delay_frame = 2,//isp gain delay frame for AE cycle
	.ihdr_support = 0,     //1, support; 0,not support
	.ihdr_le_firstline = 0,  //1,le first ; 0, se first
	.sensor_mode_num = 5,           //support sensor mode num
	.cap_delay_frame = 2,            //enter capture delay frame num
	.pre_delay_frame = 2,            //enter preview delay frame num
	.video_delay_frame = 2,        //enter video delay frame num
	.hs_video_delay_frame = 2,  //enter high speed video  delay frame num
	.slim_video_delay_frame = 2,//enter slim video delay frame num
	.isp_driving_current = ISP_DRIVING_4MA, //mclk driving current
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,//sensor_interface_type
	.mipi_sensor_type = MIPI_OPHY_NCSI2, //0,MIPI_OPHY_NCSI2;  1,MIPI_OPHY_CSI2
	.mipi_settle_delay_mode = MIPI_SETTLEDELAY_MANUAL,//0,MIPI_SETTLEDELAY_AUTO; 1,MIPI_SETTLEDELAY_MANNUAL
	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_R, //SENSOR_OUTPUT_FORMAT_RAW_Gr,//SENSOR_OUTPUT_FORMAT_RAW_R,//sensor output first pixel color
	.mclk = 24,//mclk value, suggest 24 or 26 for 24Mhz or 26Mhz
	.mipi_lane_num = SENSOR_MIPI_4_LANE,//mipi lane num
	.i2c_addr_table = {0x20,0xff},//record sensor support all write id addr, only supprt 4must end with 0xff
};


static imgsensor_struct imgsensor = {
	.mirror = IMAGE_HV_MIRROR,                                //mirrorflip information
	.sensor_mode = IMGSENSOR_MODE_INIT, //IMGSENSOR_MODE enum value,record current sensor mode,such as: INIT, Preview, Capture, Video,High Speed Video, Slim Video
	.shutter = 0x14d,                                       //current shutter
	.gain = 0xe000,                                                   //current gain
	.dummy_pixel = 0,                                     //current dummypixel
	.dummy_line = 0,                                       //current dummyline
	.current_fps = 300,  //full size current fps : 24fps for PIP, 30fps for Normal or ZSD
	.autoflicker_en = KAL_FALSE,  //auto flicker enable: KAL_FALSE for disable auto flicker, KAL_TRUE for enable auto flicker
	.test_pattern = KAL_FALSE,            //test pattern mode or not. KAL_FALSE for in test pattern mode, KAL_TRUE for normal output
	.current_scenario_id = MSDK_SCENARIO_ID_CAMERA_PREVIEW,//current scenario id
	.hdr_mode = 0, //sensor need support LE, SE with HDR feature
	.i2c_write_id = 0x34,//record current sensor's i2c write id
};

#define IMX363_HDR_TYPE (0x00)
//#define IMX363_BINNING_TYPE (0x10)
static kal_uint16 imx363_type = 0;/*0x00=HDR type, 0x10=binning type*/
/* Sensor output window information */
/*according toIMX363 datasheet p53 image cropping*/
static struct SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[5] ={
	{ 4032, 3024,     0,   0, 4032, 3024, 4032,  3024, 0000, 0000, 4032,  3024,   0,   0, 4032, 3024}, // Preview
	{ 4032, 3024,     0,   0, 4032, 3024, 4032,  3024, 0000, 0000, 4032,  3024,   0,   0, 4032, 3024}, // capture
	{ 4032, 3024,     0,   0, 4032, 3024, 4032,  3024, 0000, 0000, 4032,  3024,   0,   0, 4032, 3024}, // video
	//{ 4032, 3024,    0,   0, 4208, 3120, 4208,  3120, 0000, 0000, 4208,  3120,   0,   0, 4208, 3120}, // video
	{ 4032, 3024,     0,   0, 4032, 3024, 4032,  3024, 0000, 0000, 4032,  3024,   0,   0, 4032, 3024}, //hight speed video
	//{ 4208, 2688,         0,  432, 4208, 2256, 1400,  752 , 0000, 0000, 1400,  752,   0,   0, 1400,  752}, //hight speed video
	{ 4032, 3024,     0,   0, 4032, 3024, 4032,  3024, 0000, 0000, 4032,  3024,   0,   0, 4032, 3024}};// slim video
//{ 4208, 2688,         0,  432, 4208, 2256, 1400,  752 , 0000, 0000, 1400,  752,   0,   0, 1400,  752}};// slim video
static struct SET_PD_BLOCK_INFO_T imgsensor_pd_info =
{
	.i4OffsetX = 0,
	.i4OffsetY = 0,
	.i4PitchX  = 0,
	.i4PitchY  = 0,
	.i4PairNum	=0,
	.i4SubBlkW	=0,
	.i4SubBlkH	=0,
	.i4PosL = {{0,0}},
	.i4PosR = {{0,0}},
	.iMirrorFlip = 0,
	.i4BlockNumX = 0,
	.i4BlockNumY = 0,
	.i4LeFirst = 0,
	.i4Crop = {{0,0},{0,0},{0,0},{0,0},{0,0}, \
		{0,0},{0,0},{0,0},{0,0},{0,0}},
};

static struct SENSOR_VC_INFO_STRUCT SENSOR_VC_INFO[2]=
{
	/* Full mode setting */
	{0x03, 0x0a,	 0x00,		 0x08, 0x40, 0x00,
	0x00, 0x2b, 0x0834, 0x0618, 0x00, 0x35, 0x0280, 0x0001,
	0x01, 0x30, 0x13B0, 0x02f4, 0x03, 0x00, 0x0000, 0x0000},
	/* 2Bin mode setting */
  	{0x03, 0x0a,	 0x00,	 0x08, 0x40, 0x00,
   	0x00, 0x2b, 0x0834, 0x0618, 0x00, 0x35, 0x0280, 0x0001,
   	0x01, 0x2b, 0x07e0, 0x017a, 0x03, 0x00, 0x0000, 0x0000},
};
extern int iReadReg(u16 a_u2Addr , u8 * a_puBuff , u16 i2cId);
extern int iWriteReg(u16 a_u2Addr , u32 a_u4Data , u32 a_u4Bytes , u16 i2cId);
extern void kdSetI2CSpeed(u16 i2cSpeed);
//extern bool read_IMX363_eeprom_pdaf( kal_uint16 addr, BYTE* data, kal_uint32 size);


#define IMX363MIPI_MaxGainIndex (235)
kal_uint16 IMX363MIPI_sensorGainMapping[IMX363MIPI_MaxGainIndex][2] ={
	{64 ,0       },
	{65 ,8       },
	{66 ,16 },
	{67,23},
	{68 ,30 },
	{69 ,37 },
	{70,44},
	{71,50},
	{72 ,57 },
	{73 ,63 },
	{74,69},
	{75 ,75 },
	{76 ,81 },
	{77,86},
	{78 ,92 },
	{79,97},
	{80,102},
	{81 ,107},
	{82 ,112},
	{83,117},
	{84 ,122},
	{85,126},
	{86,131},
	{87,135},
	{88,140},
	{89,144},
	{90,148},
	{91,152},
	{92,156},
	{93,160},
	{94,163},
	{95,167},
	{96,171},
	{97,174},
	{98,178},
	{99,181},
	{100,184},
	{101,188},
	{102,191},
	{103,194},
	{104,197},
	{105,200},
	{106,203},
	{107,206},
	{108,209},
	{109,211},
	{110,214},
	{111,217},
	{112,219},
	{113,222},
	{114,225},
	{115,227},
	{116,230},
	{117,232},
	{118,234},
	{119,237},
	{120,239},
	{121,241},
	{122,243},
	{123,246},
	{124,248},
	{125,250},
	{126,252},
	{127,254},
	{128,256},
	{129,258},
	{130,260},
	{131,262},
	{132,264},
	{133,266},
	{134,267},
	{135,269},
	{136,271},
	{137,273},
	{138,275},
	{139,276},
	{140,278},
	{141,280},
	{142,281},
	{143,283},
	{144,284},
	{145,286},
	{146,288},
	{147,289},
	{148,291},
	{149,292},
	{150,294},
	{151,295},
	{152,296},
	{153,298},
	{154,299},
	{155,301},
	{156,302},
	{157,303},
	{158,305},
	{159,306},
	{160,307},
	{161,308},
	{162,310},
	{163,311},
	{164,312},
	{165,313},
	{166,315},
	{167,316},
	{168,317},
	{169,318},
	{170,319},
	{171,320},
	{172,321},
	{173,323},
	{174,324},
	{175,325},
	{176,326},
	{177,327},
	{178,328},
	{179,329},
	{180,330},
	{181,331},
	{182,332},
	{183,333},
	{184,334},
	{185,335},
	{186,336},
	{187,337},
	{188,338},
	{189,339},
	{191,340},
	{192,341},
	{193,342},
	{194,343},
	{195,344},
	{196,345},
	{197,346},
	{199,347},
	{200,348},
	{201,349},
	{202,350},
	{204,351},
	{205,352},
	{206,353},
	{207,354},
	{209,355},
	{210,356},
	{211,357},
	{213,358},
	{214,359},
	{216,360},
	{217,361},
	{218,362},
	{220,363},
	{221,364},
	{223,365},
	{224,366},
	{226,367},
	{228,368},
	{229,369},
	{231,370},
	{232,371},
	{234,372},
	{236,373},
	{237,374},
	{239,375},
	{241,376},
	{243,377},
	{245,378},
	{246,379},
	{248,380},
	{250,381},
	{252,382},
	{254,383},
	{256,384},
	{258,385},
	{260,386},
	{262,387},
	{264,388},
	{266,389},
	{269,390},
	{271,391},
	{273,392},
	{275,393},
	{278,394},
	{280,395},
	{282,396},
	{285,397},
	{287,398},
	{290,399},
	{293,400},
	{295,401},
	{298,402},
	{301,403},
	{303,404},
	{306,405},
	{309,406},
	{312,407},
	{315,408},
	{318,409},
	{321,410},
	{324,411},
	{328,412},
	{331,413},
	{334,414},
	{338,415},
	{341,416},
	{345,417},
	{349,418},
	{352,419},
	{356,420},
	{360,421},
	{364,422},
	{368,423},
	{372,424},
	{377,425},
	{381,426},
	{386,427},
	{390,428},
	{395,429},
	{400,430},
	{405,431},
	{410,432},
	{415,433},
	{420,434},
	{426,435},
	{431,436},
	{437,437},
	{443,438},
	{449,439},
	{455,440},
	{462,441},
	{468,442},
	{475,443},
	{482,444},
	{489,445},
	{496,446},
	{504,447},
	{512,448},

};
/*
#if defined(__Glenn_Imx363_OTP_SUPPORT__)

struct Glenn_Imx363_otp_struct {
    int flag;
    int MID;
    int Year;
    int Month;
    int Day;
    int RGr_ratio;
    int BGr_ratio;
    int GbGr_ratio;
    int VCM_start;
    int VCM_end;
} Glenn_Imx363_OTP;

#define RGr_ratio_Typical 603
#define BGr_ratio_Typical 560
#define GbGr_ratio_Typical 1024

//static int g_Glenn_Imx363_otp_init;

#endif
*/
/*
static kal_uint16 read_cmos_sensor_byte(kal_uint16 addr)
{
    kal_uint16 get_byte=0;
    char pu_send_cmd[2] = {(char)(addr >> 8) , (char)(addr & 0xFF) };

    //kdSetI2CSpeed(imgsensor_info.i2c_speed); // Add this func to set i2c speed by each sensor
    iReadRegI2C(pu_send_cmd , 2, (u8*)&get_byte,1,imgsensor.i2c_write_id);
    return get_byte;
}
*/
static kal_uint16 read_cmos_sensor(kal_uint32 addr)
{
	kal_uint16 get_byte=0;

	char pu_send_cmd[2] = {(char)(addr >> 8), (char)(addr & 0xFF) };
	iReadRegI2C(pu_send_cmd, 2, (u8*)&get_byte, 1, imgsensor.i2c_write_id);

	return get_byte;
}

static int write_cmos_sensor(kal_uint32 addr, kal_uint32 para)
{
	int ret = 0;
	char pu_send_cmd[3] = {(char)(addr >> 8), (char)(addr & 0xFF), (char)(para & 0xFF)};
	ret = iWriteRegI2C(pu_send_cmd, 3, imgsensor.i2c_write_id);

	return ret;
}
static void set_dummy(void)
{
	//LOG_INF("dummyline = %d, dummypixels = %d \n", imgsensor.dummy_line, imgsensor.dummy_pixel);
	/* you can set dummy by imgsensor.dummy_line and imgsensor.dummy_pixel, or you can set dummy by imgsensor.frame_length and imgsensor.line_length */
	write_cmos_sensor(0x0104, 0x01);

	write_cmos_sensor(0x0340, imgsensor.frame_length >> 8);
	write_cmos_sensor(0x0341, imgsensor.frame_length & 0xFF);
	write_cmos_sensor(0x0342, imgsensor.line_length >> 8);
	write_cmos_sensor(0x0343, imgsensor.line_length & 0xFF);

	write_cmos_sensor(0x0104, 0x00);
}

static kal_uint32 return_sensor_id(void)
{
	int tmp = 0;
	int retry = 10;
	tmp=read_cmos_sensor(0x0005);
	LOG_INF("liuzhixiong return sensor id read =%d \n",tmp);
	if(write_cmos_sensor(0x0A02, 0x7F)==0){
		write_cmos_sensor(0x0A00, 0x01);
		while(retry--)
		{
			if(read_cmos_sensor(0x0A01) == 0x01)
			{
				imx363_type = read_cmos_sensor(0x0A2E);
				LOG_INF("imx363 type = 0x%x(0x00=HDR,0x10=binning)", imx363_type);
				return (kal_uint16)((read_cmos_sensor(0x0A24) << 4) | (read_cmos_sensor(0x0A25) >> 4));
			}
		}
	}

	return 0x00;

}
/*
#if defined(__Glenn_Imx363_OTP_SUPPORT__)


void Glenn_Imx363_read_OTP(struct Glenn_Imx363_otp_struct *otp)
{
    unsigned char val = 0;

    LOG_INF("Glenn_Imx363_read_OTP start.\n");	

    //write_cmos_sensor_byte(0x0A02,0x01);
    //write_cmos_sensor_byte(0x0A00,0x01);
    LOG_INF("16bits write. start.\n");	
    //write_cmos_sensor(0x0A02,0x0100);
    //write_cmos_sensor(0x0A00,0x0100);
    mdelay(5);
    
    val=read_cmos_sensor_byte(0x001C);//flag of info and awb1
    LOG_INF("flag of info and awb 1:%d.\n", val);	
    
    if(val == 0x01)
    {
        otp->flag = 0x01;
        otp->MID = read_cmos_sensor_byte(0x0001);
        otp->Year= read_cmos_sensor_byte(0x0002);
        otp->Month= read_cmos_sensor_byte(0x0003);
        otp->Day= read_cmos_sensor_byte(0x0004);
        otp->RGr_ratio = (read_cmos_sensor_byte(0x001D)<<8)+read_cmos_sensor_byte(0x001E);
        otp->BGr_ratio = (read_cmos_sensor_byte(0x001F)<<8)+read_cmos_sensor_byte(0x0020);
        otp->GbGr_ratio = (read_cmos_sensor_byte(0x0021)<<8)+read_cmos_sensor_byte(0x0022);

       LOG_INF("otp otp info1: flag-%d, MID-%d\n\n\n",otp->flag,otp->MID);
       LOG_INF("otp otp info1: Year-%d, Month-%d,Day-%d,RGr_ratio-%d,BGr_ratio-%d,GbGr_ratio-%d\n\n\n",otp->Year,otp->Month,otp->Day,otp->RGr_ratio,otp->BGr_ratio,otp->GbGr_ratio);       
    }

    
  

    val=read_cmos_sensor_byte(0x0788);//falg of VCM1
    LOG_INF("flag of VCM 1:%d.\n", val);	
    
    if(val == 0x01)
    {
        otp->flag += 0x02;
        otp->VCM_start = (read_cmos_sensor_byte(0x0789)<<8)+read_cmos_sensor_byte(0x078A);
        otp->VCM_end = (read_cmos_sensor_byte(0x078B)<<8)+read_cmos_sensor_byte(0x078C);
        
        LOG_INF("VCM start-end 1:%d-%d.\n", otp->VCM_start, otp->VCM_end);	
    }

    //write_cmos_sensor_byte(0x0A00,0x00);
    //write_cmos_sensor(0x0A00,0x0000);
    
    LOG_INF("Glenn_Imx363_read_OTP complete.\n");	
}
void Glenn_Imx363_apply_OTP(struct Glenn_Imx363_otp_struct *Glenn_Imx363_OTP)
{
    int R_gain,B_gain,Gb_gain,Gr_gain,Base_gain;
    
    LOG_INF("Glenn_Imx363_apply_OTP start.\n");	
    
    if(((Glenn_Imx363_OTP->flag)&0x03) != 0x01) 
    {
        LOG_INF("Glenn_Imx363_apply_OTP error.\n");	
        return;
    }
    
    R_gain = (RGr_ratio_Typical*1000) / Glenn_Imx363_OTP->RGr_ratio;
    B_gain = (BGr_ratio_Typical*1000) / Glenn_Imx363_OTP->BGr_ratio;
    Gb_gain = (GbGr_ratio_Typical*1000) / Glenn_Imx363_OTP->GbGr_ratio;
    Gr_gain = 1000;
    Base_gain = R_gain;
    
    if(Base_gain>B_gain)
    {
        Base_gain=B_gain;
    }
    
    if(Base_gain>Gb_gain)
    {
        Base_gain=Gb_gain;
    }
    
    if(Base_gain>Gr_gain)
    {
        Base_gain=Gr_gain;
    }
    
    R_gain = 0x100 * R_gain / Base_gain;
    B_gain = 0x100 * B_gain / Base_gain;
    Gb_gain = 0x100 * Gb_gain / Base_gain;
    Gr_gain = 0x100 * Gr_gain / Base_gain;
    
    if(Gr_gain>0x100)
    {
        //write_cmos_sensor_byte(0x020E,Gr_gain>>8);
        //write_cmos_sensor_byte(0x020F,Gr_gain&0xff);
        //write_cmos_sensor(0x020E,Gr_gain);
    }
    if(R_gain>0x100)
    {
        //write_cmos_sensor_byte(0x0210,R_gain>>8);
        //write_cmos_sensor_byte(0x0211,R_gain&0xff);
        //write_cmos_sensor(0x0210,R_gain);
    }
    if(B_gain>0x100)
    {
        //write_cmos_sensor_byte(0x0212,B_gain>>8);
        //write_cmos_sensor_byte(0x0213,B_gain&0xff);
        //write_cmos_sensor(0x0212,B_gain);
    }
    if(Gb_gain>0x100)
    {
        //write_cmos_sensor_byte(0x0214,Gb_gain>>8);
        //write_cmos_sensor_byte(0x0215,Gb_gain&0xff);
        //write_cmos_sensor(0x0214,Gb_gain);
    }
    
    LOG_INF("Glenn_Imx363_apply_OTP complete.\n");	
}

#endif
*/

static void set_max_framerate(UINT16 framerate,kal_bool min_framelength_en)
{
	//kal_int16 dummy_line;
	kal_uint32 frame_length = imgsensor.frame_length;
	//unsigned long flags;

	LOG_INF("framerate = %d, min framelength should enable %d \n", framerate,min_framelength_en);

	frame_length = imgsensor.pclk / framerate * 10 / imgsensor.line_length;
	spin_lock(&imgsensor_drv_lock);
	imgsensor.frame_length = (frame_length > imgsensor.min_frame_length) ? frame_length : imgsensor.min_frame_length;
	imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;
	//dummy_line = frame_length - imgsensor.min_frame_length;
	//if (dummy_line < 0)
	//     imgsensor.dummy_line = 0;
	//else
	//     imgsensor.dummy_line = dummy_line;
	//imgsensor.frame_length = frame_length + imgsensor.dummy_line;
	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
	{
		imgsensor.frame_length = imgsensor_info.max_frame_length;
		imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;
	}
	if (min_framelength_en)
		imgsensor.min_frame_length = imgsensor.frame_length;
	spin_unlock(&imgsensor_drv_lock);
	set_dummy();
}       /*     set_max_framerate  */


static void set_shutter(kal_uint16 shutter)
{
	unsigned long flags;
	kal_uint16 realtime_fps = 0;
	spin_lock_irqsave(&imgsensor_drv_lock, flags);
	imgsensor.shutter = shutter;
	spin_unlock_irqrestore(&imgsensor_drv_lock, flags);

	//write_shutter(shutter);
	/* 0x3500, 0x3501, 0x3502 will increase VBLANK to get exposure larger than frame exposure */
	/* AE doesn't update sensor gain at capture mode, thus extra exposure lines must be updated here. */

	// OV Recommend Solution
	// if shutter bigger than frame_length, should extend frame length first
	if(!shutter) shutter = 1; /*avoid 0*/
	spin_lock(&imgsensor_drv_lock);
	if (shutter > imgsensor.min_frame_length - imgsensor_info.margin)
		imgsensor.frame_length = shutter + imgsensor_info.margin;
	else
		imgsensor.frame_length = imgsensor.min_frame_length;

	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
		imgsensor.frame_length = imgsensor_info.max_frame_length;
	spin_unlock(&imgsensor_drv_lock);

	shutter = (shutter < imgsensor_info.min_shutter) ? imgsensor_info.min_shutter : shutter;
	shutter = (shutter > (imgsensor_info.max_frame_length - imgsensor_info.margin)) ? (imgsensor_info.max_frame_length - imgsensor_info.margin) : shutter;

	if (0) {
		realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 / imgsensor.frame_length;
		if(realtime_fps >= 297 && realtime_fps <= 305)
			set_max_framerate(296,0);
		else if(realtime_fps >= 147 && realtime_fps <= 150)
			set_max_framerate(146,0);
		else {
			// Extend frame length
			write_cmos_sensor(0x0104, 0x01);
			write_cmos_sensor(0x0340, imgsensor.frame_length >> 8);
			write_cmos_sensor(0x0341, imgsensor.frame_length & 0xFF);
			write_cmos_sensor(0x0104, 0x00);
		}
	} else {
		// Extend frame length
		write_cmos_sensor(0x0104, 0x01);
		write_cmos_sensor(0x0340, imgsensor.frame_length >> 8);
		write_cmos_sensor(0x0341, imgsensor.frame_length & 0xFF);
		write_cmos_sensor(0x0104, 0x00);
	}

	// Update Shutter
	write_cmos_sensor(0x0104, 0x01);
	write_cmos_sensor(0x0202, (shutter >> 8) & 0xFF);
	write_cmos_sensor(0x0203, shutter  & 0xFF);
	write_cmos_sensor(0x0104, 0x00);
	LOG_INF("Exit! shutter =%d, framelength =%d\n", shutter,imgsensor.frame_length);

}    /*    set_shutter */


static void set_shutter_frame_time(kal_uint16 shutter, kal_uint16 frame_time)
{
	unsigned long flags;
	kal_uint16 realtime_fps = 0;
	spin_lock_irqsave(&imgsensor_drv_lock, flags);
	imgsensor.shutter = shutter;
	spin_unlock_irqrestore(&imgsensor_drv_lock, flags);
	//LOG_INF("shutter =%d, frame_time =%d\n", shutter, frame_time);

	/* 0x3500, 0x3501, 0x3502 will increase VBLANK to get exposure larger than frame exposure */
	/* AE doesn't update sensor gain at capture mode, thus extra exposure lines must be updated here. */

	// OV Recommend Solution
	// if shutter bigger than frame_length, should extend frame length first
	spin_lock(&imgsensor_drv_lock);
	/*Change frame time*/
	imgsensor.dummy_line = (frame_time > imgsensor.frame_length) ? (frame_time - imgsensor.frame_length) : 0;
	imgsensor.frame_length = imgsensor.frame_length + imgsensor.dummy_line;
	imgsensor.min_frame_length = imgsensor.frame_length;
	//
	if (shutter > imgsensor.min_frame_length - imgsensor_info.margin)
		imgsensor.frame_length = shutter + imgsensor_info.margin;
	else
		imgsensor.frame_length = imgsensor.min_frame_length;
	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
		imgsensor.frame_length = imgsensor_info.max_frame_length;
	spin_unlock(&imgsensor_drv_lock);
	shutter = (shutter < imgsensor_info.min_shutter) ? imgsensor_info.min_shutter : shutter;
	shutter = (shutter > (imgsensor_info.max_frame_length - imgsensor_info.margin)) ? (imgsensor_info.max_frame_length - imgsensor_info.margin) : shutter;

	if (0) {
		realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 / imgsensor.frame_length;
		if(realtime_fps >= 297 && realtime_fps <= 305)
			set_max_framerate(296,0);
		else if(realtime_fps >= 147 && realtime_fps <= 150)
			set_max_framerate(146,0);
		else {
			// Extend frame length
			write_cmos_sensor(0x0104, 0x01);
			write_cmos_sensor(0x0340, imgsensor.frame_length >> 8);
			write_cmos_sensor(0x0341, imgsensor.frame_length & 0xFF);
			write_cmos_sensor(0x0104, 0x00);
		}
	} else {
		// Extend frame length
		write_cmos_sensor(0x0104, 0x01);
		write_cmos_sensor(0x0340, imgsensor.frame_length >> 8);
		write_cmos_sensor(0x0341, imgsensor.frame_length & 0xFF);
		write_cmos_sensor(0x0104, 0x00);
	}

	// Update Shutter
	write_cmos_sensor(0x0104, 0x01);
	write_cmos_sensor(0x0202, (shutter >> 8) & 0xFF);
	write_cmos_sensor(0x0203, shutter  & 0xFF);
	write_cmos_sensor(0x0104, 0x00);
	LOG_INF("Exit! shutter =%d, framelength =%d\n", shutter,imgsensor.frame_length);

}    /*    set_shutter */


static kal_uint16 gain2reg(const kal_uint16 gain)
{
	kal_uint8 i;

	for (i = 0; i < IMX363MIPI_MaxGainIndex; i++) {
		if(gain <= IMX363MIPI_sensorGainMapping[i][0]){
			break;
		}
	}
	if(gain != IMX363MIPI_sensorGainMapping[i][0])
		LOG_INF("Gain mapping don't correctly:%d %d \n", gain, IMX363MIPI_sensorGainMapping[i][0]);
	return IMX363MIPI_sensorGainMapping[i][1];
}

/*************************************************************************
 * FUNCTION
 *       set_gain
 *
 * DESCRIPTION
 *       This function is to set global gain to sensor.
 *
 * PARAMETERS
 *       iGain : sensor global gain(base: 0x40)
 *
 * RETURNS
 *       the actually gain set to sensor.
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint16 set_gain(kal_uint16 gain)
{
	kal_uint16 reg_gain;

	/* 0x350A[0:1], 0x350B[0:7] AGC real gain */
	/* [0:3] = N meams N /16 X     */
	/* [4:9] = M meams M X                    */
	/* Total gain = M + N /16 X   */

	//
	if (gain < BASEGAIN || gain > 8 * BASEGAIN) {
		LOG_INF("Error gain setting");

		if (gain < BASEGAIN)
			gain = BASEGAIN;
		else if (gain > 8 * BASEGAIN)
			gain = 8 * BASEGAIN;
	}

	reg_gain = gain2reg(gain);
	spin_lock(&imgsensor_drv_lock);
	imgsensor.gain = reg_gain;
	spin_unlock(&imgsensor_drv_lock);
	LOG_INF("gain = %d , reg_gain = 0x%x\n ", gain, reg_gain);

	write_cmos_sensor(0x0104, 0x01);
	write_cmos_sensor(0x0204, (reg_gain>>8)& 0xFF);
	write_cmos_sensor(0x0205, reg_gain & 0xFF);
	write_cmos_sensor(0x0104, 0x00);

	return gain;
}       /*     set_gain  */

static void ihdr_write_shutter_gain(kal_uint16 le, kal_uint16 se, kal_uint16 gain)
{

	kal_uint16 realtime_fps = 0;
	kal_uint16 reg_gain;
	LOG_INF("le:0x%x, se:0x%x, gain:0x%x\n",le,se,gain);
	spin_lock(&imgsensor_drv_lock);
	if (le > imgsensor.min_frame_length - imgsensor_info.margin)
		imgsensor.frame_length = le + imgsensor_info.margin;
	else
		imgsensor.frame_length = imgsensor.min_frame_length;
	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
		imgsensor.frame_length = imgsensor_info.max_frame_length;
	spin_unlock(&imgsensor_drv_lock);
	if (le < imgsensor_info.min_shutter) le = imgsensor_info.min_shutter;
	if (imgsensor.autoflicker_en) {
		realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 / imgsensor.frame_length;
		if(realtime_fps >= 297 && realtime_fps <= 305)
			set_max_framerate(296,0);
		else if(realtime_fps >= 147 && realtime_fps <= 150)
			set_max_framerate(146,0);
		else {
			write_cmos_sensor(0x0104, 0x01);
			write_cmos_sensor(0x0340, imgsensor.frame_length >> 8);
			write_cmos_sensor(0x0341, imgsensor.frame_length & 0xFF);
			write_cmos_sensor(0x0104, 0x00);
		}
	} else {
		write_cmos_sensor(0x0104, 0x01);
		write_cmos_sensor(0x0340, imgsensor.frame_length >> 8);
		write_cmos_sensor(0x0341, imgsensor.frame_length & 0xFF);
		write_cmos_sensor(0x0104, 0x00);
	}
	write_cmos_sensor(0x0104, 0x01);
	/* Long exposure */
	write_cmos_sensor(0x0202, (le >> 8) & 0xFF);
	write_cmos_sensor(0x0203, le  & 0xFF);
	/* Short exposure */
	write_cmos_sensor(0x0224, (se >> 8) & 0xFF);
	write_cmos_sensor(0x0225, se  & 0xFF);
	reg_gain = gain2reg(gain);
	spin_lock(&imgsensor_drv_lock);
	imgsensor.gain = reg_gain;
	spin_unlock(&imgsensor_drv_lock);
	/* Global analog Gain for Long expo*/
	write_cmos_sensor(0x0204, (reg_gain>>8)& 0xFF);
	write_cmos_sensor(0x0205, reg_gain & 0xFF);
	/* Global analog Gain for Short expo*/
	//write_cmos_sensor(0x0216, (reg_gain>>8)& 0xFF);
	//write_cmos_sensor(0x0217, reg_gain & 0xFF);
	write_cmos_sensor(0x0104, 0x00);
}
#if 0
static void set_mirror_flip(kal_uint8 image_mirror)
{
	kal_uint8  iTemp;
	LOG_INF("image_mirror = %d\n", image_mirror);
	iTemp = read_cmos_sensor(0x0101);
	iTemp&= ~0x03; //Clear the mirror and flip bits.

	switch (image_mirror) {
	case IMAGE_NORMAL:
		write_cmos_sensor(0x0101, iTemp|0x00);
		break;
	case IMAGE_H_MIRROR:
		write_cmos_sensor(0x0101, iTemp|0x01);
		break;
	case IMAGE_V_MIRROR:
		write_cmos_sensor(0x0101, iTemp|0x02);
		break;
	case IMAGE_HV_MIRROR:
		write_cmos_sensor(0x0101, iTemp|0x03);
		break;
	default:
		LOG_INF("Error image_mirror setting\n");
	}

}
#endif 

/*************************************************************************
 * FUNCTION
 *       night_mode
 *
 * DESCRIPTION
 *       This function night mode of sensor.
 *
 * PARAMETERS
 *       bEnable: KAL_TRUE -> enable night mode, otherwise, disable night mode
 *
 * RETURNS
 *       None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static void night_mode(kal_bool enable)
{
	/*No Need to implement this function*/
}       /*     night_mode    */

/*     preview_setting  */
static void imx363_ImageQuality_Setting(void)
{
	int i=0;
	for(i = 0 ; i < sizeof(preview_setting_array)/sizeof(i2c_write_array);i++){
		write_cmos_sensor(preview_setting_array[i].addr,preview_setting_array[i].para);
	}

}


static void sensor_init(void)
{

//#if defined(__Glenn_Imx363_OTP_SUPPORT__)
//    struct Glenn_Imx363_otp_struct otp;
//#endif

	int i=0;
	LOG_INF("E\n");
	//init setting
	//IMX363
	for(i = 0; i< sizeof(init_setting_array)/sizeof(i2c_write_array);i++){
		write_cmos_sensor(init_setting_array[i].addr,init_setting_array[i].para);
	}


	imx363_ImageQuality_Setting();
	/*Need Mirror/Flip*/
	//set_mirror_flip(0);

	//load_IMX363_SPC_Data();
	//write_cmos_sensor(0x7BC8,0x01);
//#if defined(__Glenn_Imx363_OTP_SUPPORT__)
 //       Glenn_Imx363_read_OTP(&otp);
 //       Glenn_Imx363_apply_OTP(&otp);
    
//#endif
	write_cmos_sensor(0x0100,0x00);//stream off
}       /*     sensor_init  */

static void preview_setting(void)
{
	int i=0;
	LOG_INF("preview E %ld %ld %ld\n", sizeof(preview_setting_array), sizeof(i2c_write_array), sizeof(preview_setting_array)/sizeof(i2c_write_array));
	mdelay(10);
	for(i = 0 ; i < sizeof(preview_setting_array)/sizeof(i2c_write_array);i++){
		write_cmos_sensor(preview_setting_array[i].addr,preview_setting_array[i].para);
	}

}

static void capture_setting(kal_uint16 curretfps, MUINT32 linelength)
{
	int i=0;
	LOG_INF("capture E, linelength:%d\n", linelength);

	mdelay(10);
/*	for(i = 0 ; i < sizeof(capture_setting_array)/sizeof(i2c_write_array);i++){
		write_cmos_sensor(capture_setting_array[i].addr,capture_setting_array[i].para);
	}
*/
	for(i = 0 ; i < sizeof(preview_setting_array)/sizeof(i2c_write_array);i++){
		write_cmos_sensor(preview_setting_array[i].addr,preview_setting_array[i].para);
	}

}

static void normal_video_setting(kal_uint16 currefps)
{
	LOG_INF("normal video E\n");
	LOG_INF("E! currefps:%d\n",currefps);
	preview_setting();
	/*Mode2-03 FHD(HDR off)H:2016 V:1136*/
	/*
	write_cmos_sensor(0x0112,0x0A);
	write_cmos_sensor(0x0113,0x0A);
	write_cmos_sensor(0x0114,0x03);
	write_cmos_sensor(0x0220,0x00);
	write_cmos_sensor(0x0221,0x11);
	write_cmos_sensor(0x0340,0x22);
	write_cmos_sensor(0x0341,0xC8);
	write_cmos_sensor(0x0342,0x0C);
	write_cmos_sensor(0x0343,0x48);
	write_cmos_sensor(0x0381,0x01);
	write_cmos_sensor(0x0383,0x01);
	write_cmos_sensor(0x0385,0x01);
	write_cmos_sensor(0x0387,0x01);
	write_cmos_sensor(0x0900,0x01);
	write_cmos_sensor(0x0901,0x22);
	write_cmos_sensor(0x30F4,0x02);
	write_cmos_sensor(0x30F5,0x58);
	write_cmos_sensor(0x30F6,0x00);
	write_cmos_sensor(0x30F7,0x14);
	write_cmos_sensor(0x31A0,0x00);
	write_cmos_sensor(0x31A5,0x00);
	write_cmos_sensor(0x31A6,0x00);
	write_cmos_sensor(0x560F,0xFF);
	write_cmos_sensor(0x5856,0x08);
	write_cmos_sensor(0x58D0,0x10);
	write_cmos_sensor(0x734A,0x01);
	write_cmos_sensor(0x734F,0x2B);
	write_cmos_sensor(0x7441,0x55);
	write_cmos_sensor(0x7914,0x03);
	write_cmos_sensor(0x7928,0x04);
	write_cmos_sensor(0x7929,0x04);
	write_cmos_sensor(0x793F,0x03);
	write_cmos_sensor(0xBC7B,0x18);
	
	
	
	
	write_cmos_sensor(0x0344,0x00);
	write_cmos_sensor(0x0345,0x00);
	write_cmos_sensor(0x0346,0x00);
	write_cmos_sensor(0x0347,0x00);
	write_cmos_sensor(0x0348,0x0F);
	write_cmos_sensor(0x0349,0xBF);
	write_cmos_sensor(0x034A,0x0B);
	write_cmos_sensor(0x034B,0xCF);
	write_cmos_sensor(0x034C,0x07);
	write_cmos_sensor(0x034D,0xE0);
	write_cmos_sensor(0x034E,0x05);
	write_cmos_sensor(0x034F,0xE8);
	write_cmos_sensor(0x0408,0x00);
	write_cmos_sensor(0x0409,0x00);
	write_cmos_sensor(0x040A,0x00);
	write_cmos_sensor(0x040B,0x00);
	write_cmos_sensor(0x040C,0x07);
	write_cmos_sensor(0x040D,0xE0);
	write_cmos_sensor(0x040E,0x05);
	write_cmos_sensor(0x040F,0xE8);
	
	
	
	
	write_cmos_sensor(0x0301,0x03);
	write_cmos_sensor(0x0303,0x02);
	write_cmos_sensor(0x0305,0x04);
	write_cmos_sensor(0x0306,0x00);
	write_cmos_sensor(0x0307,0xD2);
	write_cmos_sensor(0x0309,0x0A);
	write_cmos_sensor(0x030B,0x01);
	write_cmos_sensor(0x030D,0x04);
	write_cmos_sensor(0x030E,0x01);
	write_cmos_sensor(0x030F,0x56);
	write_cmos_sensor(0x0310,0x01);
	
	
	
	
	write_cmos_sensor(0x0202,0x22);
	write_cmos_sensor(0x0203,0xB8);
	write_cmos_sensor(0x0224,0x01);
	write_cmos_sensor(0x0225,0xF4);
	
	
	
	
	write_cmos_sensor(0x0204,0x00);
	write_cmos_sensor(0x0205,0x00);
	write_cmos_sensor(0x0216,0x00);
	write_cmos_sensor(0x0217,0x00);
	write_cmos_sensor(0x020E,0x01);
	write_cmos_sensor(0x020F,0x00);
	write_cmos_sensor(0x0226,0x01);
	write_cmos_sensor(0x0227,0x00);
*/
	LOG_INF("imgsensor.hdr_mode in video mode:%d\n",imgsensor.hdr_mode);

}

static void slim_video_setting(void)
{
	LOG_INF("slim video E\n");
    preview_setting();

	//write_cmos_sensor(0x0220,0x00);


}


/*************************************************************************
 * FUNCTION
 *       get_imgsensor_id
 *
 * DESCRIPTION
 *       This function get the sensor ID
 *
 * PARAMETERS
 *       *sensorID : return the sensor ID
 *
 * RETURNS
 *       None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 get_imgsensor_id(UINT32 *sensor_id)
{
	kal_uint8 i = 0;
	kal_uint8 retry_total = 1;
	kal_uint8 retry_cnt = retry_total;
	//sensor have two i2c address 0x6c 0x6d & 0x21 0x20, we should detect the module used i2c address
	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
			//while(1){
		*sensor_id = return_sensor_id(); 
		//}
			if (*sensor_id == imgsensor_info.sensor_id) {
				LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id,*sensor_id);
#ifdef IMX363_OTP_FUNCTION
				imx363_otp_read_pdaf();//zy
				//read_3P3_eeprom_wb_pdaf();
#endif
				return ERROR_NONE;
			}
			LOG_INF("Read sensor id fail, write id: 0x%x, id: 0x%x\n", imgsensor.i2c_write_id,*sensor_id);
			retry_cnt--;
		} while(retry_cnt > 0);
		i++;
		retry_cnt = retry_total;
	}
	if (*sensor_id != imgsensor_info.sensor_id) {
		// if Sensor ID is not correct, Must set *sensor_id to 0xFFFFFFFF
		*sensor_id = 0xFFFFFFFF;
		return ERROR_SENSOR_CONNECT_FAIL;
	}
	return ERROR_NONE;
}


/*************************************************************************
 * FUNCTION
 *       open
 *
 * DESCRIPTION
 *       This function initialize the registers of CMOS sensor
 *
 * PARAMETERS
 *       None
 *
 * RETURNS
 *       None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 open(void)
{
	//const kal_uint8 i2c_addr[] = {IMGSENSOR_WRITE_ID_1, IMGSENSOR_WRITE_ID_2};
	kal_uint8 i = 0;
	kal_uint8 retry = 2;
	kal_uint32 sensor_id = 0;
	LOG_1;
	LOG_2;
	//sensor have two i2c address 0x6c 0x6d & 0x21 0x20, we should detect the module used i2c address
	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
			sensor_id = return_sensor_id();
			if (sensor_id == imgsensor_info.sensor_id) {
				LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id,sensor_id);
				break;
			}
			LOG_INF("Read sensor id fail, write id: 0x%x, id: 0x%x\n", imgsensor.i2c_write_id,sensor_id);
			retry--;
		} while(retry > 0);
		i++;
		if (sensor_id == imgsensor_info.sensor_id)
			break;
		retry = 2;
	}
	if (imgsensor_info.sensor_id != sensor_id)
		return ERROR_SENSOR_CONNECT_FAIL;

	/* initail sequence write in  */
	sensor_init();

	spin_lock(&imgsensor_drv_lock);

	imgsensor.autoflicker_en= KAL_FALSE;
	imgsensor.sensor_mode = IMGSENSOR_MODE_INIT;
	imgsensor.shutter = 0x3D0;
	imgsensor.gain = 0x0100; //0x100;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	imgsensor.frame_length = imgsensor_info.pre.framelength;
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.dummy_pixel = 0;
	imgsensor.dummy_line = 0;
	imgsensor.hdr_mode = 0;
	imgsensor.test_pattern = KAL_FALSE;
	imgsensor.current_fps = imgsensor_info.pre.max_framerate;
	spin_unlock(&imgsensor_drv_lock);

	return ERROR_NONE;
}       /*     open  */



/*************************************************************************
 * FUNCTION
 *       close
 *
 * DESCRIPTION
 *
 *
 * PARAMETERS
 *       None
 *
 * RETURNS
 *       None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 close(void)
{
	LOG_INF("E\n");

	/*No Need to implement this function*/

	return ERROR_NONE;
}       /*     close  */


/*************************************************************************
 * FUNCTION
 * preview
 *
 * DESCRIPTION
 *       This function start the sensor preview.
 *
 * PARAMETERS
 *       *image_window : address pointer of pixel numbers in one period of HSYNC
 *  *sensor_config_data : address pointer of line numbers in one period of VSYNC
 *
 * RETURNS
 *       None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 preview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");
	
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_PREVIEW;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	//imgsensor.video_mode = KAL_FALSE;
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.frame_length = imgsensor_info.pre.framelength;
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	preview_setting();
	//set_mirror_flip(imgsensor.mirror);
	return ERROR_NONE;
}       /*     preview   */

/*************************************************************************
 * FUNCTION
 *       capture
 *
 * DESCRIPTION
 *       This function setup the CMOS sensor in capture MY_OUTPUT mode
 *
 * PARAMETERS
 *
 * RETURNS
 *       None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 capture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E, linelength=%d\n", sensor_config_data->Pixels);
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CAPTURE;
	if (imgsensor.current_fps == imgsensor_info.cap1.max_framerate) {//PIP capture: 24fps for less than 13M, 20fps for 16M,15fps for 20M
		imgsensor.pclk = imgsensor_info.cap1.pclk;
		imgsensor.line_length = imgsensor_info.cap1.linelength;
		imgsensor.frame_length = imgsensor_info.cap1.framelength;
		imgsensor.min_frame_length = imgsensor_info.cap1.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	}

	else {
		if (imgsensor.current_fps != imgsensor_info.cap.max_framerate)
			LOG_INF("Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n",imgsensor.current_fps,imgsensor_info.cap.max_framerate/10);
		imgsensor.pclk = imgsensor_info.cap.pclk;
		imgsensor.line_length = imgsensor_info.cap.linelength;
		imgsensor.frame_length = imgsensor_info.cap.framelength;
		imgsensor.min_frame_length = imgsensor_info.cap.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	}
	spin_unlock(&imgsensor_drv_lock);
	capture_setting(imgsensor.current_fps, sensor_config_data->Pixels);
     //preview_setting();
	//set_mirror_flip(imgsensor.mirror); //
	mdelay(100);
	return ERROR_NONE;
}       /* capture() */
static kal_uint32 normal_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			       MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_VIDEO;
	imgsensor.pclk = imgsensor_info.normal_video.pclk;
	imgsensor.line_length = imgsensor_info.normal_video.linelength;
	imgsensor.frame_length = imgsensor_info.normal_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.normal_video.framelength;
	//imgsensor.current_fps = 300;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	LOG_INF("ihdr enable :%d\n", imgsensor.hdr_mode);
	normal_video_setting(imgsensor.current_fps);
	return ERROR_NONE;
}       /*     normal_video   */

static kal_uint32 hs_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			   MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_HIGH_SPEED_VIDEO;
	imgsensor.pclk = imgsensor_info.hs_video.pclk;
	//imgsensor.video_mode = KAL_TRUE;
	imgsensor.line_length = imgsensor_info.hs_video.linelength;
	imgsensor.frame_length = imgsensor_info.hs_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.hs_video.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	//imgsensor.current_fps = 600;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	normal_video_setting(imgsensor.current_fps);

	return ERROR_NONE;
}       /*     hs_video   */

static kal_uint32 slim_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			     MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_SLIM_VIDEO;
	imgsensor.pclk = imgsensor_info.slim_video.pclk;
	//imgsensor.video_mode = KAL_TRUE;
	imgsensor.line_length = imgsensor_info.slim_video.linelength;
	imgsensor.frame_length = imgsensor_info.slim_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.slim_video.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	//imgsensor.current_fps = 1200;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	slim_video_setting();

	return ERROR_NONE;
}       /*     slim_video       */



static kal_uint32 get_resolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *sensor_resolution)
{
	LOG_INF("E\n");
	sensor_resolution->SensorFullWidth = imgsensor_info.cap.grabwindow_width;
	sensor_resolution->SensorFullHeight = imgsensor_info.cap.grabwindow_height;

	sensor_resolution->SensorPreviewWidth = imgsensor_info.pre.grabwindow_width;
	sensor_resolution->SensorPreviewHeight = imgsensor_info.pre.grabwindow_height;

	sensor_resolution->SensorVideoWidth = imgsensor_info.normal_video.grabwindow_width;
	sensor_resolution->SensorVideoHeight = imgsensor_info.normal_video.grabwindow_height;


	sensor_resolution->SensorHighSpeedVideoWidth       = imgsensor_info.hs_video.grabwindow_width;
	sensor_resolution->SensorHighSpeedVideoHeight      = imgsensor_info.hs_video.grabwindow_height;

	sensor_resolution->SensorSlimVideoWidth = imgsensor_info.slim_video.grabwindow_width;
	sensor_resolution->SensorSlimVideoHeight         = imgsensor_info.slim_video.grabwindow_height;
	return ERROR_NONE;
}       /*     get_resolution        */

static kal_uint32 get_info(enum MSDK_SCENARIO_ID_ENUM scenario_id,
			   MSDK_SENSOR_INFO_STRUCT *sensor_info,
			   MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("scenario_id = %d\n", scenario_id);
	sensor_info->SensorClockPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorClockFallingPolarity = SENSOR_CLOCK_POLARITY_LOW; /* not use */
	sensor_info->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW; // inverse with datasheet
	sensor_info->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorInterruptDelayLines = 4; /* not use */
	sensor_info->SensorResetActiveHigh = FALSE; /* not use */
	sensor_info->SensorResetDelayCount = 5; /* not use */

	sensor_info->SensroInterfaceType = imgsensor_info.sensor_interface_type;
	sensor_info->MIPIsensorType = imgsensor_info.mipi_sensor_type;
	sensor_info->SettleDelayMode = imgsensor_info.mipi_settle_delay_mode;

	sensor_info->SensorOutputDataFormat = imgsensor_info.sensor_output_dataformat;

	sensor_info->CaptureDelayFrame = imgsensor_info.cap_delay_frame;
	sensor_info->PreviewDelayFrame = imgsensor_info.pre_delay_frame;
	sensor_info->VideoDelayFrame = imgsensor_info.video_delay_frame;
	sensor_info->HighSpeedVideoDelayFrame = imgsensor_info.hs_video_delay_frame;
	sensor_info->SlimVideoDelayFrame = imgsensor_info.slim_video_delay_frame;

	sensor_info->SensorMasterClockSwitch = 0; /* not use */
	sensor_info->SensorDrivingCurrent = imgsensor_info.isp_driving_current;

	sensor_info->AEShutDelayFrame = imgsensor_info.ae_shut_delay_frame;               /* The frame of setting shutter default 0 for TG int */
	sensor_info->AESensorGainDelayFrame = imgsensor_info.ae_sensor_gain_delay_frame;     /* The frame of setting sensor gain */
	sensor_info->AEISPGainDelayFrame = imgsensor_info.ae_ispGain_delay_frame;
	sensor_info->IHDR_Support = imgsensor_info.ihdr_support;
	sensor_info->IHDR_LE_FirstLine = imgsensor_info.ihdr_le_firstline;
	sensor_info->SensorModeNum = imgsensor_info.sensor_mode_num;

	sensor_info->SensorMIPILaneNumber = imgsensor_info.mipi_lane_num;
	sensor_info->SensorClockFreq = imgsensor_info.mclk;
	sensor_info->SensorClockDividCount = 3; /* not use */
	sensor_info->SensorClockRisingCount = 0;
	sensor_info->SensorClockFallingCount = 2; /* not use */
	sensor_info->SensorPixelClockCount = 3; /* not use */
	sensor_info->SensorDataLatchCount = 2; /* not use */

	sensor_info->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
	sensor_info->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
	sensor_info->SensorWidthSampling = 0;  // 0 is default 1x
	sensor_info->SensorHightSampling = 0;       // 0 is default 1x
	sensor_info->SensorPacketECCOrder = 1;
#if 0 // hct-drv  zhb
		sensor_info->PDAF_Support = 1;
#else
		sensor_info->PDAF_Support = 5;
#endif

	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.pre.mipi_data_lp2hs_settle_dc;

		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		sensor_info->SensorGrabStartX = imgsensor_info.cap.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.cap.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.cap.mipi_data_lp2hs_settle_dc;

		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:

		sensor_info->SensorGrabStartX = imgsensor_info.normal_video.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.normal_video.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.normal_video.mipi_data_lp2hs_settle_dc;

		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		sensor_info->SensorGrabStartX = imgsensor_info.hs_video.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.hs_video.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.hs_video.mipi_data_lp2hs_settle_dc;

		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		sensor_info->SensorGrabStartX = imgsensor_info.slim_video.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.slim_video.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.slim_video.mipi_data_lp2hs_settle_dc;

		break;
	default:
		sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.pre.mipi_data_lp2hs_settle_dc;
		break;
	}

	return ERROR_NONE;
}       /*     get_info  */


static kal_uint32 control(enum MSDK_SCENARIO_ID_ENUM scenario_id, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("scenario_id = %d\n", scenario_id);
	spin_lock(&imgsensor_drv_lock);
	imgsensor.current_scenario_id = scenario_id;
	spin_unlock(&imgsensor_drv_lock);
	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		preview(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		capture(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		normal_video(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		hs_video(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		slim_video(image_window, sensor_config_data);
		break;
	default:
		LOG_INF("Error ScenarioId setting");
		preview(image_window, sensor_config_data);
		return ERROR_INVALID_SCENARIO_ID;
	}
	return ERROR_NONE;
}       /* control() */



static kal_uint32 set_video_mode(UINT16 framerate)
{
	LOG_INF("framerate = %d\n ", framerate);
	// SetVideoMode Function should fix framerate
	if (framerate == 0)
		// Dynamic frame rate
		return ERROR_NONE;
	spin_lock(&imgsensor_drv_lock);
	if ((framerate == 300) && (imgsensor.autoflicker_en == KAL_TRUE))
		imgsensor.current_fps = 296;
	else if ((framerate == 150) && (imgsensor.autoflicker_en == KAL_TRUE))
		imgsensor.current_fps = 146;
	else
		imgsensor.current_fps = framerate;
	spin_unlock(&imgsensor_drv_lock);
	set_max_framerate(imgsensor.current_fps,1);

	return ERROR_NONE;
}

static kal_uint32 set_auto_flicker_mode(kal_bool enable, UINT16 framerate)
{
	LOG_INF("enable = %d, framerate = %d \n", enable, framerate);
	spin_lock(&imgsensor_drv_lock);
	if (enable) //enable auto flicker
		imgsensor.autoflicker_en = KAL_TRUE;
	else //Cancel Auto flick
		imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}

static kal_uint32 set_max_framerate_by_scenario(enum MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 framerate)
{
	kal_uint32 frame_length;

	LOG_INF("scenario_id = %d, framerate = %d\n", scenario_id, framerate);

	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		frame_length = imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.pre.framelength) ? (frame_length - imgsensor_info.pre.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		//set_dummy();
		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		if(framerate == 0)
			return ERROR_NONE;
		frame_length = imgsensor_info.normal_video.pclk / framerate * 10 / imgsensor_info.normal_video.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.normal_video.framelength) ? (frame_length - imgsensor_info.normal_video.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.normal_video.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		//set_dummy();
		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		if (imgsensor.current_fps == imgsensor_info.cap1.max_framerate) {
			frame_length = imgsensor_info.cap1.pclk / framerate * 10 / imgsensor_info.cap1.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.cap1.framelength) ? (frame_length - imgsensor_info.cap1.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.cap1.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
		} else {
			if (imgsensor.current_fps != imgsensor_info.cap.max_framerate)
				LOG_INF("Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n",framerate,imgsensor_info.cap.max_framerate/10);
			frame_length = imgsensor_info.cap.pclk / framerate * 10 / imgsensor_info.cap.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.cap.framelength) ? (frame_length - imgsensor_info.cap.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.cap.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
		}
		//set_dummy();
		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		frame_length = imgsensor_info.hs_video.pclk / framerate * 10 / imgsensor_info.hs_video.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.hs_video.framelength) ? (frame_length - imgsensor_info.hs_video.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.hs_video.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		//set_dummy();
		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		frame_length = imgsensor_info.slim_video.pclk / framerate * 10 / imgsensor_info.slim_video.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.slim_video.framelength) ? (frame_length - imgsensor_info.slim_video.framelength): 0;
		imgsensor.frame_length = imgsensor_info.slim_video.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		//set_dummy();
		break;
	default:  //coding with  preview scenario by default
		frame_length = imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.pre.framelength) ? (frame_length - imgsensor_info.pre.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		//set_dummy();
		LOG_INF("error scenario_id = %d, we use preview scenario \n", scenario_id);
		break;
	}
	return ERROR_NONE;
}


static kal_uint32 get_default_framerate_by_scenario(enum MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 *framerate)
{
	LOG_INF("scenario_id = %d\n", scenario_id);

	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		*framerate = imgsensor_info.pre.max_framerate;
		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		*framerate = imgsensor_info.normal_video.max_framerate;
		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		*framerate = imgsensor_info.cap.max_framerate;
		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		*framerate = imgsensor_info.hs_video.max_framerate;
		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		*framerate = imgsensor_info.slim_video.max_framerate;
		break;
	default:
		break;
	}

	return ERROR_NONE;
}


static kal_uint32 set_test_pattern_mode(kal_bool enable)
{
	LOG_INF("enable: %d\n", enable);

	if (enable) {
		write_cmos_sensor(0x0601, 0x02);
	} else {
		write_cmos_sensor(0x0601, 0x00);
	}
	spin_lock(&imgsensor_drv_lock);
	imgsensor.test_pattern = enable;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}

static kal_uint32 streaming_control(kal_bool enable)
{
	LOG_INF("streaming_enable(0=Sw Standby,1=streaming): %d\n", enable);
	if (enable)
		write_cmos_sensor(0x0100, 0X01);
	else
		write_cmos_sensor(0x0100, 0x00);
	mdelay(10);
	return ERROR_NONE;
}
static kal_uint32 feature_control(MSDK_SENSOR_FEATURE_ENUM feature_id,
				  UINT8 *feature_para,UINT32 *feature_para_len)
{
	UINT16 *feature_return_para_16=(UINT16 *) feature_para;
	UINT16 *feature_data_16=(UINT16 *) feature_para;
	UINT32 *feature_return_para_32=(UINT32 *) feature_para;
	UINT32 *feature_data_32=(UINT32 *) feature_para;
	unsigned long long *feature_data=(unsigned long long *) feature_para;
	//    unsigned long long *feature_return_para=(unsigned long long *) feature_para;

	struct SET_PD_BLOCK_INFO_T *PDAFinfo;
	struct SENSOR_WINSIZE_INFO_STRUCT *wininfo;
	struct SENSOR_VC_INFO_STRUCT *pvcinfo;
	MSDK_SENSOR_REG_INFO_STRUCT *sensor_reg_data=(MSDK_SENSOR_REG_INFO_STRUCT *) feature_para;

	if(!((feature_id == 3004) || (feature_id == 3006)))
		LOG_INF("feature_id = %d\n", feature_id);

	switch (feature_id) {
	case SENSOR_FEATURE_GET_PERIOD:
		*feature_return_para_16++ = imgsensor.line_length;
		*feature_return_para_16 = imgsensor.frame_length;
		*feature_para_len=4;
		break;
	case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
		*feature_return_para_32 = imgsensor.pclk;
		*feature_para_len=4;
		break;
	case SENSOR_FEATURE_SET_ESHUTTER:
		set_shutter((UINT16)*feature_data);
		break;
	case SENSOR_FEATURE_SET_NIGHTMODE:
		night_mode((BOOL) *feature_data);
		break;
	case SENSOR_FEATURE_SET_GAIN:
		set_gain((UINT16) *feature_data);
		break;
	case SENSOR_FEATURE_SET_FLASHLIGHT:
		break;
	case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
		break;
	case SENSOR_FEATURE_SET_REGISTER:
		write_cmos_sensor(sensor_reg_data->RegAddr, sensor_reg_data->RegData);
		break;
	case SENSOR_FEATURE_GET_REGISTER:
		sensor_reg_data->RegData = read_cmos_sensor(sensor_reg_data->RegAddr);
		break;
	case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
		// get the lens driver ID from EEPROM or just return LENS_DRIVER_ID_DO_NOT_CARE
		// if EEPROM does not exist in camera module.
		*feature_return_para_32=LENS_DRIVER_ID_DO_NOT_CARE;
		*feature_para_len=4;
		break;
	case SENSOR_FEATURE_SET_VIDEO_MODE:
		set_video_mode(*feature_data);
		break;
	case SENSOR_FEATURE_CHECK_SENSOR_ID:
		get_imgsensor_id(feature_return_para_32);
		break;
	case SENSOR_FEATURE_SET_AUTO_FLICKER_MODE:
		set_auto_flicker_mode((BOOL)*feature_data_16,*(feature_data_16+1));
		break;
	case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
		set_max_framerate_by_scenario((enum MSDK_SCENARIO_ID_ENUM)*feature_data, *(feature_data+1));
		break;
	case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
		get_default_framerate_by_scenario((enum MSDK_SCENARIO_ID_ENUM)*feature_data, (MUINT32 *)(uintptr_t)(*(feature_data+1)));
		break;
//	case SENSOR_FEATURE_GET_PDAF_DATA:
	//	LOG_INF("SENSOR_FEATURE_GET_PDAF_DATA\n");
	//	read_IMX363_eeprom_pdaf((kal_uint16 )(*feature_data),(char*)(uintptr_t)(*(feature_data+1)),(kal_uint32)(*(feature_data+2)));
		//LOG_INF("zhangrui hct-drv read_IMX363_eeprom_pdaf");
	//	break;
	case SENSOR_FEATURE_SET_TEST_PATTERN:
		set_test_pattern_mode((BOOL)*feature_data);
		break;
	case SENSOR_FEATURE_SET_PDAF:
		LOG_INF("PDAF mode :%d\n", *feature_data_16);
		imgsensor.pdaf_mode = 1;//*feature_data_16;
		break;
	case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE: //for factory mode auto testing
		*feature_return_para_32 = imgsensor_info.checksum_value;
		*feature_para_len=4;
		break;
	case SENSOR_FEATURE_SET_FRAMERATE:
		LOG_INF("current fps :%d\n", *feature_data_32);
		spin_lock(&imgsensor_drv_lock);
		imgsensor.current_fps = (UINT16)*feature_data_32;
		spin_unlock(&imgsensor_drv_lock);
		break;
	case SENSOR_FEATURE_SET_HDR:
		/* HDR mODE : 0: disable HDR, 1:IHDR, 2:HDR, 9:ZHDR */
		LOG_INF("ihdr enable :%d\n", *feature_data_32);
		spin_lock(&imgsensor_drv_lock);
		imgsensor.hdr_mode = (UINT8)*feature_data_32;
		LOG_INF("ihdr enable :%d\n", imgsensor.hdr_mode);
		spin_unlock(&imgsensor_drv_lock);
		break;
	case SENSOR_FEATURE_GET_CROP_INFO:
		LOG_INF("SENSOR_FEATURE_GET_CROP_INFO scenarioId:%d\n", (UINT32)*feature_data);
		wininfo = (struct SENSOR_WINSIZE_INFO_STRUCT *)(uintptr_t)(*(feature_data+1));

		switch (*feature_data_32) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[1],sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[2],sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[3],sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[4],sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[0],sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		}
		break;
	
		//add for imx363 pdaf
	case SENSOR_FEATURE_GET_PDAF_INFO:
		LOG_INF("SENSOR_FEATURE_GET_PDAF_INFO scenarioId:%lld\n", *feature_data);
		PDAFinfo= (struct SET_PD_BLOCK_INFO_T *)(uintptr_t)(*(feature_data+1));

		switch (*feature_data) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			memcpy((void *)PDAFinfo,(void *)&imgsensor_pd_info,sizeof(struct SET_PD_BLOCK_INFO_T));
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			break;
		}
		break;

	case SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY:

		LOG_INF("SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY scenarioId:%lld\n", *feature_data);
		//PDAF capacity enable or not, s5k2l7 only full size support PDAF
		switch (*feature_data) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
			break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1; ///preview support
			break;
		default:
			*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
			break;
		}
		break;


	case SENSOR_FEATURE_SET_IHDR_SHUTTER_GAIN:
		LOG_INF("SENSOR_SET_SENSOR_IHDR LE=%d, SE=%d, Gain=%d\n",(UINT16)*feature_data,(UINT16)*(feature_data+1),(UINT16)*(feature_data+2));
		ihdr_write_shutter_gain((UINT16)*feature_data,(UINT16)*(feature_data+1),(UINT16)*(feature_data+2));
		break;
	case SENSOR_FEATURE_GET_VC_INFO:
	LOG_INF("SENSOR_FEATURE_GET_VC_INFO %d\n", (UINT16)*feature_data);
	pvcinfo = (struct SENSOR_VC_INFO_STRUCT *)(uintptr_t)(*(feature_data+1));
		switch (*feature_data_32) {
			case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
				memcpy((void *)pvcinfo,(void *)&SENSOR_VC_INFO[1],sizeof(struct SENSOR_VC_INFO_STRUCT));
				break;
			case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
            default:
	memcpy((void *)pvcinfo,(void *)&SENSOR_VC_INFO[0],sizeof(struct SENSOR_VC_INFO_STRUCT));
				break;
			}
		break;

	case SENSOR_FEATURE_SET_SHUTTER_FRAME_TIME:
		set_shutter_frame_time((UINT16)*feature_data,(UINT16)*(feature_data+1));
		break;
	case SENSOR_FEATURE_SET_STREAMING_SUSPEND:
		LOG_INF("SENSOR_FEATURE_SET_STREAMING_SUSPEND\n");
		streaming_control(KAL_FALSE);
		break;
	case SENSOR_FEATURE_SET_STREAMING_RESUME:
		LOG_INF("SENSOR_FEATURE_SET_STREAMING_RESUME, shutter:%llu\n", *feature_data);
		if (*feature_data != 0)
			set_shutter(*feature_data);
		streaming_control(KAL_TRUE);
		break;
	default:
		break;
	}

	return ERROR_NONE;
}       /*     feature_control()  */

static struct SENSOR_FUNCTION_STRUCT sensor_func = {
	open,
	get_info,
	get_resolution,
	feature_control,
	control,
	close
};

UINT32 IMX363_MIPI_RAW_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc)

{
	/* To Do : Check Sensor status here */
	if (pfFunc!=NULL)
		*pfFunc=&sensor_func;
	return ERROR_NONE;
}       /*     IMX363_MIPI_RAW_SensorInit     */
