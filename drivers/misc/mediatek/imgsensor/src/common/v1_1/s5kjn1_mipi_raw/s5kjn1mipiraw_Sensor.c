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
 *	 s5kjn1mipi_Sensor.c
 *
 * Project:
 * --------
 *	 ALPS
 *
 * Description:
 * ------------
 *	 Source code of Sensor driver
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
#include <linux/atomic.h>
#include <linux/types.h>


#include "kd_camera_typedef.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"
#include "s5kjn1mipiraw_Sensor.h"
#include "s5kjn1_sensor_setting.h"

//add by wq ***********
//#include "imgsensor_common.h"
#include "kd_imgsensor_define.h"

//**********end***********

#undef VENDOR_EDIT
#ifndef VENDOR_EDIT
#define VENDOR_EDIT
#endif
#ifdef VENDOR_EDIT

#ifndef USE_TNP_BURST
#define USE_TNP_BURST
#endif
#endif

/***************Modify Following Strings for Debug**********************/
#define PFX "s5kjn1_camera_sensor"
/****************************   Modify end	**************************/
#define LOG_INF(format, args...) pr_debug(PFX "[%s] " format, __func__, ##args)


#define MULTI_WRITE 1
#if MULTI_WRITE
#define I2C_BUFFER_LEN 1020 /* trans# max is 255, each 3 bytes */
#else
#define I2C_BUFFER_LEN 3
#endif

#ifdef VENDOR_EDIT
#define DEVICE_VERSION_S5KJN1    "s5kjn1"
#endif

#define S5KJN1_MIRROR_FLIP_ENABLE 1
#define S5KJN1_REMOSAIC_ENABLE 1

typedef uint64_t kal_uint64;
static bool bNeedSetNormalMode = KAL_FALSE;

/*prize add by zhuzhengjiang for search camera 2019622 start*/
extern int curr_sensor_id;
/*prize add by zhuzhengjiang for search camera 2019622 end*/

static DEFINE_SPINLOCK(imgsensor_drv_lock);

static struct imgsensor_info_struct imgsensor_info = {
	.sensor_id = S5KJN1_SENSOR_ID,

	.checksum_value =  0x40631970,
	.pre = {
		.pclk = 560000000,
		.linelength = 5910, // 5910
		.framelength = 3156, // 3156
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4080,
		.grabwindow_height = 3072,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 300,
		.mipi_pixel_rate = 662400000,
	},
	.cap = {
		.pclk = 560000000,
		.linelength = 5910, // 5910
		.framelength = 3156, // 3156
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4080,
		.grabwindow_height = 3072,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 300,
		.mipi_pixel_rate = 556800000,
	},
	.normal_video =  {
		.pclk = 560000000,
		.linelength = 5910, // 5910
		.framelength = 3156, // 3156
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4080,
		.grabwindow_height = 3072,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 300,
		.mipi_pixel_rate = 662400000,
	},
	.hs_video =  {
		.pclk = 600000000,
		.linelength = 2096,
		.framelength = 2380,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 1280,
		.grabwindow_height = 720,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 1200,
		.mipi_pixel_rate = 640000000,
	},
	.slim_video =  {
		.pclk = 560000000,
		.linelength = 5910,
		.framelength = 3156,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4080,
		.grabwindow_height = 2296,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 300,
		.mipi_pixel_rate = 480000000,
	},
	.custom1 = {
		.pclk = 560000000,
		.linelength = 5910, // 5910
		.framelength = 3156, // 3156
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4080,
		.grabwindow_height = 3072,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 300,
		.mipi_pixel_rate = 662400000,
	},
	.custom2 = {
		.pclk = 560000000,
		.linelength = 5910, // 5910
		.framelength = 3156, // 3156
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4080,
		.grabwindow_height = 3072,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 300,
		.mipi_pixel_rate = 662400000,
	},
	#if S5KJN1_REMOSAIC_ENABLE
	.custom3 = {
		.pclk = 560000000,
		.linelength = 8688, // 5910
		.framelength = 6400, // 3156
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 8160,
		.grabwindow_height = 6144,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 100,
		.mipi_pixel_rate = 556800000,
	},
	#else
	.custom3 = {
		.pclk = 560000000,
		.linelength = 5910, // 5910
		.framelength = 3156, // 3156
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4080,
		.grabwindow_height = 3072,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 300,
		.mipi_pixel_rate = 662400000,
	},
	#endif
	// prize add by linchong for 5G platform 20220228 start  
	.min_gain = 64,//1x
	.max_gain = 4096,//64x
	.min_gain_iso = 100,
	.exp_step = 2,
	.gain_step = 2,//ov 1, samsung 2, hynix 4, gc 1
	.gain_type = 2,//sony type0, ov type1, samsung type2, hynix type3, gc type4
	.temperature_support=0,
	// prize add by linchong for 5G platform 20220228 end 
	.margin = 5,		/* sensor framelength & shutter margin */
	.min_shutter = 4,	/* min shutter */
	.max_frame_length = 0xffff,
	.ae_shut_delay_frame = 0,
	.ae_sensor_gain_delay_frame = 0,
	.ae_ispGain_delay_frame = 2,	/* isp gain delay frame for AE cycle */
	.ihdr_support = 0,	/* 1, support; 0,not support */
	.ihdr_le_firstline = 0,	/* 1,le first ; 0, se first */
	.temperature_support = 1, /* 1, support; 0,not support */
	.sensor_mode_num = 8,	/* support sensor mode num */

	.cap_delay_frame = 3,
	.pre_delay_frame = 3,
	.video_delay_frame = 3,
	.hs_video_delay_frame = 3,
	.slim_video_delay_frame = 3,                            /*enter slim video delay frame num*/
	.custom1_delay_frame = 3,
	.custom2_delay_frame = 3,
	.custom3_delay_frame = 3,
	.frame_time_delay_frame = 1,

	.isp_driving_current = ISP_DRIVING_6MA,
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
	.mipi_sensor_type = MIPI_OPHY_NCSI2, //0,MIPI_OPHY_NCSI2;  1,MIPI_OPHY_CSI2
	.mipi_settle_delay_mode = 0,
	#if S5KJN1_MIRROR_FLIP_ENABLE
	#if S5KJN1_REMOSAIC_ENABLE
	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_4CELL_BAYER_Gb,
	#else
	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_Gb,
    #endif
	#else
	#if S5KJN1_REMOSAIC_ENABLE
    .sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_4CELL_BAYER_Gr,
	#else
    .sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_Gr,
    #endif
    #endif
	.mclk = 24, /* mclk value, suggest 24 or 26 for 24Mhz or 26Mhz */
	.mipi_lane_num = SENSOR_MIPI_4_LANE,
	.i2c_addr_table = {0x21,0x5a,0xff},
	/* record sensor support all write id addr,
	 * only supprt 4 must end with 0xff
	 */
	.i2c_speed = 400, /* i2c read/write speed */
};

static struct SENSOR_VC_INFO_STRUCT SENSOR_VC_INFO[3]=
{
	/* Preview mode setting */
	 {0x02, //VC_Num
	  0x0a, //VC_PixelNum	
	  0x00, //ModeSelect	/* 0:auto 1:direct */
	  0x00, //EXPO_Ratio	/* 1/1, 1/2, 1/4, 1/8 */
	  0x00, //0DValue		/* 0D Value */
	  0x00, //RG_STATSMODE	/* STATS divistion mode 0:16x16  1:8x8	2:4x4  3:1x1 */
	  0x00, 0x2b, 0x0FF0, 0x0C00,	// VC0 Maybe image data?
	  0x01, 0x00, 0x0000, 0x0000,	// VC1 MVHDR
	  0x01, 0x30, 0x027C, 0x0BF0,   // VC2 PDAF //508*10/8 1528*2 need 4byte align
	  0x00, 0x00, 0x0000, 0x0000},	// VC3 ??
	/* Capture mode setting */ 
	 {0x02, //VC_Num
	  0x0a, //VC_PixelNum	
	  0x00, //ModeSelect	/* 0:auto 1:direct */
	  0x00, //EXPO_Ratio	/* 1/1, 1/2, 1/4, 1/8 */
	  0x00, //0DValue		/* 0D Value */
	  0x00, //RG_STATSMODE	/* STATS divistion mode 0:16x16  1:8x8	2:4x4  3:1x1 */
	  0x00, 0x2b, 0x0FF0, 0x0C00,	// VC0 Maybe image data?
	  0x01, 0x00, 0x0000, 0x0000,	// VC1 MVHDR
	  0x01, 0x30, 0x027C, 0x0BF0,   // VC2 PDAF //508*10/8 1528*2 need 4byte align
	  0x00, 0x00, 0x0000, 0x0000},	// VC3 ??
	/* Video mode setting */
	 {0x02, //VC_Num
	  0x0a, //VC_PixelNum	
	  0x00, //ModeSelect	/* 0:auto 1:direct */
	  0x00, //EXPO_Ratio	/* 1/1, 1/2, 1/4, 1/8 */
	  0x00, //0DValue		/* 0D Value */
	  0x00, //RG_STATSMODE	/* STATS divistion mode 0:16x16  1:8x8	2:4x4  3:1x1 */
	  0x00, 0x2b, 0x0FF0, 0x0C00,	// VC0 Maybe image data?
	  0x01, 0x00, 0x0000, 0x0000,	// VC1 MVHDR
	  0x01, 0x30, 0x027C, 0x0BF0,   // VC2 PDAF //508*10/8 1528*2 need 4byte align
	  0x00, 0x00, 0x0000, 0x0000},	// VC3 ??

};

static struct imgsensor_struct imgsensor = {
	#if S5KJN1_MIRROR_FLIP_ENABLE
	.mirror = IMAGE_HV_MIRROR,    //mirrorflip information
	#else
	.mirror = IMAGE_NORMAL,    //mirrorflip information
	#endif
	.sensor_mode = IMGSENSOR_MODE_INIT, //IMGSENSOR_MODE enum value,record current sensor mode,such as: INIT, Preview, Capture, Video,High Speed Video, Slim Video
	.shutter = 0x3D0,    //current shutter
	.gain = 0x100,    //current gain
	.dummy_pixel = 0,    //current dummypixel
	.dummy_line = 0,    //current dummyline
	.current_fps = 0,  //full size current fps : 24fps for PIP, 30fps for Normal or ZSD
	.autoflicker_en = KAL_FALSE,  //auto flicker enable: KAL_FALSE for disable auto flicker, KAL_TRUE for enable auto flicker
	.test_pattern = KAL_FALSE,    //test pattern mode or not. KAL_FALSE for in test pattern mode, KAL_TRUE for normal output
	.current_scenario_id = MSDK_SCENARIO_ID_CAMERA_PREVIEW,//current scenario id
	.ihdr_mode = 0, //sensor need support LE, SE with HDR feature
	.i2c_write_id = 0x20,
#ifdef VENDOR_EDIT
	.current_ae_effective_frame = 2,
#endif
};

/* Sensor output window information */
static struct SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[] = {
	{ 8160, 6144,    0,    0, 8160, 6144, 4080, 3072,  0,   0, 4080, 3072,    0,    0, 4080, 3072}, // preview
	{ 8160, 6144,    0,    0, 8160, 6144, 4080, 3072,  0,   0, 4080, 3072,    0,    0, 4080, 3072}, // capture
	{ 8160, 6144,    0,    0, 8160, 6144, 4080, 3072,  0,   0, 4080, 3072,    0,    0, 4080, 3072}, // video
	{ 8160, 6144,    0,    0, 8160, 6144, 2040, 1536,380, 408, 1280,  720,    0,    0, 1280,  720}, // high speed
	{ 8160, 6144,    0,    0, 8160, 6144, 4080, 3072,  0, 388, 4080, 2296,    0,    0, 4080, 2296}, // slim video
	{ 8160, 6144,    0,    0, 8160, 6144, 4080, 3072,  0,   0, 4080, 3072,    0,    0, 4080, 3072}, // custom1
	{ 8160, 6144,    0,    0, 8160, 6144, 4080, 3072,  0,   0, 4080, 3072,    0,    0, 4080, 3072}, // custom2
    #if S5KJN1_REMOSAIC_ENABLE
	{ 8160, 6144,    0,    0, 8160, 6144, 8160, 6144,  0,   0, 8160, 6144,    0,    0, 8160, 6144}, // capture
	#else
	{ 8160, 6144,    0,    0, 8160, 6144, 4080, 3072,  0,   0, 4080, 3072,    0,    0, 4080, 3072}, // custom3
    #endif
};

static struct SET_PD_BLOCK_INFO_T imgsensor_pd_info =
{
	.i4OffsetX = 8,
	.i4OffsetY = 8,
	.i4PitchX  = 8,
	.i4PitchY  = 8,
	.i4PairNum  =4,//(pitch_y/density_y)*(pitch_x/desity_x)
	.i4SubBlkW  =8,//density_x
	.i4SubBlkH  =2,//density_y
	.i4PosL = {{11,8},{9,11},{13,12},{15,15}},
	.i4PosR = {{10,8},{8,11},{12,12},{14,15}},
	.i4BlockNumX = 508,//block_x*pitch_x+offset_x*2=raw_size=508*8+8*2=4080
	.i4BlockNumY = 382,
	.iMirrorFlip = 0,
	.i4Crop = { {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0} },
};

#if S5KJN1_REMOSAIC_ENABLE

#define XTC_SIZE (2612)
#define SENSOR_XTC_SIZE (768)
#define PD_XTC_SIZE (4000)
#define SW_GGC_SIZE (626)
#define HW_GGC_SIZE (346)
#define XTC_ADDR (0x0FAE)
#define SENSOR_XTC_ADDR (0x19E4)
#define PD_XTC_ADDR (0x1CE6)
#define SW_GGC_ADDR (0x2C88)
#define HW_GGC_ADDR (0x2EFC)
#define MAX_READ_WRITE_SIZE 255
#define EEPROM_SLAVE_ID (0xA0)
kal_uint8 xtc[XTC_SIZE] = {0};
kal_uint8 sensor_xtc[SENSOR_XTC_SIZE] = {0};
kal_uint8 pd_xtc[PD_XTC_SIZE] = {0};
kal_uint8 sw_ggc[SW_GGC_SIZE] = {0};

static kal_uint8 read_eeprom_data(kal_uint16 addr)
{
	kal_uint8 get_byte = 0;
	char pusendcmd[2] = {(char)(addr >> 8), (char)(addr & 0xFF)};

	iReadRegI2C(pusendcmd, 2, (u8 *)&get_byte, 1, EEPROM_SLAVE_ID);
	return get_byte;
}

static int read_eeprom_data_burst(kal_uint16 addr, kal_uint16 size, kal_uint8 *data)
{
	int i;
	int ret;
	u8 pu_send_cmd[2];
	for (i = 0; i < size; i += MAX_READ_WRITE_SIZE) {
		pu_send_cmd[0] = (u8) (addr >> 8);
		pu_send_cmd[1] = (u8) (addr & 0xFF);
		if (i + MAX_READ_WRITE_SIZE > size) {
			ret = iReadRegI2C(pu_send_cmd, 2, (u8*)(data+i), (size-i), EEPROM_SLAVE_ID);
		} else {
			ret = iReadRegI2C(pu_send_cmd, 2, (u8*)(data+i), MAX_READ_WRITE_SIZE, EEPROM_SLAVE_ID);
		}
		if (ret < 0)
			return ret;
		addr += MAX_READ_WRITE_SIZE;
	}
	return ret;
}

static int check_remosaic_data(kal_int16 addr, kal_int16 size, kal_uint8 *data)
{
	long long checksum = 0;
	int i;
	if(read_eeprom_data(addr)!=0x01){
		LOG_INF("eeprom data is invalid!\n");
		return -1;
	}
	if(read_eeprom_data_burst((addr+1), size, data)) {
		LOG_INF("read eeprom data failed!\n");
		return -1;
	}
	for(i=0; i<size; i++){
		checksum+=data[i];
	}
	if((checksum%256)!=read_eeprom_data(addr+size+1)){
		LOG_INF("eeprom data checksum failed!\n");
		return -1;
	}
	LOG_INF("first 4byte data: 0x%x 0x%x 0x%x 0x%x\n", data[0], data[1], data[2], data[3]);
	return 0;
}

static int read_remosaic_data(void)
{
	int ret = 0;
	ret |= check_remosaic_data(XTC_ADDR, XTC_SIZE, &xtc[0]);
	ret |= check_remosaic_data(SENSOR_XTC_ADDR, SENSOR_XTC_SIZE, &sensor_xtc[0]);
	ret |= check_remosaic_data(PD_XTC_ADDR, PD_XTC_SIZE, &pd_xtc[0]);
	ret |= check_remosaic_data(SW_GGC_ADDR, SW_GGC_SIZE, &sw_ggc[0]);
	
	return ret;
}

#endif

static kal_uint16 read_cmos_sensor(kal_uint32 addr)
{
	kal_uint16 get_byte = 0;
	char pusendcmd[2] = {(char)(addr >> 8), (char)(addr & 0xFF)};

	iReadRegI2C(pusendcmd, 2, (u8 *)&get_byte, 2, imgsensor.i2c_write_id);
	return ((get_byte<<8)&0xff00) | ((get_byte>>8)&0x00ff);
}

static void write_cmos_sensor(kal_uint16 addr, kal_uint16 para)
{
	char pusendcmd[4] = {(char)(addr >> 8) , (char)(addr & 0xFF) ,(char)(para >> 8),(char)(para & 0xFF)};
	iWriteRegI2C(pusendcmd , 4, imgsensor.i2c_write_id);
}

static kal_uint16 read_cmos_sensor_8(kal_uint16 addr)
{
	kal_uint16 get_byte = 0;
	char pusendcmd[2] = {(char)(addr >> 8), (char)(addr & 0xFF) };

	iReadRegI2C(pusendcmd, 2, (u8 *)&get_byte, 1, imgsensor.i2c_write_id);
	return get_byte;
}

static void write_cmos_sensor_8(kal_uint16 addr, kal_uint8 para)
{
	char pusendcmd[3] = {(char)(addr >> 8), (char)(addr & 0xFF),
			(char)(para & 0xFF)};

	iWriteRegI2C(pusendcmd, 3, imgsensor.i2c_write_id);
}

#if MULTI_WRITE
static kal_uint16 table_write_cmos_sensor(kal_uint16 *para, kal_uint32 len)
{
	char puSendCmd[I2C_BUFFER_LEN];
	kal_uint32 tosend, IDX;
	kal_uint16 addr = 0, addr_last = 0, data;
	tosend = 0;
	IDX = 0;

	while (len > IDX) {
		addr = para[IDX];

		{
			puSendCmd[tosend++] = (char)(addr >> 8);
			puSendCmd[tosend++] = (char)(addr & 0xFF);
			data = para[IDX + 1];
			puSendCmd[tosend++] = (char)(data >> 8);
			puSendCmd[tosend++] = (char)(data & 0xFF);
			IDX += 2;
			addr_last = addr;

		}
#if MULTI_WRITE
		/* Write when remain buffer size is less than 3 bytes
		 * or reach end of data
		 */
		if ((I2C_BUFFER_LEN - tosend) < 4
			|| IDX == len || addr != addr_last) {
			iBurstWriteReg_multi(puSendCmd,
						tosend,
						imgsensor.i2c_write_id,
						4,
						imgsensor_info.i2c_speed);
			tosend = 0;
		}
#else
		iWriteRegI2C(puSendCmd, 4, imgsensor.i2c_write_id);
		tosend = 0;
#endif
	}

#if 0 /*for debug*/
	for (int i = 0; i < len/2; i++)
		LOG_INF("readback addr(0x%x)=0x%x\n",
			para[2*i], read_cmos_sensor_8(para[2*i]));
#endif
	return 0;
}

#endif //add by wq


static void set_dummy(void)
{
	LOG_INF("dummyline = %d, dummypixels = %d \n", imgsensor.dummy_line, imgsensor.dummy_pixel);
	//return; //for test
	write_cmos_sensor(0x0340, imgsensor.frame_length);
	write_cmos_sensor(0x0342, imgsensor.line_length);

}	/*	set_dummy  */

static void set_mirror_flip(kal_uint8 image_mirror)
{
	kal_uint8 itemp;
	LOG_INF("image_mirror = %d\n", image_mirror);
	itemp=read_cmos_sensor(0x0101);
	LOG_INF("image_mirror itemp = %d\n", itemp);
	itemp &= ~0x03;

	switch(image_mirror)
		{

			case IMAGE_NORMAL:
				write_cmos_sensor_8(0x0101, itemp);
				break;

			case IMAGE_V_MIRROR:
				write_cmos_sensor_8(0x0101, itemp | 0x02);
				break;

			case IMAGE_H_MIRROR:
				write_cmos_sensor_8(0x0101, itemp | 0x01);
				break;

			case IMAGE_HV_MIRROR:
				write_cmos_sensor_8(0x0101, itemp | 0x03);
				break;
		}
}

static void set_max_framerate(UINT16 framerate, kal_bool min_framelength_en)
{
	/*kal_int16 dummy_line;*/
	kal_uint32 frame_length = imgsensor.frame_length;

	LOG_INF("framerate = %d, min framelength should enable %d\n", framerate,
		min_framelength_en);

	frame_length = imgsensor.pclk / framerate * 10 / imgsensor.line_length;
	spin_lock(&imgsensor_drv_lock);
	if (frame_length >= imgsensor.min_frame_length)
		imgsensor.frame_length = frame_length;
	else
		imgsensor.frame_length = imgsensor.min_frame_length;

	imgsensor.dummy_line =
			imgsensor.frame_length - imgsensor.min_frame_length;

	if (imgsensor.frame_length > imgsensor_info.max_frame_length) {
		imgsensor.frame_length = imgsensor_info.max_frame_length;
		imgsensor.dummy_line =
			imgsensor.frame_length - imgsensor.min_frame_length;
	}
	if (min_framelength_en)
		imgsensor.min_frame_length = imgsensor.frame_length;
	spin_unlock(&imgsensor_drv_lock);
	set_dummy();
}	/*	set_max_framerate  */

static kal_uint32 streaming_control(kal_bool enable)
{
	int timeout = 100;
	int i = 0;
	int framecnt = 0;

	LOG_INF("streaming_enable(0=Sw Standby,1=streaming): %d\n", enable);
	if (enable) {
		write_cmos_sensor(0x6028, 0x4000);
		write_cmos_sensor_8(0x0100, 0X01);
		mdelay(10);
	} else {
		write_cmos_sensor(0x6028, 0x4000);
		write_cmos_sensor_8(0x0100, 0x00);
		for (i = 0; i < timeout; i++) {
			mdelay(10);
			framecnt = read_cmos_sensor_8(0x0005);
			if (framecnt == 0xFF) {
				LOG_INF(" Stream Off OK at i=%d.\n", i);
				return ERROR_NONE;
			}
		}
		LOG_INF("Stream Off Fail! framecnt=%d.\n", framecnt);
	}
	return ERROR_NONE;
}


static void write_shutter(kal_uint32 shutter)
{
	kal_uint16 realtime_fps = 0;
	#ifdef VENDOR_EDIT
	kal_uint64 CintR = 0;
	kal_uint64 Time_Farme = 0;
	#endif
	spin_lock(&imgsensor_drv_lock);
	if (shutter > imgsensor.min_frame_length - imgsensor_info.margin) {
		imgsensor.frame_length = shutter + imgsensor_info.margin;
	} else {
		imgsensor.frame_length = imgsensor.min_frame_length;
	}
	if (imgsensor.frame_length > imgsensor_info.max_frame_length) {
		imgsensor.frame_length = imgsensor_info.max_frame_length;
	}
	spin_unlock(&imgsensor_drv_lock);
	if (shutter < imgsensor_info.min_shutter) {
		shutter = imgsensor_info.min_shutter;
	}

	if (imgsensor.autoflicker_en) {
		realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 / imgsensor.frame_length;
		if (realtime_fps >= 297 && realtime_fps <= 305) {
			set_max_framerate(296,0);
		} else if (realtime_fps >= 147 && realtime_fps <= 150) {
			set_max_framerate(146,0);
		} else {
			// Extend frame length
			write_cmos_sensor(0x0340, imgsensor.frame_length);
		}
	} else {
		// Extend frame length
		write_cmos_sensor(0x0340, imgsensor.frame_length);
	}
	// Update Shutter
	#ifdef VENDOR_EDIT
	LOG_INF("2 shutter = %d\n", shutter);
	// 1H=4584/560000000 =8.1857 us
	// 16s=16000000/8.1857 =1954628H
	//1s=1000000/8.1857=122164H
	if (shutter >= 0xFFF0) {  // need to modify line_length & PCLK
		bNeedSetNormalMode = KAL_TRUE;

		if (shutter >= 1954628) {  //>16s
			shutter = 1954628;
		}
		// 1s > 0x05DB (1499)
		// 2s > 0x0BB6
		// 3s > 0x1191
		// 4s > 0x176C
		// 5s > 0x1D47
		// 6s > 0x2322
		// 7s > 0x28FD
		// 8s > 0x2ED8
		// 9s > 0x34B3
		//10s > 0x3A8E  959324(14956)
		//11s > 0x4069
		//12s > 0x4644
		//13s > 0x4C1F  1247122(19443)
		//14s > 0x51FA
		//15s > 0x57D5
		//16s > 0xBB61  1534919(23930)
		//0x0E14 value = shutter * 482000000 / (5024 * 2^1536)
		//0x0E14 value = shutter * 1499;
		//CintR = (482000000*shutter*0.0000104)/(5024*64);
		CintR = (5013 * (unsigned long long)shutter) / 321536;
		Time_Farme = CintR + 0x0002;  // 1st framelength
		LOG_INF("CintR =%d \n", CintR);
		//write_cmos_sensor(0x6028, 0x4000);
		//write_cmos_sensor(0x0100, 0x0000);
		//streaming_control(KAL_FALSE); // check stream off
		write_cmos_sensor(0x0340, Time_Farme & 0xFFFF);  // Framelength
		write_cmos_sensor(0x0202, CintR & 0xFFFF);  //shutter
		write_cmos_sensor(0x0702, 0x0600);
		write_cmos_sensor(0x0704, 0x0600);
		//write_cmos_sensor(0x0100, 0x0100);
		//streaming_control(KAL_TRUE);

		/*Chengtian.Ding@Camera, 2018-12-28 add for n+1 long exposure*/
		/* Frame exposure mode customization for LE*/
		imgsensor.ae_frm_mode.frame_mode_1 = IMGSENSOR_AE_MODE_SE;
		imgsensor.ae_frm_mode.frame_mode_2 = IMGSENSOR_AE_MODE_SE;
		imgsensor.current_ae_effective_frame = 2;
	} else {
		if (bNeedSetNormalMode) {
			LOG_INF("exit long shutter\n");
			//write_cmos_sensor(0x6028, 0x4000);
			//write_cmos_sensor(0x0100, 0x0000);
			//remove stream off to fix long shutter issue
			//streaming_control(KAL_FALSE); // check stream off
			write_cmos_sensor(0x0702, 0x0000);
			write_cmos_sensor(0x0704, 0x0000);
			//write_cmos_sensor(0x0100, 0x0100);
			//streaming_control(KAL_TRUE);
			bNeedSetNormalMode = KAL_FALSE;
		}

		write_cmos_sensor(0x0340, imgsensor.frame_length);
		write_cmos_sensor(0x0202, imgsensor.
		shutter);


		imgsensor.current_ae_effective_frame = 2;
	}
	#endif
	LOG_INF("shutter =%d, framelength =%d \n", shutter,imgsensor.frame_length);



}	/*	write_shutter  */

/*************************************************************************
 * FUNCTION
 *	set_shutter
 *
 * DESCRIPTION
 *	This function set e-shutter of sensor to change exposure time.
 *
 * PARAMETERS
 *	iShutter : exposured lines
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static void set_shutter(kal_uint32 shutter)
{
	unsigned long flags;

	spin_lock_irqsave(&imgsensor_drv_lock, flags);
	imgsensor.shutter = shutter;
	spin_unlock_irqrestore(&imgsensor_drv_lock, flags);

	write_shutter(shutter);
} /* set_shutter */


/*************************************************************************
 * FUNCTION
 *	set_shutter_frame_length
 *
 * DESCRIPTION
 *	for frame & 3A sync
 *
 *************************************************************************/
static void set_shutter_frame_length(kal_uint16 shutter,
				     kal_uint16 frame_length)
{
	unsigned long flags;
	kal_uint16 realtime_fps = 0;
	kal_int32 dummy_line = 0;

	spin_lock_irqsave(&imgsensor_drv_lock, flags);
	imgsensor.shutter = shutter;
	spin_unlock_irqrestore(&imgsensor_drv_lock, flags);

	
	spin_lock(&imgsensor_drv_lock);
	/* Change frame time */
	dummy_line = frame_length - imgsensor.frame_length;
	imgsensor.frame_length = imgsensor.frame_length + dummy_line;
	imgsensor.min_frame_length = imgsensor.frame_length;

	if (shutter > imgsensor.frame_length - imgsensor_info.margin) {
		imgsensor.frame_length = shutter + imgsensor_info.margin;
	}
	if (imgsensor.frame_length > imgsensor_info.max_frame_length) {
		imgsensor.frame_length = imgsensor_info.max_frame_length;
	}
	spin_unlock(&imgsensor_drv_lock);
	shutter = (shutter < imgsensor_info.min_shutter) ? imgsensor_info.min_shutter : shutter;
	shutter = (shutter > (imgsensor_info.max_frame_length - imgsensor_info.margin))
		? (imgsensor_info.max_frame_length - imgsensor_info.margin) : shutter;


	if (imgsensor.autoflicker_en) {
		realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 / imgsensor.frame_length;
		if (realtime_fps >= 297 && realtime_fps <= 305) {
			set_max_framerate(296, 0);
		} else if (realtime_fps >= 147 && realtime_fps <= 150) {
			set_max_framerate(146, 0);
		} else {
			/* Extend frame length */
			write_cmos_sensor(0x0340, imgsensor.frame_length);
		}
	} else {
		/* Extend frame length */
		write_cmos_sensor(0x0340, imgsensor.frame_length);
	}

	/* Update Shutter */
	write_cmos_sensor(0x0202, imgsensor.shutter);
	LOG_INF("Exit! shutter =%d, framelength =%d/%d, dummy_line=%d, auto_extend=%d\n",
		shutter, imgsensor.frame_length, frame_length, dummy_line, read_cmos_sensor(0x0350));


}	/* set_shutter_frame_length */

static kal_uint16 gain2reg(const kal_uint16 gain)
{
	kal_uint16 reg_gain = 0x0;

    reg_gain = gain/2;
    return (kal_uint16)reg_gain;
}

/*************************************************************************
 * FUNCTION
 *	set_gain
 *
 * DESCRIPTION
 *	This function is to set global gain to sensor.
 *
 * PARAMETERS
 *	iGain : sensor global gain(base: 0x40)
 *
 * RETURNS
 *	the actually gain set to sensor.
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint16 set_gain(kal_uint16 gain)
{
	kal_uint16 reg_gain;

	if (gain < BASEGAIN || gain > 64 * BASEGAIN) {
		LOG_INF("Error gain setting");
		if (gain < BASEGAIN) {
			gain = BASEGAIN;
		} else if (gain > 64 * BASEGAIN) {
			gain = 64 * BASEGAIN;
		}
	}

	reg_gain = gain2reg(gain);
	spin_lock(&imgsensor_drv_lock);
	imgsensor.gain = reg_gain;
	spin_unlock(&imgsensor_drv_lock);
	LOG_INF("gain = %d , reg_gain = 0x%x\n ", gain, reg_gain);

	//write_cmos_sensor(0x0204,reg_gain);
	write_cmos_sensor_8(0x0204, (reg_gain >> 8));
	write_cmos_sensor_8(0x0205, (reg_gain & 0xff));

    return gain;
} /* set_gain */


static void sensor_init(void)
{
	LOG_INF("E\n");
	
	write_cmos_sensor(0x6028,0x4000);
	write_cmos_sensor(0x0000,0x0001);
	write_cmos_sensor(0x0000,0x38E1);
	write_cmos_sensor(0x001E,0x0005);
	write_cmos_sensor(0x6028,0x4000);
	write_cmos_sensor(0x6010,0x0001);
    mdelay(5);
	write_cmos_sensor(0x6226, 0x0001);
	mdelay(10);
	#ifdef USE_TNP_BURST
	table_write_cmos_sensor(addr_data_pair_init,
		sizeof(addr_data_pair_init) / sizeof(kal_uint16));
	#else
	
	#endif
}	/*	  sensor_init  */


static void preview_setting(void)
{
	//Preview 4080*3072 30fps 24M MCLK 4lane 1656Mbps/lane
	// preview 30.01fps
	
	LOG_INF("preview_setting\n");
	#ifdef USE_TNP_BURST
	table_write_cmos_sensor(addr_data_pair_preview,
		sizeof(addr_data_pair_preview) / sizeof(kal_uint16));
	#else

	#endif
} /* preview_setting */


static void capture_setting(kal_uint16 currefps)
{
	LOG_INF("capture_setting enter\n");
	#ifdef USE_TNP_BURST
	
	table_write_cmos_sensor(addr_data_pair_capture,
		sizeof(addr_data_pair_capture) / sizeof(kal_uint16));
	#else

	#endif
}/* capture_setting */


static void normal_video_setting(kal_uint16 currefps)
{
	LOG_INF("normal_video_setting!\n");
	// full size 30fps
	#ifdef USE_TNP_BURST
	table_write_cmos_sensor(addr_data_pair_normal_video,
		sizeof(addr_data_pair_normal_video) / sizeof(kal_uint16));
	#else

	#endif
	
}/* normal_video_setting */

static void hs_video_setting(void)
{
	LOG_INF("hs_video_setting E\n");
	//720p 120fps
	#ifdef USE_TNP_BURST
	table_write_cmos_sensor(addr_data_pair_hs_video,
		sizeof(addr_data_pair_hs_video) / sizeof(kal_uint16));
	#else

	#endif
}
static void slim_video_setting(void)
{
	LOG_INF("%s E\n", __func__);
}
static void custom1_setting(void)
{
	LOG_INF("%s E\n", __func__);
}
static void custom2_setting(void)
{
	LOG_INF("%s E\n", __func__);
}
static void custom3_setting(void)
{
	LOG_INF("%s E\n", __func__);
	#if S5KJN1_REMOSAIC_ENABLE
	table_write_cmos_sensor(addr_data_pair_capture_remosaic,
		sizeof(addr_data_pair_capture_remosaic) / sizeof(kal_uint16));
	#else
	table_write_cmos_sensor(addr_data_pair_capture,
		sizeof(addr_data_pair_capture) / sizeof(kal_uint16));
    #endif		
}

/*************************************************************************
 * FUNCTION
 *	get_imgsensor_id
 *
 * DESCRIPTION
 *	This function get the sensor ID
 *
 * PARAMETERS
 *	*sensorID : return the sensor ID
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 get_imgsensor_id(UINT32 *sensor_id)
{
	kal_uint8 i = 0;
	kal_uint8 retry = 2;
	//sensor have two i2c address 0x6c 0x6d & 0x21 0x20, we should detect the module used i2c address
	/*prize add by zhuzhengjiang for search camera 2019622 start*/
	if(curr_sensor_id != 0)
	    return ERROR_SENSOR_CONNECT_FAIL;
	/*prize add by zhuzhengjiang for search camera 2019622 end*/
	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
			*sensor_id = ((read_cmos_sensor_8(0x0000) << 8) | read_cmos_sensor_8(0x0001));
			LOG_INF("read_0x0000=0x%x, 0x0001=0x%x,0x0000_0001=0x%x\n",read_cmos_sensor_8(0x0000),read_cmos_sensor_8(0x0001),read_cmos_sensor(0x0000));
			if (*sensor_id == imgsensor_info.sensor_id) {
				#if S5KJN1_REMOSAIC_ENABLE
				if (read_remosaic_data())
					LOG_INF("init remosaic data failed!\n");
				
			//	LOG_INF("prize linchong xtc 4bytes: %02x %02x %02x %02x\n", xtc[0], xtc[1], xtc[2], xtc[3]);
			//	LOG_INF("prize linchong sensor_xtc 4bytes: %02x %02x %02x %02x\n", sensor_xtc[0], sensor_xtc[1], sensor_xtc[2], sensor_xtc[3]);
			//	LOG_INF("prize linchong pd_xtc 4bytes: %02x %02x %02x %02x\n", pd_xtc[0], pd_xtc[1], pd_xtc[2], pd_xtc[3]);
			//	LOG_INF("prize linchong sw_ggc 4bytes: %02x %02x %02x %02x\n", sw_ggc[0], sw_ggc[1], sw_ggc[2], sw_ggc[3]);
				#endif
				LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id,*sensor_id);
				return ERROR_NONE;
			}
			LOG_INF("Read sensor id fail, id: 0x%x ensor id: 0x%x\n", imgsensor.i2c_write_id,*sensor_id);
			retry--;
		} while (retry > 0);
		i++;
		retry = 2;
	}
	if (*sensor_id != imgsensor_info.sensor_id) {
		/*if Sensor ID is not correct,
		 *Must set *sensor_id to 0xFFFFFFFF
		 */
		*sensor_id = 0xFFFFFFFF;
		return ERROR_SENSOR_CONNECT_FAIL;
	}
	return ERROR_NONE;
}


/*************************************************************************
 * FUNCTION
 *	open
 *
 * DESCRIPTION
 *	This function initialize the registers of CMOS sensor
 *
 * PARAMETERS
 *	None
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 open(void)
{
	kal_uint8 i = 0;
	kal_uint8 retry = 2;
	kal_uint16 sensor_id = 0;
	/*prize add by zhuzhengjiang for search camera 2019622 start*/
	if(curr_sensor_id != 0)
	    return ERROR_SENSOR_CONNECT_FAIL;
	/*prize add by zhuzhengjiang for search camera 2019622 end*/
	/*sensor have two i2c address 0x6c 0x6d & 0x21 0x20,
	 *we should detect the module used i2c address
	 */
	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
			sensor_id = ((read_cmos_sensor_8(0x0000) << 8) | read_cmos_sensor_8(0x0001));
			if (sensor_id == imgsensor_info.sensor_id) {
				LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id,sensor_id);
				break;
			}
			LOG_INF("Read sensor id fail, id: 0x%x\n", imgsensor.i2c_write_id);
			retry--;
		} while (retry > 0);
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

	imgsensor.autoflicker_en = KAL_FALSE;
	imgsensor.sensor_mode = IMGSENSOR_MODE_INIT;
	imgsensor.shutter = 0x3D0;
	imgsensor.gain = 0x100;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	imgsensor.frame_length = imgsensor_info.pre.framelength;
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.dummy_pixel = 0;
	imgsensor.dummy_line = 0;
	imgsensor.ihdr_mode = 0;
	imgsensor.test_pattern = KAL_FALSE;
	imgsensor.current_fps = imgsensor_info.pre.max_framerate;
	spin_unlock(&imgsensor_drv_lock);

	return ERROR_NONE;
} /* open */

/*************************************************************************
 * FUNCTION
 *	close
 *
 * DESCRIPTION
 *
 *
 * PARAMETERS
 *	None
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 close(void)
{
	LOG_INF("E\n");

	/*No Need to implement this function*/
	streaming_control(KAL_FALSE);
	return ERROR_NONE;
} /* close */


/*************************************************************************
 * FUNCTION
 * preview
 *
 * DESCRIPTION
 *	This function start the sensor preview.
 *
 * PARAMETERS
 *	*image_window : address pointer of pixel numbers in one period of HSYNC
 *  *sensor_config_data : address pointer of line numbers in one period of VSYNC
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 preview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("%s E\n", __func__);

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_PREVIEW;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.frame_length = imgsensor_info.pre.framelength;
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);

	preview_setting();
	set_mirror_flip(imgsensor.mirror);

	return ERROR_NONE;
} /* preview */

/*************************************************************************
 * FUNCTION
 *	capture
 *
 * DESCRIPTION
 *	This function setup the CMOS sensor in capture MY_OUTPUT mode
 *
 * PARAMETERS
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 capture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CAPTURE;
#if 0
	if (imgsensor.current_fps == imgsensor_info.cap1.max_framerate) {
	/* PIP capture: 24fps for less than 13M, 20fps for 16M,15fps for 20M */
		imgsensor.pclk = imgsensor_info.cap1.pclk;
		imgsensor.line_length = imgsensor_info.cap1.linelength;
		imgsensor.frame_length = imgsensor_info.cap1.framelength;
		imgsensor.min_frame_length = imgsensor_info.cap1.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	} else if (imgsensor.current_fps == imgsensor_info.cap2.max_framerate) {
		if (imgsensor.current_fps != imgsensor_info.cap.max_framerate)
			LOG_INF(
			"Warning: current_fps %d fps is not support, so use cap1's setting: %d fps!\n",
		imgsensor.current_fps, imgsensor_info.cap1.max_framerate/10);
		imgsensor.pclk = imgsensor_info.cap2.pclk;
		imgsensor.line_length = imgsensor_info.cap2.linelength;
		imgsensor.frame_length = imgsensor_info.cap2.framelength;
		imgsensor.min_frame_length = imgsensor_info.cap2.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	} else {
		if (imgsensor.current_fps != imgsensor_info.cap.max_framerate)
			LOG_INF(
			"Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n",
			imgsensor.current_fps,
			imgsensor_info.cap.max_framerate / 10);
		imgsensor.pclk = imgsensor_info.cap.pclk;
		imgsensor.line_length = imgsensor_info.cap.linelength;
		imgsensor.frame_length = imgsensor_info.cap.framelength;
		imgsensor.min_frame_length = imgsensor_info.cap.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	}
#else
	if (imgsensor.current_fps != imgsensor_info.cap.max_framerate)
		LOG_INF(
			"Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n",
			imgsensor.current_fps,
			imgsensor_info.cap.max_framerate / 10);
	imgsensor.pclk = imgsensor_info.cap.pclk;
	imgsensor.line_length = imgsensor_info.cap.linelength;
	imgsensor.frame_length = imgsensor_info.cap.framelength;
	imgsensor.min_frame_length = imgsensor_info.cap.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
#endif

	spin_unlock(&imgsensor_drv_lock);
	capture_setting(imgsensor.current_fps);
	set_mirror_flip(imgsensor.mirror);

	return ERROR_NONE;
}	/* capture() */
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
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	normal_video_setting(imgsensor.current_fps);
	set_mirror_flip(imgsensor.mirror);

	return ERROR_NONE;
}	/*	normal_video   */

static kal_uint32 hs_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
				MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_HIGH_SPEED_VIDEO;
	imgsensor.pclk = imgsensor_info.hs_video.pclk;
	/*imgsensor.video_mode = KAL_TRUE;*/
	imgsensor.line_length = imgsensor_info.hs_video.linelength;
	imgsensor.frame_length = imgsensor_info.hs_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.hs_video.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	/*imgsensor.current_fps = 300;*/
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	hs_video_setting();
	set_mirror_flip(imgsensor.mirror);

	return ERROR_NONE;
}	/*	hs_video   */

static kal_uint32 slim_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
				MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_SLIM_VIDEO;
	imgsensor.pclk = imgsensor_info.slim_video.pclk;
	/*imgsensor.video_mode = KAL_TRUE;*/
	imgsensor.line_length = imgsensor_info.slim_video.linelength;
	imgsensor.frame_length = imgsensor_info.slim_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.slim_video.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	/*imgsensor.current_fps = 300;*/
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	slim_video_setting();
	set_mirror_flip(imgsensor.mirror);

	return ERROR_NONE;
}	/* slim_video */
static kal_uint32 custom1(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
				MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM1;
	imgsensor.pclk = imgsensor_info.custom1.pclk;
	/*imgsensor.video_mode = KAL_TRUE;*/
	imgsensor.line_length = imgsensor_info.custom1.linelength;
	imgsensor.frame_length = imgsensor_info.custom1.framelength;
	imgsensor.min_frame_length = imgsensor_info.custom1.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	/*imgsensor.current_fps = 300;*/
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	custom1_setting();
	set_mirror_flip(imgsensor.mirror);

	return ERROR_NONE;
}	/* custom1 */
static kal_uint32 custom2(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
				MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM2;
	imgsensor.pclk = imgsensor_info.custom2.pclk;
	/*imgsensor.video_mode = KAL_TRUE;*/
	imgsensor.line_length = imgsensor_info.custom2.linelength;
	imgsensor.frame_length = imgsensor_info.custom2.framelength;
	imgsensor.min_frame_length = imgsensor_info.custom2.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	/*imgsensor.current_fps = 300;*/
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	custom2_setting();
	set_mirror_flip(imgsensor.mirror);

	return ERROR_NONE;
}	/* custom2 */
static kal_uint32 custom3(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
				MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM3;
	imgsensor.pclk = imgsensor_info.custom3.pclk;
	/*imgsensor.video_mode = KAL_TRUE;*/
	imgsensor.line_length = imgsensor_info.custom3.linelength;
	imgsensor.frame_length = imgsensor_info.custom3.framelength;
	imgsensor.min_frame_length = imgsensor_info.custom3.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	/*imgsensor.current_fps = 300;*/
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	custom3_setting();
	set_mirror_flip(imgsensor.mirror);

	return ERROR_NONE;
}	/* custom3 */
static kal_uint32
get_resolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *sensor_resolution)
{
	LOG_INF("E\n");
	sensor_resolution->SensorFullWidth =
		imgsensor_info.cap.grabwindow_width;
	sensor_resolution->SensorFullHeight =
		imgsensor_info.cap.grabwindow_height;

	sensor_resolution->SensorPreviewWidth =
		imgsensor_info.pre.grabwindow_width;
	sensor_resolution->SensorPreviewHeight =
		imgsensor_info.pre.grabwindow_height;

	sensor_resolution->SensorVideoWidth =
		imgsensor_info.normal_video.grabwindow_width;
	sensor_resolution->SensorVideoHeight =
		imgsensor_info.normal_video.grabwindow_height;

	sensor_resolution->SensorHighSpeedVideoWidth =
		imgsensor_info.hs_video.grabwindow_width;
	sensor_resolution->SensorHighSpeedVideoHeight =
		imgsensor_info.hs_video.grabwindow_height;

	sensor_resolution->SensorSlimVideoWidth =
		imgsensor_info.slim_video.grabwindow_width;
	sensor_resolution->SensorSlimVideoHeight =
		imgsensor_info.slim_video.grabwindow_height;
		
	sensor_resolution->SensorCustom1Width =
		imgsensor_info.custom1.grabwindow_width;
	sensor_resolution->SensorCustom1Height =
		imgsensor_info.custom1.grabwindow_height;
	
	sensor_resolution->SensorCustom2Width =
		imgsensor_info.custom2.grabwindow_width;
	sensor_resolution->SensorCustom2Height =
		imgsensor_info.custom2.grabwindow_height;
		
	sensor_resolution->SensorCustom3Width =
		imgsensor_info.custom3.grabwindow_width;
	sensor_resolution->SensorCustom3Height =
		imgsensor_info.custom3.grabwindow_height;

	return ERROR_NONE;
} /* get_resolution */

static kal_uint32 get_info(enum MSDK_SCENARIO_ID_ENUM scenario_id,
			   MSDK_SENSOR_INFO_STRUCT *sensor_info,
			   MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{

	sensor_info->SensorClockPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorClockFallingPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorInterruptDelayLines = 4; /* not use */
	sensor_info->SensorResetActiveHigh = FALSE; /* not use */
	sensor_info->SensorResetDelayCount = 5; /* not use */

	sensor_info->SensroInterfaceType = imgsensor_info.sensor_interface_type;
	sensor_info->MIPIsensorType = imgsensor_info.mipi_sensor_type;
	sensor_info->SettleDelayMode = imgsensor_info.mipi_settle_delay_mode;
	sensor_info->SensorOutputDataFormat =
		imgsensor_info.sensor_output_dataformat;

	sensor_info->CaptureDelayFrame = imgsensor_info.cap_delay_frame;
	sensor_info->PreviewDelayFrame = imgsensor_info.pre_delay_frame;
	sensor_info->VideoDelayFrame = imgsensor_info.video_delay_frame;
	sensor_info->HighSpeedVideoDelayFrame =
		imgsensor_info.hs_video_delay_frame;
	sensor_info->SlimVideoDelayFrame =
		imgsensor_info.slim_video_delay_frame;
	sensor_info->Custom1DelayFrame =
		imgsensor_info.custom1_delay_frame;
	sensor_info->Custom2DelayFrame =
		imgsensor_info.custom2_delay_frame;
	sensor_info->Custom3DelayFrame =
		imgsensor_info.custom3_delay_frame;

	sensor_info->SensorMasterClockSwitch = 0; /* not use */
	sensor_info->SensorDrivingCurrent = imgsensor_info.isp_driving_current;

	sensor_info->AEShutDelayFrame = imgsensor_info.ae_shut_delay_frame;
	sensor_info->AESensorGainDelayFrame =
		imgsensor_info.ae_sensor_gain_delay_frame;
	sensor_info->AEISPGainDelayFrame =
		imgsensor_info.ae_ispGain_delay_frame;
	sensor_info->IHDR_Support = imgsensor_info.ihdr_support;
	sensor_info->IHDR_LE_FirstLine = imgsensor_info.ihdr_le_firstline;
	sensor_info->TEMPERATURE_SUPPORT = imgsensor_info.temperature_support;
	sensor_info->SensorModeNum = imgsensor_info.sensor_mode_num;
	sensor_info->PDAF_Support = 2;
	sensor_info->SensorMIPILaneNumber = imgsensor_info.mipi_lane_num;
	sensor_info->SensorClockFreq = imgsensor_info.mclk;
	sensor_info->SensorClockDividCount = 3; /* not use */
	sensor_info->SensorClockRisingCount = 0;
	sensor_info->SensorClockFallingCount = 2; /* not use */
	sensor_info->SensorPixelClockCount = 3; /* not use */
	sensor_info->SensorDataLatchCount = 2; /* not use */

	sensor_info->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
	sensor_info->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
	sensor_info->SensorWidthSampling = 0; /* 0 is default 1x */
	sensor_info->SensorHightSampling = 0; /* 0 is default 1x */
	sensor_info->SensorPacketECCOrder = 1;

	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.pre.mipi_data_lp2hs_settle_dc;

		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		sensor_info->SensorGrabStartX = imgsensor_info.cap.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.cap.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.cap.mipi_data_lp2hs_settle_dc;

		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:

		sensor_info->SensorGrabStartX =
			imgsensor_info.normal_video.startx;
		sensor_info->SensorGrabStartY =
			imgsensor_info.normal_video.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.normal_video.mipi_data_lp2hs_settle_dc;

		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		sensor_info->SensorGrabStartX = imgsensor_info.hs_video.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.hs_video.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.hs_video.mipi_data_lp2hs_settle_dc;

		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		sensor_info->SensorGrabStartX =
			imgsensor_info.slim_video.startx;
		sensor_info->SensorGrabStartY =
			imgsensor_info.slim_video.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.slim_video.mipi_data_lp2hs_settle_dc;

		break;
	case MSDK_SCENARIO_ID_CUSTOM1:
		sensor_info->SensorGrabStartX =
			imgsensor_info.custom1.startx;
		sensor_info->SensorGrabStartY =
			imgsensor_info.custom1.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.custom1.mipi_data_lp2hs_settle_dc;
		break;
	case MSDK_SCENARIO_ID_CUSTOM2:
		sensor_info->SensorGrabStartX =
			imgsensor_info.custom2.startx;
		sensor_info->SensorGrabStartY =
			imgsensor_info.custom2.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.custom2.mipi_data_lp2hs_settle_dc;
		break;
	case MSDK_SCENARIO_ID_CUSTOM3:
		sensor_info->SensorGrabStartX =
			imgsensor_info.custom3.startx;
		sensor_info->SensorGrabStartY =
			imgsensor_info.custom3.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.custom3.mipi_data_lp2hs_settle_dc;
		break;
	default:
		sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.pre.mipi_data_lp2hs_settle_dc;
		break;
	}

	return ERROR_NONE;
}	/*	get_info  */


static kal_uint32 control(enum MSDK_SCENARIO_ID_ENUM scenario_id,
			MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
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
	case MSDK_SCENARIO_ID_CUSTOM1:
		custom1(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_CUSTOM2:
		custom2(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_CUSTOM3:
		custom3(image_window, sensor_config_data);
		break;
	default:
		LOG_INF("Error ScenarioId setting");
		preview(image_window, sensor_config_data);
		return ERROR_INVALID_SCENARIO_ID;
	}
	return ERROR_NONE;
}	/* control() */



static kal_uint32 set_video_mode(UINT16 framerate)
{
	LOG_INF("framerate = %d\n ", framerate);
	/* SetVideoMode Function should fix framerate */
	if (framerate == 0)
		/* Dynamic frame rate */
		return ERROR_NONE;
	spin_lock(&imgsensor_drv_lock);
	if ((framerate == 300) && (imgsensor.autoflicker_en == KAL_TRUE))
		imgsensor.current_fps = 296;
	else if ((framerate == 150) && (imgsensor.autoflicker_en == KAL_TRUE))
		imgsensor.current_fps = 146;
	else
		imgsensor.current_fps = framerate;
	spin_unlock(&imgsensor_drv_lock);
	set_max_framerate(imgsensor.current_fps, 1);

	return ERROR_NONE;
}

static kal_uint32 set_auto_flicker_mode(kal_bool enable, UINT16 framerate)
{
	LOG_INF("enable = %d, framerate = %d\n", enable, framerate);
	spin_lock(&imgsensor_drv_lock);
	if (enable) /*enable auto flicker*/
		imgsensor.autoflicker_en = KAL_TRUE;
	else /*Cancel Auto flick*/
		imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}


static kal_uint32 set_max_framerate_by_scenario(
		enum MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 framerate)
{
	kal_uint32 frame_length;

	LOG_INF("scenario_id = %d, framerate = %d\n", scenario_id, framerate);

	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		frame_length = imgsensor_info.pre.pclk / framerate * 10
				/ imgsensor_info.pre.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line =
			(frame_length > imgsensor_info.pre.framelength)
		? (frame_length - imgsensor_info.pre.framelength) : 0;
		imgsensor.frame_length =
			imgsensor_info.pre.framelength
			+ imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		if (framerate == 0)
			return ERROR_NONE;
		frame_length = imgsensor_info.normal_video.pclk /
				framerate * 10 /
				imgsensor_info.normal_video.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line =
			(frame_length > imgsensor_info.normal_video.framelength)
		? (frame_length - imgsensor_info.normal_video.framelength)
		: 0;
		imgsensor.frame_length =
			imgsensor_info.normal_video.framelength
			+ imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:

	if (imgsensor.current_fps != imgsensor_info.cap.max_framerate)
		LOG_INF(
			"Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n"
			, framerate, imgsensor_info.cap.max_framerate/10);
		frame_length = imgsensor_info.cap.pclk / framerate * 10
				/ imgsensor_info.cap.linelength;
		spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line =
			(frame_length > imgsensor_info.cap.framelength)
			  ? (frame_length - imgsensor_info.cap.framelength) : 0;
			imgsensor.frame_length =
				imgsensor_info.cap.framelength
				+ imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);

		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		frame_length = imgsensor_info.hs_video.pclk / framerate * 10
				/ imgsensor_info.hs_video.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line =
			(frame_length > imgsensor_info.hs_video.framelength)
			  ? (frame_length - imgsensor_info.hs_video.framelength)
			  : 0;
		imgsensor.frame_length =
			imgsensor_info.hs_video.framelength
				+ imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		frame_length = imgsensor_info.slim_video.pclk / framerate * 10
			/ imgsensor_info.slim_video.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line =
			(frame_length > imgsensor_info.slim_video.framelength)
			? (frame_length - imgsensor_info.slim_video.framelength)
			: 0;
		imgsensor.frame_length =
			imgsensor_info.slim_video.framelength
			+ imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;

	case MSDK_SCENARIO_ID_CUSTOM1:
		frame_length = imgsensor_info.custom1.pclk / framerate * 10
			/ imgsensor_info.custom1.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line =
			(frame_length > imgsensor_info.custom1.framelength)
			? (frame_length - imgsensor_info.custom1.framelength)
			: 0;
		imgsensor.frame_length =
			imgsensor_info.custom1.framelength
			+ imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;
	case MSDK_SCENARIO_ID_CUSTOM2:
		frame_length = imgsensor_info.custom2.pclk / framerate * 10
			/ imgsensor_info.custom2.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line =
			(frame_length > imgsensor_info.custom2.framelength)
			? (frame_length - imgsensor_info.custom2.framelength)
			: 0;
		imgsensor.frame_length =
			imgsensor_info.custom2.framelength
			+ imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;
	case MSDK_SCENARIO_ID_CUSTOM3:
		frame_length = imgsensor_info.custom3.pclk / framerate * 10
			/ imgsensor_info.custom3.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line =
			(frame_length > imgsensor_info.custom3.framelength)
			? (frame_length - imgsensor_info.custom3.framelength)
			: 0;
		imgsensor.frame_length =
			imgsensor_info.custom3.framelength
			+ imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;
	default:  /*coding with  preview scenario by default*/
		frame_length = imgsensor_info.pre.pclk / framerate * 10
			/ imgsensor_info.pre.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line =
			(frame_length > imgsensor_info.pre.framelength)
			? (frame_length - imgsensor_info.pre.framelength) : 0;
		imgsensor.frame_length =
			imgsensor_info.pre.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		LOG_INF("error scenario_id = %d, we use preview scenario\n",
			scenario_id);
		break;
	}
	return ERROR_NONE;
}


static kal_uint32 get_default_framerate_by_scenario(
		enum MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 *framerate)
{

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
	case MSDK_SCENARIO_ID_CUSTOM1:
		*framerate = imgsensor_info.custom1.max_framerate;
		break;
	case MSDK_SCENARIO_ID_CUSTOM2:
		*framerate = imgsensor_info.custom2.max_framerate;
		break;
	case MSDK_SCENARIO_ID_CUSTOM3:
		*framerate = imgsensor_info.custom3.max_framerate;
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
                write_cmos_sensor(0x0204, 0x0020);
		write_cmos_sensor(0x0600, 0x0001);
	} else {

		write_cmos_sensor(0x0600,0x0000);
	}
	spin_lock(&imgsensor_drv_lock);
	imgsensor.test_pattern = enable;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}

#if 0
static kal_int32 get_sensor_temperature(void)
{
	UINT8 temperature = 0;
	INT32 temperature_convert = 0;

	temperature = read_cmos_sensor_8(0x013a);

	if (temperature >= 0x0 && temperature <= 0x60)
		temperature_convert = temperature;
	else if (temperature >= 0x61 && temperature <= 0x7F)
		temperature_convert = 97;
	else if (temperature >= 0x80 && temperature <= 0xE2)
		temperature_convert = -30;
	else
		temperature_convert = (INT8)temperature | 0xFFFFFF0;

/* LOG_INF("temp_c(%d), read_reg(%d)\n", temperature_convert, temperature); */

	return temperature_convert;
}
#endif
static kal_uint32 feature_control(MSDK_SENSOR_FEATURE_ENUM feature_id,
				 UINT8 *feature_para, UINT32 *feature_para_len)
{
	UINT16 *feature_return_para_16=(UINT16 *) feature_para;
	UINT16 *feature_data_16=(UINT16 *) feature_para;
	UINT32 *feature_return_para_32=(UINT32 *) feature_para;
	UINT32 *feature_data_32=(UINT32 *) feature_para;
	unsigned long long *feature_data=(unsigned long long *) feature_para;
	struct SENSOR_VC_INFO_STRUCT *pvcinfo;

	struct SENSOR_WINSIZE_INFO_STRUCT *wininfo;
	struct SET_PD_BLOCK_INFO_T *PDAFinfo;
	MSDK_SENSOR_REG_INFO_STRUCT *sensor_reg_data=(MSDK_SENSOR_REG_INFO_STRUCT *) feature_para;
	#if S5KJN1_REMOSAIC_ENABLE
	kal_uint8 *data;
	#endif
	
	LOG_INF("feature_id = %d", feature_id);
	switch (feature_id) 
	{
		// prize add for its:test_sensor_fusion 20201029 start
		// case SENSOR_FEATURE_GET_OFFSET_TO_START_OF_EXPOSURE:
			// *(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = 9900000;
			// break;
		// prize add for its:test_sensor_fusion 20201029 end
		// prize add by linchong 20220228 for 5G platform start
		case SENSOR_FEATURE_GET_GAIN_RANGE_BY_SCENARIO:		
			*(feature_data + 1) = imgsensor_info.min_gain;		
			*(feature_data + 2) = imgsensor_info.max_gain;		
			break;	
		case SENSOR_FEATURE_GET_BASE_GAIN_ISO_AND_STEP:		
			*(feature_data + 0) = imgsensor_info.min_gain_iso;		
			*(feature_data + 1) = imgsensor_info.gain_step;		
			*(feature_data + 2) = imgsensor_info.gain_type;		
			break;	
		case SENSOR_FEATURE_GET_MIN_SHUTTER_BY_SCENARIO:		
			*(feature_data + 1) = imgsensor_info.min_shutter;		
			*(feature_data + 2) = imgsensor_info.exp_step;		
			break;	
		#if S5KJN1_REMOSAIC_ENABLE
		case SENSOR_FEATURE_GET_4CELL_DATA:
			// if(*(feature_data+2)!=8006)
				// LOG_INF("prize linchong error remosaic data size!\n");
			data = (kal_uint8 *)(uintptr_t)(*(feature_data + 1));
			switch (*feature_data){
				case FOUR_CELL_CAL_TYPE_GAIN_TBL://xtc+sensor_xtc
				memcpy(data, &xtc[0], XTC_SIZE);
				memcpy(data+XTC_SIZE, &sensor_xtc[0], SENSOR_XTC_SIZE);
				break;
				case FOUR_CELL_CAL_TYPE_DPC://pd_xtc
				memcpy(data, &pd_xtc[0], PD_XTC_SIZE);
				break;
				case FOUR_CELL_CAL_TYPE_FD_DPC://sw_ggc
				memcpy(data, &sw_ggc[0], SW_GGC_SIZE);
				break;
				default:
				LOG_INF("error remosaic data type!\n");
				break;
			}
			break;
		#endif
		case SENSOR_FEATURE_GET_BINNING_TYPE:		
					switch (*(feature_data + 1)) 
						{		
						case MSDK_SCENARIO_ID_CUSTOM3:			
							*feature_return_para_32 = 1; /*BINNING_NONE*/			
							break;		
							case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:		
							case MSDK_SCENARIO_ID_VIDEO_PREVIEW:		
							case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:		
							case MSDK_SCENARIO_ID_SLIM_VIDEO:		
							case MSDK_SCENARIO_ID_CAMERA_PREVIEW:							
							case MSDK_SCENARIO_ID_CUSTOM1:
							case MSDK_SCENARIO_ID_CUSTOM2:
						
							case MSDK_SCENARIO_ID_CUSTOM4:		
								default:			
								*feature_return_para_32 = 1; /*BINNING_AVERAGED*/			
								break;	
						}		
					LOG_INF("SENSOR_FEATURE_GET_BINNING_TYPE AE_binning_type:%d,\n",*feature_return_para_32);		
					*feature_para_len = 4;		
					break;
		case SENSOR_FEATURE_GET_AWB_REQ_BY_SCENARIO:	
			switch (*feature_data) {	
				case MSDK_SCENARIO_ID_CUSTOM3:			
					*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = 1;			
					break;		
					default:			
						*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = 0;			
						break;		
						}		
			break;
			case SENSOR_FEATURE_GET_FRAME_CTRL_INFO_BY_SCENARIO:		
				/*		 * 1, if driver support new sw frame sync		
				* set_shutter_frame_length() support third para auto_extend_en		 */		
				*(feature_data + 1) = 1;		/* margin info by scenario */		
				*(feature_data + 2) = imgsensor_info.margin;		
				break;	
		case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ_BY_SCENARIO:
			switch (*feature_data) {
			case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
				*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
					= imgsensor_info.cap.pclk;
				break;
			case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
				*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
					= imgsensor_info.normal_video.pclk;
				break;
			case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
				*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
					= imgsensor_info.hs_video.pclk;
				break;
			case MSDK_SCENARIO_ID_SLIM_VIDEO:
				*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
					= imgsensor_info.slim_video.pclk;
				break;
			case MSDK_SCENARIO_ID_CUSTOM1:
				*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
					= imgsensor_info.custom1.pclk;
				break;
			case MSDK_SCENARIO_ID_CUSTOM2:
				*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
					= imgsensor_info.custom2.pclk;
				break;
			case MSDK_SCENARIO_ID_CUSTOM3:
				*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
					= imgsensor_info.custom3.pclk;
				break;
			case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			default:
				*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
					= imgsensor_info.pre.pclk;
				break;
			}
			break;
		case SENSOR_FEATURE_GET_PERIOD_BY_SCENARIO:
			switch (*feature_data) {
			case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
				*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= (imgsensor_info.cap.framelength << 16)
					+ imgsensor_info.cap.linelength;
				break;
			case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
				*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= (imgsensor_info.normal_video.framelength << 16)
					+ imgsensor_info.normal_video.linelength;
				break;
			case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
				*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= (imgsensor_info.hs_video.framelength << 16)
					+ imgsensor_info.hs_video.linelength;
				break;
			case MSDK_SCENARIO_ID_SLIM_VIDEO:
				*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= (imgsensor_info.slim_video.framelength << 16)
					+ imgsensor_info.slim_video.linelength;
				break;
			case MSDK_SCENARIO_ID_CUSTOM1:
				*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= (imgsensor_info.custom1.framelength << 16)
					+ imgsensor_info.custom1.linelength;
				break;
			case MSDK_SCENARIO_ID_CUSTOM2:
				*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= (imgsensor_info.custom2.framelength << 16)
					+ imgsensor_info.custom2.linelength;
				break;
			case MSDK_SCENARIO_ID_CUSTOM3:
				*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= (imgsensor_info.custom3.framelength << 16)
					+ imgsensor_info.custom3.linelength;
				break;
			case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			default:
				*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= (imgsensor_info.pre.framelength << 16)
					+ imgsensor_info.pre.linelength;
				break;
			}
			break;
		// prize add by linchong 20220228 for 5G platform end
		case SENSOR_FEATURE_GET_PERIOD:
			*feature_return_para_16++ = imgsensor.line_length;
			*feature_return_para_16 = imgsensor.frame_length;
			*feature_para_len=4;
			break;
		case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
			LOG_INF("feature_Control imgsensor.pclk = %d,imgsensor.current_fps = %d\n", imgsensor.pclk,imgsensor.current_fps);
			*feature_return_para_32 = imgsensor.pclk;
			*feature_para_len=4;
			break;
		case SENSOR_FEATURE_SET_ESHUTTER:
			set_shutter(*feature_data);
			break;
		case SENSOR_FEATURE_SET_NIGHTMODE:
			break;
		case SENSOR_FEATURE_SET_GAIN:
			set_gain((UINT16) *feature_data);
			break;
		case SENSOR_FEATURE_SET_FLASHLIGHT:
			break;
		case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
			break;
		case SENSOR_FEATURE_SET_REGISTER:
			if((sensor_reg_data->RegData>>8)>0)
			write_cmos_sensor_8(sensor_reg_data->RegAddr, sensor_reg_data->RegData);
			else
			write_cmos_sensor_8(sensor_reg_data->RegAddr, sensor_reg_data->RegData);
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
			get_default_framerate_by_scenario((enum MSDK_SCENARIO_ID_ENUM)*(feature_data), (MUINT32 *)(uintptr_t)(*(feature_data+1)));
			break;
		case SENSOR_FEATURE_GET_PDAF_DATA:
			LOG_INF("SENSOR_FEATURE_GET_PDAF_DATA\n");
			//read_eeprom((kal_uint16 )(*feature_data),(char*)(uintptr_t)(*(feature_data+1)),(kal_uint32)(*(feature_data+2)));
			//read_ov16a1q_eeprom((kal_uint16 )(*feature_data),(char*)(uintptr_t)(*(feature_data+1)),(kal_uint32)(*(feature_data+2)));
			break;
	    case SENSOR_FEATURE_SET_PDAF:
		LOG_INF("PDAF mode :%d\n", *feature_data_16);
		imgsensor.pdaf_mode = *feature_data_16;
		break;
	case SENSOR_FEATURE_SET_TEST_PATTERN:
		set_test_pattern_mode((BOOL)*feature_data);
		break;
	case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE:
		/* for factory mode auto testing */
		*feature_return_para_32 = imgsensor_info.checksum_value;
		*feature_para_len = 4;
		break;
	//prize-camera  add for  by zhuzhengjiang 20190830-begin
	case SENSOR_FEATURE_SET_FRAMERATE:
		LOG_INF("current fps :%d\n", (UINT16)*feature_data_32);
		spin_lock(&imgsensor_drv_lock);
		imgsensor.current_fps = (UINT16)*feature_data_32;
		spin_unlock(&imgsensor_drv_lock);
		break;
	case SENSOR_FEATURE_SET_HDR:
		LOG_INF("ihdr enable :%d\n", (UINT8)*feature_data_32);
			LOG_INF("Warning! Not Support IHDR Feature");
			spin_lock(&imgsensor_drv_lock);
			imgsensor.ihdr_en = (UINT8) *feature_data_32;
			//imgsensor.ihdr_en = *feature_data_32;
			spin_unlock(&imgsensor_drv_lock);
			break;
	//prize-camera  add for  by zhuzhengjiang 20190830-end
		case SENSOR_FEATURE_GET_CROP_INFO:
			LOG_INF("SENSOR_FEATURE_GET_CROP_INFO scenarioId:%d\n", (UINT32)*feature_data);
			wininfo = (struct SENSOR_WINSIZE_INFO_STRUCT *)(uintptr_t)(*(feature_data+1));

			switch (*feature_data_32) 
			{
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
				case MSDK_SCENARIO_ID_CUSTOM1:
				memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[5],sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
				break;
				case MSDK_SCENARIO_ID_CUSTOM2:
				memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[6],sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
				break;
				case MSDK_SCENARIO_ID_CUSTOM3:
				memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[7],sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
				break;
				case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
				default:
				memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[0],sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
				break;
			}
			break;
		case SENSOR_FEATURE_GET_VC_INFO:
		LOG_INF("SENSOR_FEATURE_GET_VC_INFO %d\n",
			(UINT16) *feature_data);

		pvcinfo =
		 (struct SENSOR_VC_INFO_STRUCT *)(uintptr_t)(*(feature_data+1));
		switch (*feature_data_32) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			memcpy((void *)pvcinfo, (void *)&SENSOR_VC_INFO[1],
				sizeof(struct SENSOR_VC_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			memcpy((void *)pvcinfo, (void *)&SENSOR_VC_INFO[2],
				sizeof(struct SENSOR_VC_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			memcpy((void *)pvcinfo, (void *)&SENSOR_VC_INFO[0],
			       sizeof(struct SENSOR_VC_INFO_STRUCT));
			break;
		}
		break;
		case SENSOR_FEATURE_GET_PDAF_INFO:
			LOG_INF("SENSOR_FEATURE_GET_PDAF_INFO scenarioId:%d\n", (UINT32)*feature_data);
			PDAFinfo= (struct SET_PD_BLOCK_INFO_T *)(uintptr_t)(*(feature_data+1));

			switch (*feature_data) 
			{
				case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
				case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
				case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
				memcpy((void *)PDAFinfo,(void *)&imgsensor_pd_info,sizeof(struct SET_PD_BLOCK_INFO_T));
				break;
				// prize add by zhuzhengjiang video pdaf 20200427 start
				//case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
				//case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
				//memcpy((void *)PDAFinfo,(void *)&imgsensor_pd_binning_info,sizeof(struct SET_PD_BLOCK_INFO_T));
				//break;
				// prize add by zhuzhengjiang video pdaf 20200427 end
				case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
				case MSDK_SCENARIO_ID_SLIM_VIDEO:
				case MSDK_SCENARIO_ID_CUSTOM1:
				case MSDK_SCENARIO_ID_CUSTOM2:
				case MSDK_SCENARIO_ID_CUSTOM3:
				//case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
				default:
				break;
			}
			break;
		case SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY:
			LOG_INF("SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY scenarioId:%llu\n", *feature_data);
			//PDAF capacity enable or not, OV16A10 only full size support PDAF
			switch (*feature_data) 
			{
				case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
				*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;  //1
				break;
				case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
				*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1; // video & capture use same setting
				break;
				case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
				*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
				break;
				case MSDK_SCENARIO_ID_SLIM_VIDEO:
				*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
				break;
				case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
				*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1; //// prize add by zhuzhengjiang video pdaf 20200427 start
				break;
				default:
				*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
				break;
			}
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
		case SENSOR_FEATURE_SET_IHDR_SHUTTER_GAIN:
			LOG_INF("SENSOR_SET_SENSOR_IHDR LE=%d, SE=%d, Gain=%d\n",(UINT16)*feature_data,(UINT16)*(feature_data+1),(UINT16)*(feature_data+2));
			//ihdr_write_shutter_gain((UINT16)*feature_data,(UINT16)*(feature_data+1),(UINT16)*(feature_data+2));
			break;
		case SENSOR_FEATURE_GET_MIPI_PIXEL_RATE:
		{
			kal_uint32 rate;

			switch (*feature_data) {
				case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
				rate = imgsensor_info.cap.mipi_pixel_rate;
				break;
				case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
				rate = imgsensor_info.normal_video.mipi_pixel_rate;
				break;
				case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
				rate = imgsensor_info.hs_video.mipi_pixel_rate;
				break;
				case MSDK_SCENARIO_ID_SLIM_VIDEO:
				rate = imgsensor_info.slim_video.mipi_pixel_rate;
				break;
				case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
				default:
				rate = imgsensor_info.pre.mipi_pixel_rate;
				break;
			}
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = rate;
		}
		break;
		case SENSOR_FEATURE_SET_SHUTTER_FRAME_TIME://lzl
			set_shutter_frame_length((UINT16)*feature_data,(UINT16)*(feature_data+1));
			break;
		// prize add by zhuzhengjiang for long exposure 20200721 start
		case SENSOR_FEATURE_GET_AE_FRAME_MODE_FOR_LE:
			memcpy(feature_return_para_32,
			&imgsensor.ae_frm_mode, sizeof(struct IMGSENSOR_AE_FRM_MODE));
			break;
		case SENSOR_FEATURE_GET_AE_EFFECTIVE_FRAME_FOR_LE:
			*feature_return_para_32 = imgsensor.current_ae_effective_frame;
			break;
		// prize add by zhuzhengjiang for long exposure 20200721 end
		default:
			break;
	}

	return ERROR_NONE;
} /* feature_control() */

static struct  SENSOR_FUNCTION_STRUCT sensor_func = {
	open,
	get_info,
	get_resolution,
	feature_control,
	control,
	close
};

UINT32 S5KJN1_MIPI_RAW_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc)
{
	/* To Do : Check Sensor status here */
	if (pfFunc != NULL)
		*pfFunc = &sensor_func;
	return ERROR_NONE;
} /* s5kjn1_MIPI_RAW_SensorInit */
