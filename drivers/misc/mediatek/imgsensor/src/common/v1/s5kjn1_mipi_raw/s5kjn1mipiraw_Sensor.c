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
#define MODULE_ID_OFFSET 0x0000

#endif

#ifdef VENDOR_EDIT
#define DEVICE_VERSION_S5KJN1    "s5kjn1"
//add by wq  extern void register_imgsensor_deviceinfo(char *name, char *version, u8 module_id);
static kal_uint8 deviceInfo_register_value = 0x00;
#endif


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
		.linelength = 4584,
		.framelength = 4064,
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
		.linelength = 8688,
		.framelength = 6400,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 8160,
		.grabwindow_height = 6144,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 100,
		.mipi_pixel_rate = 556800000,
	},
	.normal_video =  {
		.pclk = 560000000,
		.linelength = 4584,
		.framelength = 4064,
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
	.frame_time_delay_frame = 1,

	.isp_driving_current = ISP_DRIVING_6MA,
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
	/* .mipi_sensor_type = MIPI_OPHY_NCSI2, */
	/* 0,MIPI_OPHY_NCSI2;  1,MIPI_OPHY_CSI2 */
	.mipi_sensor_type = MIPI_OPHY_NCSI2, //0,MIPI_OPHY_NCSI2;  1,MIPI_OPHY_CSI2
	.mipi_settle_delay_mode = 0,
	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_4CELL_BAYER_Gr,

	.mclk = 24, /* mclk value, suggest 24 or 26 for 24Mhz or 26Mhz */
	/*.mipi_lane_num = SENSOR_MIPI_4_LANE,*/
	.mipi_lane_num = SENSOR_MIPI_4_LANE,

	.i2c_addr_table = {0x21,0x5a,0xff},
	/* record sensor support all write id addr,
	 * only supprt 4 must end with 0xff
	 */
	.i2c_speed = 1000, /* i2c read/write speed */
};

static struct SENSOR_VC_INFO_STRUCT SENSOR_VC_INFO[3]=
{  // Preview mode setting
   {0x02, 0x0A,   0x00,   0x08, 0x40, 0x00,
    0x00, 0x2B, 0x0910, 0x06D0, 0x01, 0x00, 0x0000, 0x0000,
    0x01, 0x30, 0x0136, 0x0170, 0x03, 0x00, 0x0000, 0x0000},
   // Capture mode setting
   {0x02, 0x0A,   0x00,   0x08, 0x40, 0x00,
    0x00, 0x2B, 0x1220, 0x0DA0, 0x01, 0x00, 0x0000, 0x0000,
    0x01, 0x30, 0x026C, 0x02D0, 0x03, 0x00, 0x0000, 0x0000},
   // Video mode setting
   {0x02, 0x0A,   0x00,   0x08, 0x40, 0x00,
    0x00, 0x2B, 0x1220, 0x0DA0, 0x01, 0x00, 0x0000, 0x0000,
    0x01, 0x30, 0x026C, 0x0228, 0x03, 0x00, 0x0000, 0x0000}};

static struct imgsensor_struct imgsensor = {
	.mirror = IMAGE_NORMAL,    //mirrorflip information
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
	{ 8160, 6144,    0,    0, 8160, 6144, 4080, 3072,  0,   0, 4080, 3072,    0,    0, 4080, 3072}, // preveiw
	{ 8160, 6144,    0,    0, 8160, 6144, 8160, 6144,  0,   0, 8160, 6144,    0,    0, 8160, 6144}, // capture
	{ 8160, 6144,    0,    0, 8160, 6144, 4080, 3072,  0,   0, 4080, 3072,    0,    0, 4080, 3072}, // video
	{ 8160, 6144,    0,    0, 8160, 6144, 2040, 1536,380, 408, 1280,  720,    0,    0, 1280,  720}, // high speed
	{ 8160, 6144,    0,    0, 8160, 6144, 4080, 3072,  0, 388, 4080, 2296,    0,    0, 4080, 2296}, // slim video
};


static struct SET_PD_BLOCK_INFO_T imgsensor_pd_info =
{
	.i4OffsetX = 16,
	.i4OffsetY = 60,
	.i4PitchX  = 32,
	.i4PitchY  = 32,
	.i4PairNum  =16,
	.i4SubBlkW  =8,
	.i4SubBlkH  =8,
	.i4PosL = {{18,61},{26,61},{34,61},{42,61},{22,73},{30,73},{38,73},{46,73},{18,81},{26,81},{34,81},{42,81},{22,85},{30,85},{38,85},{46,85}},
	.i4PosR = {{18,65},{26,65},{34,65},{42,65},{22,69},{30,69},{38,69},{46,69},{18,77},{26,77},{34,77},{42,77},{22,89},{30,89},{38,89},{46,89}},
	.i4BlockNumX = 124,
	.i4BlockNumY = 90,
	.iMirrorFlip = 0,
	.i4Crop = { {0, 0}, {0, 0}, {0, 380}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0} },
};

#if  0 
static struct SET_PD_BLOCK_INFO_T imgsensor_pd_info_16_9 =
{
	.i4OffsetX = 16,
	.i4OffsetY = 8,
	.i4PitchX  = 32,
	.i4PitchY  = 32,
	.i4PairNum  =16,
	.i4SubBlkW  =8,
	.i4SubBlkH  =8,
	.i4PosL = {{18,61},{26,61},{34,61},{42,61},{22,73},{30,73},{38,73},{46,73},{18,81},{26,81},{34,81},{42,81},{22,85},{30,85},{38,85},{46,85}},
	.i4PosR = {{18,65},{26,65},{34,65},{42,65},{22,69},{30,69},{38,69},{46,69},{18,77},{26,77},{34,77},{42,77},{22,89},{30,89},{38,89},{46,89}},
	.i4BlockNumX = 124,
	.i4BlockNumY = 70,
	.iMirrorFlip = 0,
	.i4Crop = { {0, 0}, {0, 0}, {0, 380}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0} },
};

static struct SET_PD_BLOCK_INFO_T imgsensor_pd_info_binning =
{
	.i4OffsetX = 8,
	.i4OffsetY = 14,
	.i4PitchX  = 16,
	.i4PitchY  = 16,
	.i4PairNum  =4,
	.i4SubBlkW  =8,
	.i4SubBlkH  =8,
	.i4PosL = {{8,15},{16,15},{8,25},{16,25}},
	.i4PosR = {{8,17},{16,17},{8,23},{16,23}},
	.i4BlockNumX = 124,
	.i4BlockNumY = 69,
	.iMirrorFlip = 0,
	.i4Crop = { {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0} },
};

#endif
extern int iReadReg(u16 a_u2Addr , u8 * a_puBuff , u16 i2cId);
extern int iWriteReg(u16 a_u2Addr , u32 a_u4Data , u32 a_u4Bytes , u16 i2cId);
extern void kdSetI2CSpeed(u16 i2cSpeed);

#ifdef VENDOR_EDIT
static kal_uint8 gS5kgm1_SN[16];
static void read_eeprom_SN(void)
{
	kal_uint16 idx = 0;
	kal_uint8 *get_byte= &gS5kgm1_SN[0];
	for (idx = 0; idx <16; idx++) {
		char pusendcmd[2] = {0x00 , (char)((0xB0 + idx) & 0xFF) };
		iReadRegI2C(pusendcmd , 2, (u8*)&get_byte[idx],1, 0xA0);
		LOG_INF("s5kgm1_SN[%d]: 0x%x  0x%x\n", idx, get_byte[idx], gS5kgm1_SN[idx]);
	}
}
#define   WRITE_DATA_MAX_LENGTH     (16)
#if  0  //add by wq
static kal_int32 table_write_eeprom_30Bytes(kal_uint16 addr, kal_uint8 *para, kal_uint32 len)
{
	kal_int32 ret = IMGSENSOR_RETURN_SUCCESS;
	char pusendcmd[WRITE_DATA_MAX_LENGTH+2];
	pusendcmd[0] = (char)(addr >> 8);
	pusendcmd[1] = (char)(addr & 0xFF);

	memcpy(&pusendcmd[2], para, len);

	ret = iBurstWriteReg((kal_uint8 *)pusendcmd , (len + 2), 0xA0);

	return ret;
}

static kal_uint16 read_cmos_eeprom_8(kal_uint16 addr)
{
	kal_uint16 get_byte=0;
	char pusendcmd[2] = {(char)(addr >> 8) , (char)(addr & 0xFF) };
	iReadRegI2C(pusendcmd , 2, (u8*)&get_byte, 1, 0xA0);
	return get_byte;
}

static kal_int32 write_Module_data(ACDK_SENSOR_ENGMODE_STEREO_STRUCT * pStereodata)
{
	kal_int32  ret = IMGSENSOR_RETURN_SUCCESS;
	kal_uint16 data_base, data_length;
	kal_uint32 idx, idy;
	kal_uint8 *pData;
	UINT32 i = 0;
	if(pStereodata != NULL) {
		pr_debug("s5kgm1 SET_SENSOR_OTP: 0x%x %d 0x%x %d\n",
                       pStereodata->uSensorId,
                       pStereodata->uDeviceId,
                       pStereodata->baseAddr,
                       pStereodata->dataLength);

		data_base = pStereodata->baseAddr;
		data_length = pStereodata->dataLength;
		pData = pStereodata->uData;
		if ((pStereodata->uSensorId == S5KJN1_SENSOR_ID)
				&& (data_base == 0x1640)
				&& (data_length == 1561)) {
			pr_debug("s5kgm1 Write: %x %x %x %x %x %x %x %x\n", pData[0], pData[39], pData[40], pData[1556],
					pData[1557], pData[1558], pData[1559], pData[1560]);
			idx = data_length/WRITE_DATA_MAX_LENGTH;
			idy = data_length%WRITE_DATA_MAX_LENGTH;
			for (i = 0; i < idx; i++ ) {
				ret = table_write_eeprom_30Bytes((data_base+WRITE_DATA_MAX_LENGTH*i),
					    &pData[WRITE_DATA_MAX_LENGTH*i], WRITE_DATA_MAX_LENGTH);
				if (ret != IMGSENSOR_RETURN_SUCCESS) {
				    pr_err("write_eeprom error: i= %d\n", i);
					return IMGSENSOR_RETURN_ERROR;
				}
				msleep(6);
			}
			ret = table_write_eeprom_30Bytes((data_base+WRITE_DATA_MAX_LENGTH*idx),
				      &pData[WRITE_DATA_MAX_LENGTH*idx], idy);
			if (ret != IMGSENSOR_RETURN_SUCCESS) {
				pr_err("write_eeprom error: idx= %d idy= %d\n", idx, idy);
				return IMGSENSOR_RETURN_ERROR;
			}
			msleep(6);
			pr_debug("com 0x1640:0x%x\n", read_cmos_eeprom_8(0x1640));
			msleep(6);
			pr_debug("com 0x1667:0x%x\n", read_cmos_eeprom_8(0x1667));
			msleep(6);
			pr_debug("innal 0x1668:0x%x\n", read_cmos_eeprom_8(0x1668));
			msleep(6);
			pr_debug("innal 0x1C54:0x%x\n", read_cmos_eeprom_8(0x1C54));
			msleep(6);
			pr_debug("tail1 0x1C55:0x%x\n", read_cmos_eeprom_8(0x1C55));
			msleep(6);
			pr_debug("tail2 0x1C56:0x%x\n", read_cmos_eeprom_8(0x1C56));
			msleep(6);
			pr_debug("tail3 0x1C57:0x%x\n", read_cmos_eeprom_8(0x1C57));
			msleep(6);
			pr_debug("tail4 0x1C58:0x%x\n", read_cmos_eeprom_8(0x1C58));
			msleep(6);
			pr_debug("s5kgm1write_Module_data Write end\n");
		}else {
			pr_err("Invalid Sensor id:0x%x write_gm1 eeprom\n", pStereodata->uSensorId);
			return IMGSENSOR_RETURN_ERROR;
		}
	} else {
		pr_err("s5kgm1write_Module_data pStereodata is null\n");
		return IMGSENSOR_RETURN_ERROR;
	}
	return ret;
}
#endif //add by wq


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
	// 1H=5024/4820000000 =10.4 us
	// 16s=16000000/10.4 =1538462
	//1s=1000000/10.4=96153
	if (shutter >= 0xFFF0) {  // need to modify line_length & PCLK
		bNeedSetNormalMode = KAL_TRUE;


		if (shutter >= 1538000) {  //>16s
			shutter = 1538000;
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
		write_cmos_sensor(0x0202, imgsensor.shutter);


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

	if (gain < BASEGAIN || gain > 32 * BASEGAIN) {
		LOG_INF("Error gain setting");
		if (gain < BASEGAIN) {
			gain = BASEGAIN;
		} else if (gain > 32 * BASEGAIN) {
			gain = 32 * BASEGAIN;
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
#ifdef USE_TNP_BURST
static kal_uint16 addr_data_pair_init[] =   {
    0x6028,0x2400,
	0x602A,0x7700,
	0x6F12,0x1753,
	0x6F12,0x01FC,
	0x6F12,0xE702,
	0x6F12,0x6385,
	0x6F12,0xB787,
	0x6F12,0x0024,
	0x6F12,0x9386,
	0x6F12,0xC701,
	0x6F12,0xB7B7,
	0x6F12,0x0024,
	0x6F12,0x3777,
	0x6F12,0x0024,
	0x6F12,0x9387,
	0x6F12,0xC77F,
	0x6F12,0x1307,
	0x6F12,0x07C7,
	0x6F12,0x958F,
	0x6F12,0x2328,
	0x6F12,0xD774,
	0x6F12,0x231A,
	0x6F12,0xF774,
	0x6F12,0x012F,
	0x6F12,0xB777,
	0x6F12,0x0024,
	0x6F12,0x1307,
	0x6F12,0xD003,
	0x6F12,0x23A0,
	0x6F12,0xE774,
	0x6F12,0x1753,
	0x6F12,0x01FC,
	0x6F12,0x6700,
	0x6F12,0x2384,
	0x6F12,0x0000,
	0x6F12,0x0000,
	0x6F12,0x8547,
	0x6F12,0x6310,
	0x6F12,0xF506,
	0x6F12,0x3786,
	0x6F12,0x0024,
	0x6F12,0x9306,
	0x6F12,0x4601,
	0x6F12,0x83C7,
	0x6F12,0x4600,
	0x6F12,0xA1CB,
	0x6F12,0xB737,
	0x6F12,0x0024,
	0x6F12,0x83A7,
	0x6F12,0x875C,
	0x6F12,0x83D6,
	0x6F12,0x2600,
	0x6F12,0x83D7,
	0x6F12,0x271E,
	0x6F12,0x13D7,
	0x6F12,0x2700,
	0x6F12,0xB707,
	0x6F12,0x0140,
	0x6F12,0x83D5,
	0x6F12,0x27F0,
	0x6F12,0x8357,
	0x6F12,0x4601,
	0x6F12,0x1306,
	0x6F12,0xE7FF,
	0x6F12,0xB697,
	0x6F12,0x8D8F,
	0x6F12,0xC207,
	0x6F12,0xC183,
	0x6F12,0x9396,
	0x6F12,0x0701,
	0x6F12,0xC186,
	0x6F12,0x635F,
	0x6F12,0xD600,
	0x6F12,0x8907,
	0x6F12,0x998F,
	0x6F12,0x9396,
	0x6F12,0x0701,
	0x6F12,0xC186,
	0x6F12,0x9397,
	0x6F12,0x0601,
	0x6F12,0xC183,
	0x6F12,0x37B7,
	0x6F12,0x0040,
	0x6F12,0x2311,
	0x6F12,0xF7A0,
	0x6F12,0x8280,
	0x6F12,0xE3D8,
	0x6F12,0x06FE,
	0x6F12,0xBA97,
	0x6F12,0xF917,
	0x6F12,0xCDB7,
	0x6F12,0xB717,
	0x6F12,0x0024,
	0x6F12,0x9387,
	0x6F12,0x07CA,
	0x6F12,0xAA97,
	0x6F12,0x3387,
	0x6F12,0xB700,
	0x6F12,0x8D8F,
	0x6F12,0x83C5,
	0x6F12,0xD705,
	0x6F12,0xB747,
	0x6F12,0x0024,
	0x6F12,0x9386,
	0x6F12,0x078F,
	0x6F12,0x83D7,
	0x6F12,0xE670,
	0x6F12,0x0905,
	0x6F12,0x0347,
	0x6F12,0xC705,
	0x6F12,0x630B,
	0x6F12,0xF500,
	0x6F12,0x8567,
	0x6F12,0xB697,
	0x6F12,0x03A6,
	0x6F12,0x8794,
	0x6F12,0xB306,
	0x6F12,0xB700,
	0x6F12,0xB296,
	0x6F12,0x23A4,
	0x6F12,0xD794,
	0x6F12,0x2207,
	0x6F12,0x3305,
	0x6F12,0xB700,
	0x6F12,0x4205,
	0x6F12,0x4181,
	0x6F12,0x8280,
	0x6F12,0x5D71,
	0x6F12,0xA2C6,
	0x6F12,0xA6C4,
	0x6F12,0x7324,
	0x6F12,0x2034,
	0x6F12,0xF324,
	0x6F12,0x1034,
	0x6F12,0x7360,
	0x6F12,0x0430,
	0x6F12,0x2AD8,
	0x6F12,0x2ED6,
	0x6F12,0x3545,
	0x6F12,0x9305,
	0x6F12,0x8008,
	0x6F12,0x22DA,
	0x6F12,0x3ECE,
	0x6F12,0x86C2,
	0x6F12,0x96C0,
	0x6F12,0x1ADE,
	0x6F12,0x1EDC,
	0x6F12,0x32D4,
	0x6F12,0x36D2,
	0x6F12,0x3AD0,
	0x6F12,0x42CC,
	0x6F12,0x46CA,
	0x6F12,0x72C8,
	0x6F12,0x76C6,
	0x6F12,0x7AC4,
	0x6F12,0x7EC2,
	0x6F12,0x9710,
	0x6F12,0x00FC,
	0x6F12,0xE780,
	0x6F12,0x40F5,
	0x6F12,0x9377,
	0x6F12,0x8500,
	0x6F12,0x2A84,
	0x6F12,0x85C3,
	0x6F12,0xB737,
	0x6F12,0x0024,
	0x6F12,0x9387,
	0x6F12,0x077C,
	0x6F12,0x03D7,
	0x6F12,0x6702,
	0x6F12,0x0507,
	0x6F12,0x2393,
	0x6F12,0xE702,
	0x6F12,0x9710,
	0x6F12,0x00FC,
	0x6F12,0xE780,
	0x6F12,0x00ED,
	0x6F12,0x0545,
	0x6F12,0xD535,
	0x6F12,0x1374,
	0x6F12,0x0408,
	0x6F12,0x11CC,
	0x6F12,0xB737,
	0x6F12,0x0024,
	0x6F12,0x9387,
	0x6F12,0x077C,
	0x6F12,0x03D7,
	0x6F12,0x8705,
	0x6F12,0x0507,
	0x6F12,0x239C,
	0x6F12,0xE704,
	0x6F12,0x9710,
	0x6F12,0x00FC,
	0x6F12,0xE780,
	0x6F12,0x6065,
	0x6F12,0x9640,
	0x6F12,0x8642,
	0x6F12,0x7253,
	0x6F12,0xE253,
	0x6F12,0x5254,
	0x6F12,0x4255,
	0x6F12,0xB255,
	0x6F12,0x2256,
	0x6F12,0x9256,
	0x6F12,0x0257,
	0x6F12,0xF247,
	0x6F12,0x6248,
	0x6F12,0xD248,
	0x6F12,0x424E,
	0x6F12,0xB24E,
	0x6F12,0x224F,
	0x6F12,0x924F,
	0x6F12,0x7370,
	0x6F12,0x0430,
	0x6F12,0x7390,
	0x6F12,0x1434,
	0x6F12,0x7310,
	0x6F12,0x2434,
	0x6F12,0x3644,
	0x6F12,0xA644,
	0x6F12,0x6161,
	0x6F12,0x7300,
	0x6F12,0x2030,
	0x6F12,0x1743,
	0x6F12,0x01FC,
	0x6F12,0xE702,
	0x6F12,0xC369,
	0x6F12,0xB787,
	0x6F12,0x0024,
	0x6F12,0x83A4,
	0x6F12,0xC700,
	0x6F12,0x2A84,
	0x6F12,0x0146,
	0x6F12,0xA685,
	0x6F12,0x1145,
	0x6F12,0x9790,
	0x6F12,0xFFFB,
	0x6F12,0xE780,
	0x6F12,0x60AD,
	0x6F12,0x2285,
	0x6F12,0x97E0,
	0x6F12,0xFFFB,
	0x6F12,0xE780,
	0x6F12,0x40A9,
	0x6F12,0xB787,
	0x6F12,0x0024,
	0x6F12,0x9387,
	0x6F12,0x87F5,
	0x6F12,0x03C7,
	0x6F12,0x0700,
	0x6F12,0x8546,
	0x6F12,0x6315,
	0x6F12,0xD706,
	0x6F12,0xB776,
	0x6F12,0x0024,
	0x6F12,0x03A6,
	0x6F12,0x468D,
	0x6F12,0x8945,
	0x6F12,0x9386,
	0x6F12,0xA700,
	0x6F12,0x630F,
	0x6F12,0xB600,
	0x6F12,0x9386,
	0x6F12,0x2700,
	0x6F12,0x630B,
	0x6F12,0xE600,
	0x6F12,0x3717,
	0x6F12,0x0024,
	0x6F12,0x0356,
	0x6F12,0x6738,
	0x6F12,0x2D47,
	0x6F12,0x6314,
	0x6F12,0xE600,
	0x6F12,0x9386,
	0x6F12,0xA700,
	0x6F12,0xB747,
	0x6F12,0x0024,
	0x6F12,0x83A5,
	0x6F12,0xC786,
	0x6F12,0x2946,
	0x6F12,0x1305,
	0x6F12,0x0404,
	0x6F12,0x83C7,
	0x6F12,0xC52F,
	0x6F12,0x2148,
	0x6F12,0x1D8E,
	0x6F12,0x8147,
	0x6F12,0x3387,
	0x6F12,0xF500,
	0x6F12,0xB388,
	0x6F12,0xF600,
	0x6F12,0x0317,
	0x6F12,0xE73C,
	0x6F12,0x8398,
	0x6F12,0x0800,
	0x6F12,0x8907,
	0x6F12,0x1105,
	0x6F12,0x4697,
	0x6F12,0x3317,
	0x6F12,0xC700,
	0x6F12,0x232E,
	0x6F12,0xE5FE,
	0x6F12,0xE391,
	0x6F12,0x07FF,
	0x6F12,0x0546,
	0x6F12,0xA685,
	0x6F12,0x1145,
	0x6F12,0x9790,
	0x6F12,0xFFFB,
	0x6F12,0xE780,
	0x6F12,0x60A4,
	0x6F12,0x1743,
	0x6F12,0x01FC,
	0x6F12,0x6700,
	0x6F12,0x0361,
	0x6F12,0x1743,
	0x6F12,0x01FC,
	0x6F12,0xE702,
	0x6F12,0xA35C,
	0x6F12,0xB787,
	0x6F12,0x0024,
	0x6F12,0x83A4,
	0x6F12,0x8700,
	0x6F12,0x4111,
	0x6F12,0xAA89,
	0x6F12,0x2E8A,
	0x6F12,0xB28A,
	0x6F12,0xA685,
	0x6F12,0x0146,
	0x6F12,0x1145,
	0x6F12,0x36C6,
	0x6F12,0x9790,
	0x6F12,0xFFFB,
	0x6F12,0xE780,
	0x6F12,0x60A1,
	0x6F12,0xB787,
	0x6F12,0x0024,
	0x6F12,0x1387,
	0x6F12,0x87F5,
	0x6F12,0x0347,
	0x6F12,0x2701,
	0x6F12,0x1384,
	0x6F12,0x87F5,
	0x6F12,0xB246,
	0x6F12,0x0149,
	0x6F12,0x11CF,
	0x6F12,0x3767,
	0x6F12,0x0024,
	0x6F12,0x0357,
	0x6F12,0x2777,
	0x6F12,0xB777,
	0x6F12,0x0024,
	0x6F12,0x9387,
	0x6F12,0x07C7,
	0x6F12,0x0E07,
	0x6F12,0x03D9,
	0x6F12,0x871C,
	0x6F12,0x2394,
	0x6F12,0xE71C,
	0x6F12,0x5686,
	0x6F12,0xD285,
	0x6F12,0x4E85,
	0x6F12,0x97E0,
	0x6F12,0xFFFB,
	0x6F12,0xE780,
	0x6F12,0x0088,
	0x6F12,0x8347,
	0x6F12,0x2401,
	0x6F12,0x89C7,
	0x6F12,0xB777,
	0x6F12,0x0024,
	0x6F12,0x239C,
	0x6F12,0x27E3,
	0x6F12,0x0546,
	0x6F12,0xA685,
	0x6F12,0x1145,
	0x6F12,0x9790,
	0x6F12,0xFFFB,
	0x6F12,0xE780,
	0x6F12,0xC09B,
	0x6F12,0x4101,
	0x6F12,0x1743,
	0x6F12,0x01FC,
	0x6F12,0x6700,
	0x6F12,0xA357,
	0x6F12,0x1743,
	0x6F12,0x01FC,
	0x6F12,0xE702,
	0x6F12,0x0353,
	0x6F12,0x3784,
	0x6F12,0x0024,
	0x6F12,0x9307,
	0x6F12,0x04F4,
	0x6F12,0x83C7,
	0x6F12,0x4701,
	0x6F12,0x5D71,
	0x6F12,0x2A89,
	0x6F12,0x8DEF,
	0x6F12,0xB787,
	0x6F12,0x0024,
	0x6F12,0x03A4,
	0x6F12,0x4700,
	0x6F12,0x0146,
	0x6F12,0x1145,
	0x6F12,0xA285,
	0x6F12,0x9790,
	0x6F12,0xFFFB,
	0x6F12,0xE780,
	0x6F12,0x2098,
	0x6F12,0x4A85,
	0x6F12,0x97E0,
	0x6F12,0xFFFB,
	0x6F12,0xE780,
	0x6F12,0x400F,
	0x6F12,0x0546,
	0x6F12,0xA285,
	0x6F12,0x1145,
	0x6F12,0x9790,
	0x6F12,0xFFFB,
	0x6F12,0xE780,
	0x6F12,0xA096,
	0x6F12,0x6161,
	0x6F12,0x1743,
	0x6F12,0x01FC,
	0x6F12,0x6700,
	0x6F12,0xE351,
	0x6F12,0x1304,
	0x6F12,0x04F4,
	0x6F12,0x0347,
	0x6F12,0x5401,
	0x6F12,0x8547,
	0x6F12,0x6310,
	0x6F12,0xF716,
	0x6F12,0xB785,
	0x6F12,0x0024,
	0x6F12,0x9384,
	0x6F12,0x05F8,
	0x6F12,0x4146,
	0x6F12,0x9385,
	0x6F12,0x05F8,
	0x6F12,0x0A85,
	0x6F12,0x9740,
	0x6F12,0x01FC,
	0x6F12,0xE780,
	0x6F12,0x6058,
	0x6F12,0x4146,
	0x6F12,0x9385,
	0x6F12,0x0401,
	0x6F12,0x0808,
	0x6F12,0x9740,
	0x6F12,0x01FC,
	0x6F12,0xE780,
	0x6F12,0x6057,
	0x6F12,0x4146,
	0x6F12,0x9385,
	0x6F12,0x0402,
	0x6F12,0x0810,
	0x6F12,0x9740,
	0x6F12,0x01FC,
	0x6F12,0xE780,
	0x6F12,0x6056,
	0x6F12,0x4146,
	0x6F12,0x9385,
	0x6F12,0x0403,
	0x6F12,0x0818,
	0x6F12,0x9740,
	0x6F12,0x01FC,
	0x6F12,0xE780,
	0x6F12,0x6055,
	0x6F12,0x1C40,
	0x6F12,0xB7DB,
	0x6F12,0x0040,
	0x6F12,0x014B,
	0x6F12,0xBEC0,
	0x6F12,0x5C40,
	0x6F12,0x8149,
	0x6F12,0x854A,
	0x6F12,0xBEC2,
	0x6F12,0x5C44,
	0x6F12,0x096D,
	0x6F12,0x370C,
	0x6F12,0x0040,
	0x6F12,0xBEC4,
	0x6F12,0x1C48,
	0x6F12,0x938B,
	0x6F12,0x0B03,
	0x6F12,0x930C,
	0x6F12,0x0004,
	0x6F12,0xBEC6,
	0x6F12,0x0347,
	0x6F12,0x7401,
	0x6F12,0x8967,
	0x6F12,0x631B,
	0x6F12,0x5701,
	0x6F12,0x93F7,
	0x6F12,0xF900,
	0x6F12,0x9808,
	0x6F12,0xBA97,
	0x6F12,0x83C7,
	0x6F12,0x07FB,
	0x6F12,0x8A07,
	0x6F12,0xCA97,
	0x6F12,0x9C43,
	0x6F12,0x63DE,
	0x6F12,0x3A01,
	0x6F12,0x1387,
	0x6F12,0x69FE,
	0x6F12,0x63FA,
	0x6F12,0xEA00,
	0x6F12,0x1387,
	0x6F12,0xA9FD,
	0x6F12,0x63F6,
	0x6F12,0xEA00,
	0x6F12,0x1387,
	0x6F12,0x49FC,
	0x6F12,0x63E3,
	0x6F12,0xEA0A,
	0x6F12,0x131A,
	0x6F12,0x1B00,
	0x6F12,0x9808,
	0x6F12,0x5297,
	0x6F12,0x8354,
	0x6F12,0x07FF,
	0x6F12,0x0145,
	0x6F12,0xB384,
	0x6F12,0xF402,
	0x6F12,0xB580,
	0x6F12,0x637A,
	0x6F12,0x9D00,
	0x6F12,0x2685,
	0x6F12,0x9780,
	0x6F12,0xFFFB,
	0x6F12,0xE780,
	0x6F12,0x6052,
	0x6F12,0x4915,
	0x6F12,0x1375,
	0x6F12,0xF50F,
	0x6F12,0x9C08,
	0x6F12,0xD297,
	0x6F12,0x03D7,
	0x6F12,0x07FE,
	0x6F12,0xB3D4,
	0x6F12,0xA400,
	0x6F12,0xC204,
	0x6F12,0x6297,
	0x6F12,0xC180,
	0x6F12,0x2310,
	0x6F12,0x9700,
	0x6F12,0x03D7,
	0x6F12,0x07FC,
	0x6F12,0x83D7,
	0x6F12,0x07FD,
	0x6F12,0x050B,
	0x6F12,0x6297,
	0x6F12,0x8356,
	0x6F12,0x0700,
	0x6F12,0x3315,
	0x6F12,0xF500,
	0x6F12,0x558D,
	0x6F12,0x4205,
	0x6F12,0x4181,
	0x6F12,0x2310,
	0x6F12,0xA700,
	0x6F12,0x8509,
	0x6F12,0xE395,
	0x6F12,0x99F7,
	0x6F12,0x8D47,
	0x6F12,0x37D4,
	0x6F12,0x0040,
	0x6F12,0x2319,
	0x6F12,0xF40A,
	0x6F12,0x2316,
	0x6F12,0x040C,
	0x6F12,0x9790,
	0x6F12,0x00FC,
	0x6F12,0xE780,
	0x6F12,0x6093,
	0x6F12,0xAA84,
	0x6F12,0x9790,
	0x6F12,0x00FC,
	0x6F12,0xE780,
	0x6F12,0x2092,
	0x6F12,0x9307,
	0x6F12,0x0008,
	0x6F12,0x33D5,
	0x6F12,0xA700,
	0x6F12,0x9307,
	0x6F12,0x0004,
	0x6F12,0x1205,
	0x6F12,0xB3D7,
	0x6F12,0x9700,
	0x6F12,0x1375,
	0x6F12,0x0503,
	0x6F12,0x8D8B,
	0x6F12,0x5D8D,
	0x6F12,0x2319,
	0x6F12,0xA40C,
	0x6F12,0x45B5,
	0x6F12,0x1397,
	0x6F12,0x1900,
	0x6F12,0x9394,
	0x6F12,0x0701,
	0x6F12,0x5E97,
	0x6F12,0xC180,
	0x6F12,0x2310,
	0x6F12,0x9700,
	0x6F12,0x6DB7,
	0x6F12,0x0347,
	0x6F12,0x6401,
	0x6F12,0xE315,
	0x6F12,0xF7FA,
	0x6F12,0xB784,
	0x6F12,0x0024,
	0x6F12,0x9384,
	0x6F12,0x04F8,
	0x6F12,0x4146,
	0x6F12,0x9385,
	0x6F12,0x0404,
	0x6F12,0x0A85,
	0x6F12,0x9740,
	0x6F12,0x01FC,
	0x6F12,0xE780,
	0x6F12,0x2042,
	0x6F12,0x4146,
	0x6F12,0x9385,
	0x6F12,0x0405,
	0x6F12,0x0808,
	0x6F12,0x9740,
	0x6F12,0x01FC,
	0x6F12,0xE780,
	0x6F12,0x2041,
	0x6F12,0x4146,
	0x6F12,0x9385,
	0x6F12,0x0406,
	0x6F12,0x0810,
	0x6F12,0x9740,
	0x6F12,0x01FC,
	0x6F12,0xE780,
	0x6F12,0x2040,
	0x6F12,0x4146,
	0x6F12,0x9385,
	0x6F12,0x0407,
	0x6F12,0x0818,
	0x6F12,0x9740,
	0x6F12,0x01FC,
	0x6F12,0xE780,
	0x6F12,0x203F,
	0x6F12,0x0357,
	0x6F12,0x0400,
	0x6F12,0x8357,
	0x6F12,0x2400,
	0x6F12,0x37DB,
	0x6F12,0x0040,
	0x6F12,0x2310,
	0x6F12,0xE104,
	0x6F12,0x2311,
	0x6F12,0xF104,
	0x6F12,0x2312,
	0x6F12,0xE104,
	0x6F12,0x2313,
	0x6F12,0xF104,
	0x6F12,0x0357,
	0x6F12,0x8400,
	0x6F12,0x8357,
	0x6F12,0xA400,
	0x6F12,0x814A,
	0x6F12,0x2314,
	0x6F12,0xE104,
	0x6F12,0x2315,
	0x6F12,0xF104,
	0x6F12,0x2316,
	0x6F12,0xE104,
	0x6F12,0x2317,
	0x6F12,0xF104,
	0x6F12,0x8149,
	0x6F12,0x854B,
	0x6F12,0x096D,
	0x6F12,0x130B,
	0x6F12,0x0B03,
	0x6F12,0x370C,
	0x6F12,0x0040,
	0x6F12,0x930C,
	0x6F12,0x0004,
	0x6F12,0x0347,
	0x6F12,0x7401,
	0x6F12,0x8967,
	0x6F12,0x631B,
	0x6F12,0x7701,
	0x6F12,0x93F7,
	0x6F12,0xF900,
	0x6F12,0x9808,
	0x6F12,0xBA97,
	0x6F12,0x83C7,
	0x6F12,0x07FB,
	0x6F12,0x8A07,
	0x6F12,0xCA97,
	0x6F12,0x9C43,
	0x6F12,0x63C4,
	0x6F12,0x3B07,
	0x6F12,0x139A,
	0x6F12,0x1A00,
	0x6F12,0x9808,
	0x6F12,0x5297,
	0x6F12,0x8354,
	0x6F12,0x07FF,
	0x6F12,0x0145,
	0x6F12,0xB384,
	0x6F12,0xF402,
	0x6F12,0xB580,
	0x6F12,0x637A,
	0x6F12,0x9D00,
	0x6F12,0x2685,
	0x6F12,0x9780,
	0x6F12,0xFFFB,
	0x6F12,0xE780,
	0x6F12,0xA03B,
	0x6F12,0x4915,
	0x6F12,0x1375,
	0x6F12,0xF50F,
	0x6F12,0x9C08,
	0x6F12,0xD297,
	0x6F12,0x03D7,
	0x6F12,0x07FE,
	0x6F12,0xB3D4,
	0x6F12,0xA400,
	0x6F12,0xC204,
	0x6F12,0x6297,
	0x6F12,0xC180,
	0x6F12,0x2310,
	0x6F12,0x9700,
	0x6F12,0x03D7,
	0x6F12,0x07FC,
	0x6F12,0x83D7,
	0x6F12,0x07FD,
	0x6F12,0x850A,
	0x6F12,0x6297,
	0x6F12,0x8356,
	0x6F12,0x0700,
	0x6F12,0x3315,
	0x6F12,0xF500,
	0x6F12,0x558D,
	0x6F12,0x4205,
	0x6F12,0x4181,
	0x6F12,0x2310,
	0x6F12,0xA700,
	0x6F12,0x8509,
	0x6F12,0xE391,
	0x6F12,0x99F9,
	0x6F12,0x51BD,
	0x6F12,0x1397,
	0x6F12,0x1900,
	0x6F12,0x9394,
	0x6F12,0x0701,
	0x6F12,0x5A97,
	0x6F12,0xC180,
	0x6F12,0x2310,
	0x6F12,0x9700,
	0x6F12,0xE5B7,
	0x6F12,0x1743,
	0x6F12,0x01FC,
	0x6F12,0xE702,
	0x6F12,0xE326,
	0x6F12,0xB737,
	0x6F12,0x0024,
	0x6F12,0x83A7,
	0x6F12,0x0761,
	0x6F12,0xAA84,
	0x6F12,0x2E89,
	0x6F12,0x8297,
	0x6F12,0xB787,
	0x6F12,0x0024,
	0x6F12,0x03A4,
	0x6F12,0x0700,
	0x6F12,0x0146,
	0x6F12,0x1145,
	0x6F12,0xA285,
	0x6F12,0x9780,
	0x6F12,0xFFFB,
	0x6F12,0xE780,
	0x6F12,0xC069,
	0x6F12,0xCA85,
	0x6F12,0x2685,
	0x6F12,0x9730,
	0x6F12,0x00FC,
	0x6F12,0xE780,
	0x6F12,0x20F5,
	0x6F12,0x0546,
	0x6F12,0xA285,
	0x6F12,0x1145,
	0x6F12,0x9780,
	0x6F12,0xFFFB,
	0x6F12,0xE780,
	0x6F12,0x2068,
	0x6F12,0x1743,
	0x6F12,0x01FC,
	0x6F12,0x6700,
	0x6F12,0xC324,
	0x6F12,0xB717,
	0x6F12,0x0024,
	0x6F12,0x83C7,
	0x6F12,0x0734,
	0x6F12,0xEDCF,
	0x6F12,0x1743,
	0x6F12,0x01FC,
	0x6F12,0xE702,
	0x6F12,0x6321,
	0x6F12,0x9780,
	0x6F12,0x00FC,
	0x6F12,0xE780,
	0x6F12,0x803E,
	0x6F12,0xB787,
	0x6F12,0x0024,
	0x6F12,0x1387,
	0x6F12,0x87F5,
	0x6F12,0x0347,
	0x6F12,0xF701,
	0x6F12,0x9387,
	0x6F12,0x87F5,
	0x6F12,0x19EB,
	0x6F12,0x37F7,
	0x6F12,0x0040,
	0x6F12,0x8356,
	0x6F12,0x6772,
	0x6F12,0x2391,
	0x6F12,0xD702,
	0x6F12,0x0357,
	0x6F12,0xA772,
	0x6F12,0x2392,
	0x6F12,0xE702,
	0x6F12,0xB776,
	0x6F12,0x0024,
	0x6F12,0x83C6,
	0x6F12,0xB6F1,
	0x6F12,0x0547,
	0x6F12,0xA38F,
	0x6F12,0xE700,
	0x6F12,0x99C6,
	0x6F12,0x83D6,
	0x6F12,0x4701,
	0x6F12,0x238F,
	0x6F12,0xE700,
	0x6F12,0x2380,
	0x6F12,0xD702,
	0x6F12,0x83C6,
	0x6F12,0x0702,
	0x6F12,0x03C7,
	0x6F12,0xE701,
	0x6F12,0xB9CE,
	0x6F12,0x0DC3,
	0x6F12,0x03D7,
	0x6F12,0x6701,
	0x6F12,0x0DCF,
	0x6F12,0xB7F6,
	0x6F12,0x0040,
	0x6F12,0x2393,
	0x6F12,0xE672,
	0x6F12,0x03D7,
	0x6F12,0x8701,
	0x6F12,0x0DCF,
	0x6F12,0xB7F6,
	0x6F12,0x0040,
	0x6F12,0x2395,
	0x6F12,0xE672,
	0x6F12,0x238F,
	0x6F12,0x0700,
	0x6F12,0x03C7,
	0x6F12,0x0702,
	0x6F12,0x7D17,
	0x6F12,0x1377,
	0x6F12,0xF70F,
	0x6F12,0x2380,
	0x6F12,0xE702,
	0x6F12,0x01E7,
	0x6F12,0x0547,
	0x6F12,0x238F,
	0x6F12,0xE700,
	0x6F12,0x1743,
	0x6F12,0x01FC,
	0x6F12,0x6700,
	0x6F12,0x631A,
	0x6F12,0x83D6,
	0x6F12,0x2702,
	0x6F12,0x37F7,
	0x6F12,0x0040,
	0x6F12,0x2313,
	0x6F12,0xD772,
	0x6F12,0xD1B7,
	0x6F12,0x83D6,
	0x6F12,0x4702,
	0x6F12,0x37F7,
	0x6F12,0x0040,
	0x6F12,0x2315,
	0x6F12,0xD772,
	0x6F12,0xD1B7,
	0x6F12,0x71DF,
	0x6F12,0x03D7,
	0x6F12,0xA701,
	0x6F12,0x19CF,
	0x6F12,0xB7F6,
	0x6F12,0x0040,
	0x6F12,0x2393,
	0x6F12,0xE672,
	0x6F12,0x03D7,
	0x6F12,0xC701,
	0x6F12,0x19CF,
	0x6F12,0xB7F6,
	0x6F12,0x0040,
	0x6F12,0x2395,
	0x6F12,0xE672,
	0x6F12,0x238F,
	0x6F12,0x0700,
	0x6F12,0x6DBF,
	0x6F12,0x83D6,
	0x6F12,0x2702,
	0x6F12,0x37F7,
	0x6F12,0x0040,
	0x6F12,0x2313,
	0x6F12,0xD772,
	0x6F12,0xC5B7,
	0x6F12,0x83D6,
	0x6F12,0x4702,
	0x6F12,0x37F7,
	0x6F12,0x0040,
	0x6F12,0x2315,
	0x6F12,0xD772,
	0x6F12,0xC5B7,
	0x6F12,0x8280,
	0x6F12,0x1743,
	0x6F12,0x01FC,
	0x6F12,0xE702,
	0x6F12,0xC311,
	0x6F12,0xB787,
	0x6F12,0x0024,
	0x6F12,0xA166,
	0x6F12,0xB775,
	0x6F12,0x0024,
	0x6F12,0x9386,
	0x6F12,0x76F7,
	0x6F12,0x3777,
	0x6F12,0x0024,
	0x6F12,0x9387,
	0x6F12,0xC701,
	0x6F12,0x2946,
	0x6F12,0x9385,
	0x6F12,0xA57F,
	0x6F12,0x3545,
	0x6F12,0x2320,
	0x6F12,0xF73C,
	0x6F12,0x9710,
	0x6F12,0x00FC,
	0x6F12,0xE780,
	0x6F12,0xA01F,
	0x6F12,0xB777,
	0x6F12,0x0024,
	0x6F12,0xB785,
	0x6F12,0x0024,
	0x6F12,0x3755,
	0x6F12,0x0020,
	0x6F12,0x3737,
	0x6F12,0x0024,
	0x6F12,0x9387,
	0x6F12,0x4774,
	0x6F12,0x0146,
	0x6F12,0x9385,
	0x6F12,0xA58B,
	0x6F12,0x1305,
	0x6F12,0x0537,
	0x6F12,0x232C,
	0x6F12,0xF75E,
	0x6F12,0x9710,
	0x6F12,0x00FC,
	0x6F12,0xE780,
	0x6F12,0x4058,
	0x6F12,0xB787,
	0x6F12,0x0024,
	0x6F12,0x23A6,
	0x6F12,0xA700,
	0x6F12,0xB785,
	0x6F12,0x0024,
	0x6F12,0x3765,
	0x6F12,0x0020,
	0x6F12,0x0146,
	0x6F12,0x9385,
	0x6F12,0xE59F,
	0x6F12,0x1305,
	0x6F12,0x45B2,
	0x6F12,0x9710,
	0x6F12,0x00FC,
	0x6F12,0xE780,
	0x6F12,0x2056,
	0x6F12,0xB787,
	0x6F12,0x0024,
	0x6F12,0x23A2,
	0x6F12,0xA700,
	0x6F12,0xB785,
	0x6F12,0x0024,
	0x6F12,0x3755,
	0x6F12,0x0020,
	0x6F12,0x0146,
	0x6F12,0x9385,
	0x6F12,0x2597,
	0x6F12,0x1305,
	0x6F12,0x0525,
	0x6F12,0x9710,
	0x6F12,0x00FC,
	0x6F12,0xE780,
	0x6F12,0x0054,
	0x6F12,0xB787,
	0x6F12,0x0024,
	0x6F12,0x23A4,
	0x6F12,0xA700,
	0x6F12,0xB775,
	0x6F12,0x0024,
	0x6F12,0x3775,
	0x6F12,0x0020,
	0x6F12,0x0146,
	0x6F12,0x9385,
	0x6F12,0x257B,
	0x6F12,0x1305,
	0x6F12,0x85D3,
	0x6F12,0x9710,
	0x6F12,0x00FC,
	0x6F12,0xE780,
	0x6F12,0xE051,
	0x6F12,0xB787,
	0x6F12,0x0024,
	0x6F12,0x23A8,
	0x6F12,0xA700,
	0x6F12,0xB785,
	0x6F12,0x0024,
	0x6F12,0x37B5,
	0x6F12,0x0020,
	0x6F12,0x0146,
	0x6F12,0x9385,
	0x6F12,0x85CE,
	0x6F12,0x1305,
	0x6F12,0xA5C6,
	0x6F12,0x9710,
	0x6F12,0x00FC,
	0x6F12,0xE780,
	0x6F12,0xC04F,
	0x6F12,0xB787,
	0x6F12,0x0024,
	0x6F12,0x23A0,
	0x6F12,0xA700,
	0x6F12,0xB787,
	0x6F12,0x0024,
	0x6F12,0x3737,
	0x6F12,0x0024,
	0x6F12,0x9387,
	0x6F12,0x67D3,
	0x6F12,0x2320,
	0x6F12,0xF768,
	0x6F12,0x1743,
	0x6F12,0x01FC,
	0x6F12,0x6700,
	0x6F12,0x4304,
	0x6F12,0x0000,
	0x6F12,0x0020,
	0x6F12,0x0020,
	0x6F12,0x0020,
	0x6F12,0x0020,
	0x6F12,0x0020,
	0x6F12,0x0020,
	0x6F12,0x0020,
	0x6F12,0x0020,
	0x6F12,0x0020,
	0x6F12,0x0020,
	0x6F12,0x0000,
	0x6F12,0x0000,
	0x6F12,0x0000,
	0x6F12,0x0000,
	0x6F12,0x0000,
	0x6F12,0x0000,
	0x6F12,0x0000,
	0x6F12,0x0000,
	0x6F12,0x0000,
	0x6F12,0x0000,
	0x6F12,0x0000,
	0x6F12,0x0000,
	0x6F12,0x0000,
	0x6F12,0x0000,
	0x6F12,0x0000,
	0x6F12,0x0000,
	0x6F12,0x0000,
	0x6F12,0x0000,
	0x6F12,0x0000,
	0x6F12,0x0000,
	0x6F12,0x0000,
	0x6F12,0x0000,
	0x6F12,0x0001,
	0x6F12,0x0203,
	0x6F12,0x0001,
	0x6F12,0x0203,
	0x6F12,0x0405,
	0x6F12,0x0607,
	0x6F12,0x0405,
	0x6F12,0x0607,
	0x6F12,0x10D0,
	0x6F12,0x10D0,
	0x6F12,0x1CD0,
	0x6F12,0x1CD0,
	0x6F12,0x22D0,
	0x6F12,0x22D0,
	0x6F12,0x2ED0,
	0x6F12,0x2ED0,
	0x6F12,0x0000,
	0x6F12,0x0400,
	0x6F12,0x0800,
	0x6F12,0x0C00,
	0x6F12,0x0800,
	0x6F12,0x0C00,
	0x6F12,0x0000,
	0x6F12,0x0400,
	0x6F12,0x30D0,
	0x6F12,0x32D0,
	0x6F12,0x64D0,
	0x6F12,0x66D0,
	0x6F12,0x7CD0,
	0x6F12,0x7ED0,
	0x6F12,0xA8D0,
	0x6F12,0xAAD0,
	0x6F12,0x0001,
	0x6F12,0x0001,
	0x6F12,0x0001,
	0x6F12,0x0001,
	0x6F12,0x0405,
	0x6F12,0x0405,
	0x6F12,0x0405,
	0x6F12,0x0405,
	0x6F12,0x10D0,
	0x6F12,0x10D0,
	0x6F12,0x12D0,
	0x6F12,0x12D0,
	0x6F12,0x20D0,
	0x6F12,0x20D0,
	0x6F12,0x22D0,
	0x6F12,0x22D0,
	0x6F12,0x0000,
	0x6F12,0x0400,
	0x6F12,0x0000,
	0x6F12,0x0400,
	0x6F12,0x0000,
	0x6F12,0x0400,
	0x6F12,0x0000,
	0x6F12,0x0400,
	0x6F12,0x30D0,
	0x6F12,0x32D0,
	0x6F12,0x38D0,
	0x6F12,0x3AD0,
	0x6F12,0x70D0,
	0x6F12,0x72D0,
	0x6F12,0x78D0,
	0x6F12,0x7AD0,
	0x6F12,0x0000,
	0x6F12,0x0000,
	0x6F12,0x0000,
	0x6F12,0x0000,
	0x6F12,0x0000,
	0x6F12,0x0000,
	0x6F12,0x0000,
	0x6F12,0x0000,
	0x6F12,0x0000,
	0x6F12,0x0000,
	0x6F12,0x0000,
	0x6F12,0x0000,
	0x6F12,0x0000,
	0x6F12,0x0000,
	0x6F12,0x1743,
	0x6F12,0x01FC,
	0x6F12,0xE702,
	0x6F12,0xA3F3,
	0x6F12,0xB787,
	0x6F12,0x0024,
	0x6F12,0x1307,
	0x6F12,0xC00E,
	0x6F12,0x9387,
	0x6F12,0xC701,
	0x6F12,0xB785,
	0x6F12,0x0024,
	0x6F12,0x3755,
	0x6F12,0x0020,
	0x6F12,0xBA97,
	0x6F12,0x0146,
	0x6F12,0x3777,
	0x6F12,0x0024,
	0x6F12,0x9385,
	0x6F12,0x4507,
	0x6F12,0x1305,
	0x6F12,0xC504,
	0x6F12,0x2320,
	0x6F12,0xF73C,
	0x6F12,0x9710,
	0x6F12,0x00FC,
	0x6F12,0xE780,
	0x6F12,0x603C,
	0x6F12,0xB787,
	0x6F12,0x0024,
	0x6F12,0x23A0,
	0x6F12,0xA710,
	0x6F12,0x4928,
	0x6F12,0xE177,
	0x6F12,0x3747,
	0x6F12,0x0024,
	0x6F12,0x9387,
	0x6F12,0x5776,
	0x6F12,0x2317,
	0x6F12,0xF782,
	0x6F12,0x1743,
	0x6F12,0x01FC,
	0x6F12,0x6700,
	0x6F12,0xE3F0,
	0x6F12,0x1743,
	0x6F12,0x01FC,
	0x6F12,0xE702,
	0x6F12,0x83EC,
	0x6F12,0xB787,
	0x6F12,0x0024,
	0x6F12,0x83A4,
	0x6F12,0x0710,
	0x6F12,0xAA89,
	0x6F12,0x2E8A,
	0x6F12,0x0146,
	0x6F12,0xA685,
	0x6F12,0x1145,
	0x6F12,0x9780,
	0x6F12,0xFFFB,
	0x6F12,0xE780,
	0x6F12,0xA031,
	0x6F12,0xB787,
	0x6F12,0x0024,
	0x6F12,0x03C7,
	0x6F12,0x4710,
	0x6F12,0x3E84,
	0x6F12,0x0149,
	0x6F12,0x11CF,
	0x6F12,0x3767,
	0x6F12,0x0024,
	0x6F12,0x0357,
	0x6F12,0x2777,
	0x6F12,0xB777,
	0x6F12,0x0024,
	0x6F12,0x9387,
	0x6F12,0x07C7,
	0x6F12,0x0E07,
	0x6F12,0x03D9,
	0x6F12,0x871C,
	0x6F12,0x2394,
	0x6F12,0xE71C,
	0x6F12,0xD285,
	0x6F12,0x4E85,
	0x6F12,0x97D0,
	0x6F12,0xFFFB,
	0x6F12,0xE780,
	0x6F12,0xA0F8,
	0x6F12,0x8347,
	0x6F12,0x4410,
	0x6F12,0x89C7,
	0x6F12,0xB777,
	0x6F12,0x0024,
	0x6F12,0x239C,
	0x6F12,0x27E3,
	0x6F12,0x0546,
	0x6F12,0xA685,
	0x6F12,0x1145,
	0x6F12,0x9780,
	0x6F12,0xFFFB,
	0x6F12,0xE780,
	0x6F12,0xA02C,
	0x6F12,0x1743,
	0x6F12,0x01FC,
	0x6F12,0x6700,
	0x6F12,0xA3E8,
	0x6F12,0xE177,
	0x6F12,0x3747,
	0x6F12,0x0024,
	0x6F12,0x9387,
	0x6F12,0x5776,
	0x6F12,0x2318,
	0x6F12,0xF782,
	0x6F12,0x8280,
	0x6F12,0x0000,
	0x6F12,0x0000,
	0x6F12,0x0000,
	0x6F12,0x0000,
	0x6F12,0x0000,
	0x602A,0x35CC,
	0x6F12,0x1C80,
	0x6F12,0x0024,
	0x6028,0x2400,
	0x602A,0x1354,
	0x6F12,0x0100,
	0x6F12,0x7017,
	0x602A,0x13B2,
	0x6F12,0x0000,
	0x602A,0x1236,
	0x6F12,0x0000,
	0x602A,0x1A0A,
	0x6F12,0x4C0A,
	0x602A,0x2210,
	0x6F12,0x3401,
	0x602A,0x2176,
	0x6F12,0x6400,
	0x602A,0x222E,
	0x6F12,0x0001,
	0x602A,0x06B6,
	0x6F12,0x0A00,
	0x602A,0x06BC,
	0x6F12,0x1001,
	0x602A,0x2140,
	0x6F12,0x0101,
	0x602A,0x218E,
	0x6F12,0x0000,
	0x602A,0x1A0E,
	0x6F12,0x9600,
	0x6028,0x4000,
	0xF44E,0x0011,
	0xF44C,0x0B0B,
	0xF44A,0x0006,
	0x0118,0x0002,
	0x011A,0x0001,	/*	sensor_init_setting_array  */
 };

static kal_uint16 addr_data_pair_preview[] = {
    0x602A,0x1A28,
	0x6028,0x2400,
	0x6F12,0x4C00,
	0x602A,0x065A,
	0x6F12,0x0000,
	0x602A,0x139E,
	0x6F12,0x0100,
	0x602A,0x139C,
	0x6F12,0x0000,
	0x602A,0x13A0,
	0x6F12,0x0A00,
	0x6F12,0x0120,
	0x602A,0x2072,
	0x6F12,0x0000,
	0x602A,0x1A64,
	0x6F12,0x0301,
	0x6F12,0xFF00,
	0x602A,0x19E6,
	0x6F12,0x0200,
	0x602A,0x1A30,
	0x6F12,0x3401,
	0x602A,0x19FC,
	0x6F12,0x0B00,
	0x602A,0x19F4,
	0x6F12,0x0606,
	0x602A,0x19F8,
	0x6F12,0x1010,
	0x602A,0x1B26,
	0x6F12,0x6F80,
	0x6F12,0xA060,
	0x602A,0x1A3C,
	0x6F12,0x6207,
	0x602A,0x1A48,
	0x6F12,0x6207,
	0x602A,0x1444,
	0x6F12,0x2000,
	0x6F12,0x2000,
	0x602A,0x144C,
	0x6F12,0x3F00,
	0x6F12,0x3F00,
	0x602A,0x7F6C,
	0x6F12,0x0100,
	0x6F12,0x2F00,
	0x6F12,0xFA00,
	0x6F12,0x2400,
	0x6F12,0xE500,
	0x602A,0x0650,
	0x6F12,0x0600,
	0x602A,0x0654,
	0x6F12,0x0000,
	0x602A,0x1A46,
	0x6F12,0xB000,
	0x602A,0x1A52,
	0x6F12,0xBF00,
	0x602A,0x0674,
	0x6F12,0x0500,
	0x6F12,0x0500,
	0x6F12,0x0500,
	0x6F12,0x0500,
	0x602A,0x0668,
	0x6F12,0x0800,
	0x6F12,0x0800,
	0x6F12,0x0800,
	0x6F12,0x0800,
	0x602A,0x0684,
	0x6F12,0x4001,
	0x602A,0x0688,
	0x6F12,0x4001,
	0x602A,0x147C,
	0x6F12,0x1000,
	0x602A,0x1480,
	0x6F12,0x1000,
	0x602A,0x19F6,
	0x6F12,0x0904,
	0x602A,0x0812,
	0x6F12,0x0010,
	0x602A,0x2148,
	0x6F12,0x0100,
	0x602A,0x2042,
	0x6F12,0x1A00,
	0x602A,0x0874,
	0x6F12,0x0100,
	0x602A,0x09C0,
	0x6F12,0x2008,
	0x602A,0x09C4,
	0x6F12,0x2000,
	0x602A,0x19FE,
	0x6F12,0x0E1C,
	0x602A,0x4D92,
	0x6F12,0x0100,
	0x602A,0x8104,
	0x6F12,0x0100,
	0x602A,0x4D94,
	0x6F12,0x0005,
	0x6F12,0x000A,
	0x6F12,0x0010,
	0x6F12,0x1510,
	0x6F12,0x000A,
	0x6F12,0x0040,
	0x6F12,0x1510,
	0x6F12,0x1510,
	0x602A,0x3570,
	0x6F12,0x0000,
	0x602A,0x3574,
	0x6F12,0xD803,
	0x602A,0x21E4,
	0x6F12,0x0400,
	0x602A,0x21EC,
	0x6F12,0x2A01,
	0x602A,0x2080,
	0x6F12,0x0100,
	0x6F12,0xFF00,
	0x602A,0x2086,
	0x6F12,0x0001,
	0x602A,0x208E,
	0x6F12,0x14F4,
	0x602A,0x208A,
	0x6F12,0xD244,
	0x6F12,0xD244,
	0x602A,0x120E,
	0x6F12,0x1000,
	0x602A,0x212E,
	0x6F12,0x0200,
	0x602A,0x13AE,
	0x6F12,0x0101,
	0x602A,0x0718,
	0x6F12,0x0001,
	0x602A,0x0710,
	0x6F12,0x0002,
	0x6F12,0x0804,
	0x6F12,0x0100,
	0x602A,0x1B5C,
	0x6F12,0x0000,
	0x602A,0x0786,
	0x6F12,0x7701,
	0x602A,0x2022,
	0x6F12,0x0500,
	0x6F12,0x0500,
	0x602A,0x1360,
	0x6F12,0x0100,
	0x602A,0x1376,
	0x6F12,0x0100,
	0x6F12,0x6038,
	0x6F12,0x7038,
	0x6F12,0x8038,
	0x602A,0x1386,
	0x6F12,0x0B00,
	0x602A,0x06FA,
	0x6F12,0x1000,
	0x602A,0x4A94,
	0x6F12,0x0600,
	0x6F12,0x0600,
	0x6F12,0x0600,
	0x6F12,0x0600,
	0x6F12,0x0600,
	0x6F12,0x0600,
	0x6F12,0x0600,
	0x6F12,0x0600,
	0x6F12,0x0600,
	0x6F12,0x0600,
	0x6F12,0x0600,
	0x6F12,0x0600,
	0x6F12,0x0600,
	0x6F12,0x0600,
	0x6F12,0x0600,
	0x6F12,0x0600,
	0x602A,0x0A76,
	0x6F12,0x1000,
	0x602A,0x0AEE,
	0x6F12,0x1000,
	0x602A,0x0B66,
	0x6F12,0x1000,
	0x602A,0x0BDE,
	0x6F12,0x1000,
	0x602A,0x0C56,
	0x6F12,0x1000,
	0x602A,0x0CF2,
	0x6F12,0x0001,
	0x602A,0x0CF0,
	0x6F12,0x0101,
	0x602A,0x11B8,
	0x6F12,0x0100,
	0x602A,0x11F6,
	0x6F12,0x0020,
	0x602A,0x4A74,
	0x6F12,0x0000,
	0x6F12,0x0000,
	0x6F12,0xD8FF,
	0x6F12,0x0000,
	0x6F12,0x0000,
	0x6F12,0x0000,
	0x6F12,0x0000,
	0x6F12,0x0000,
	0x6F12,0xD8FF,
	0x6F12,0x0000,
	0x6F12,0x0000,
	0x6F12,0x0000,
	0x6F12,0x0000,
	0x6F12,0x0000,
	0x6F12,0x0000,
	0x6F12,0x0000,
	0x6028,0x4000,
	0xF46A,0xAE80,
	0x0344,0x0000,
	0x0346,0x0000,
	0x0348,0x1FFF,
	0x034A,0x181F,
	0x034C,0x0FF0,
	0x034E,0x0C00,
	0x0350,0x0008,
	0x0352,0x0008,
	0x0900,0x0122,
	0x0380,0x0002,
	0x0382,0x0002,
	0x0384,0x0002,
	0x0386,0x0002,
	0x0110,0x1002,
	0x0114,0x0300,
	0x0116,0x3000,
	0x0136,0x1800,
	0x013E,0x0000,
	0x0300,0x0006,
	0x0302,0x0001,
	0x0304,0x0004,
	0x0306,0x008C,
	0x0308,0x0008,
	0x030A,0x0001,
	0x030C,0x0000,
	0x030E,0x0004,
	0x0310,0x008A,
	0x0312,0x0000,
	0x0340,0x0FE0,
	0x0342,0x11E8,
	0x0702,0x0000,
	0x0202,0x0100,
	0x0200,0x0100,
	0x0D00,0x0101,
	0x0D02,0x0001,
	0x0D04,0x0102,
	0x6226,0x0000,
};


static kal_uint16 addr_data_pair_capture[] = {
    0x6028,0x2400,
	0x602A,0x1A28,
	0x6F12,0x4C00,
	0x602A,0x065A,
	0x6F12,0x0000,
	0x602A,0x139E,
	0x6F12,0x0400,
	0x602A,0x139C,
	0x6F12,0x0100,
	0x602A,0x13A0,
	0x6F12,0x0500,
	0x6F12,0x0120,
	0x602A,0x2072,
	0x6F12,0x0101,
	0x602A,0x1A64,
	0x6F12,0x0001,
	0x6F12,0x0000,
	0x602A,0x19E6,
	0x6F12,0x0200,
	0x602A,0x1A30,
	0x6F12,0x3403,
	0x602A,0x19FC,
	0x6F12,0x0700,
	0x602A,0x19F4,
	0x6F12,0x0707,
	0x602A,0x19F8,
	0x6F12,0x0B0B,
	0x602A,0x1B26,
	0x6F12,0x6F80,
	0x6F12,0xA060,
	0x602A,0x1A3C,
	0x6F12,0x8207,
	0x602A,0x1A48,
	0x6F12,0x8207,
	0x602A,0x1444,
	0x6F12,0x2000,
	0x6F12,0x2000,
	0x602A,0x144C,
	0x6F12,0x3F00,
	0x6F12,0x3F00,
	0x602A,0x7F6C,
	0x6F12,0x0100,
	0x6F12,0x2F00,
	0x6F12,0xFA00,
	0x6F12,0x2400,
	0x6F12,0xE500,
	0x602A,0x0650,
	0x6F12,0x0600,
	0x602A,0x0654,
	0x6F12,0x0000,
	0x602A,0x1A46,
	0x6F12,0x9800,
	0x602A,0x1A52,
	0x6F12,0x9800,
	0x602A,0x0674,
	0x6F12,0x0500,
	0x6F12,0x0500,
	0x6F12,0x0500,
	0x6F12,0x0500,
	0x602A,0x0668,
	0x6F12,0x0800,
	0x6F12,0x0800,
	0x6F12,0x0800,
	0x6F12,0x0800,
	0x602A,0x0684,
	0x6F12,0x4001,
	0x602A,0x0688,
	0x6F12,0x4001,
	0x602A,0x147C,
	0x6F12,0x0400,
	0x602A,0x1480,
	0x6F12,0x0400,
	0x602A,0x19F6,
	0x6F12,0x0404,
	0x602A,0x0812,
	0x6F12,0x0010,
	0x602A,0x2148,
	0x6F12,0x0100,
	0x602A,0x2042,
	0x6F12,0x1A00,
	0x602A,0x0874,
	0x6F12,0x0106,
	0x602A,0x09C0,
	0x6F12,0x4000,
	0x602A,0x09C4,
	0x6F12,0x4000,
	0x602A,0x19FE,
	0x6F12,0x0C1C,
	0x602A,0x4D92,
	0x6F12,0x0000,
	0x602A,0x8104,
	0x6F12,0x0000,
	0x602A,0x4D94,
	0x6F12,0x0000,
	0x6F12,0x0000,
	0x6F12,0x0000,
	0x6F12,0x0000,
	0x6F12,0x0000,
	0x6F12,0x0000,
	0x6F12,0x0000,
	0x6F12,0x0000,
	0x602A,0x3570,
	0x6F12,0x0000,
	0x602A,0x3574,
	0x6F12,0x4B08,
	0x602A,0x21E4,
	0x6F12,0x0400,
	0x602A,0x21EC,
	0x6F12,0x9100,
	0x602A,0x2080,
	0x6F12,0x0100,
	0x6F12,0xFF00,
	0x602A,0x2086,
	0x6F12,0x0001,
	0x602A,0x208E,
	0x6F12,0x14F4,
	0x602A,0x208A,
	0x6F12,0xD244,
	0x6F12,0xD244,
	0x602A,0x120E,
	0x6F12,0x1000,
	0x602A,0x212E,
	0x6F12,0x0200,
	0x602A,0x13AE,
	0x6F12,0x0100,
	0x602A,0x0718,
	0x6F12,0x0000,
	0x602A,0x0710,
	0x6F12,0x0010,
	0x6F12,0x0201,
	0x6F12,0x0800,
	0x602A,0x1B5C,
	0x6F12,0x0000,
	0x602A,0x0786,
	0x6F12,0x1401,
	0x602A,0x2022,
	0x6F12,0x0500,
	0x6F12,0x0500,
	0x602A,0x1360,
	0x6F12,0x0000,
	0x602A,0x1376,
	0x6F12,0x0000,
	0x6F12,0x6038,
	0x6F12,0x7038,
	0x6F12,0x8038,
	0x602A,0x1386,
	0x6F12,0x0B00,
	0x602A,0x06FA,
	0x6F12,0x1000,
	0x602A,0x4A94,
	0x6F12,0x0600,
	0x6F12,0x0600,
	0x6F12,0x0600,
	0x6F12,0x0600,
	0x6F12,0x0600,
	0x6F12,0x0600,
	0x6F12,0x0600,
	0x6F12,0x0600,
	0x6F12,0x0600,
	0x6F12,0x0600,
	0x6F12,0x0600,
	0x6F12,0x0600,
	0x6F12,0x0600,
	0x6F12,0x0600,
	0x6F12,0x0600,
	0x6F12,0x0600,
	0x602A,0x0A76,
	0x6F12,0x1000,
	0x602A,0x0AEE,
	0x6F12,0x1000,
	0x602A,0x0B66,
	0x6F12,0x1000,
	0x602A,0x0BDE,
	0x6F12,0x1000,
	0x602A,0x0C56,
	0x6F12,0x1000,
	0x602A,0x0CF2,
	0x6F12,0x0001,
	0x602A,0x0CF0,
	0x6F12,0x0101,
	0x602A,0x11B8,
	0x6F12,0x0000,
	0x602A,0x11F6,
	0x6F12,0x0010,
	0x602A,0x4A74,
	0x6F12,0x0000,
	0x6F12,0x0000,
	0x6F12,0x0000,
	0x6F12,0x0000,
	0x6F12,0x0000,
	0x6F12,0x0000,
	0x6F12,0x0000,
	0x6F12,0x0000,
	0x6F12,0x0000,
	0x6F12,0x0000,
	0x6F12,0x0000,
	0x6F12,0x0000,
	0x6F12,0x0000,
	0x6F12,0x0000,
	0x6F12,0x0000,
	0x6F12,0x0000,
	0x6028,0x4000,
	0xF46A,0xAE80,
	0x0344,0x0000,
	0x0346,0x0000,
	0x0348,0x1FFF,
	0x034A,0x181F,
	0x034C,0x1FE0,
	0x034E,0x1800,
	0x0350,0x0010,
	0x0352,0x0010,
	0x0900,0x0111,
	0x0380,0x0001,
	0x0382,0x0001,
	0x0384,0x0001,
	0x0386,0x0001,
	0x0110,0x1002,
	0x0114,0x0300,
	0x0116,0x3000,
	0x0136,0x1800,
	0x013E,0x0000,
	0x0300,0x0006,
	0x0302,0x0001,
	0x0304,0x0004,
	0x0306,0x008C,
	0x0308,0x0008,
	0x030A,0x0001,
	0x030C,0x0000,
	0x030E,0x0004,
	0x0310,0x0074,
	0x0312,0x0000,
	0x080E,0x0000,
	0x0340,0x1900,
	0x0342,0x21F0,
	0x0702,0x0000,
	0x0202,0x0100,
	0x0200,0x0100,
	0x0D00,0x0100,
	0x0D02,0x0001,
	0x0D04,0x0002,
	0x6226,0x0000,
};


static kal_uint16 addr_data_pair_normal_video[] = {
    0x602A,0x1A28,
	0x6028,0x2400,
	0x6F12,0x4C00,
	0x602A,0x065A,
	0x6F12,0x0000,
	0x602A,0x139E,
	0x6F12,0x0100,
	0x602A,0x139C,
	0x6F12,0x0000,
	0x602A,0x13A0,
	0x6F12,0x0A00,
	0x6F12,0x0120,
	0x602A,0x2072,
	0x6F12,0x0000,
	0x602A,0x1A64,
	0x6F12,0x0301,
	0x6F12,0xFF00,
	0x602A,0x19E6,
	0x6F12,0x0200,
	0x602A,0x1A30,
	0x6F12,0x3401,
	0x602A,0x19FC,
	0x6F12,0x0B00,
	0x602A,0x19F4,
	0x6F12,0x0606,
	0x602A,0x19F8,
	0x6F12,0x1010,
	0x602A,0x1B26,
	0x6F12,0x6F80,
	0x6F12,0xA060,
	0x602A,0x1A3C,
	0x6F12,0x6207,
	0x602A,0x1A48,
	0x6F12,0x6207,
	0x602A,0x1444,
	0x6F12,0x2000,
	0x6F12,0x2000,
	0x602A,0x144C,
	0x6F12,0x3F00,
	0x6F12,0x3F00,
	0x602A,0x7F6C,
	0x6F12,0x0100,
	0x6F12,0x2F00,
	0x6F12,0xFA00,
	0x6F12,0x2400,
	0x6F12,0xE500,
	0x602A,0x0650,
	0x6F12,0x0600,
	0x602A,0x0654,
	0x6F12,0x0000,
	0x602A,0x1A46,
	0x6F12,0xB000,
	0x602A,0x1A52,
	0x6F12,0xBF00,
	0x602A,0x0674,
	0x6F12,0x0500,
	0x6F12,0x0500,
	0x6F12,0x0500,
	0x6F12,0x0500,
	0x602A,0x0668,
	0x6F12,0x0800,
	0x6F12,0x0800,
	0x6F12,0x0800,
	0x6F12,0x0800,
	0x602A,0x0684,
	0x6F12,0x4001,
	0x602A,0x0688,
	0x6F12,0x4001,
	0x602A,0x147C,
	0x6F12,0x1000,
	0x602A,0x1480,
	0x6F12,0x1000,
	0x602A,0x19F6,
	0x6F12,0x0904,
	0x602A,0x0812,
	0x6F12,0x0010,
	0x602A,0x2148,
	0x6F12,0x0100,
	0x602A,0x2042,
	0x6F12,0x1A00,
	0x602A,0x0874,
	0x6F12,0x0100,
	0x602A,0x09C0,
	0x6F12,0x2008,
	0x602A,0x09C4,
	0x6F12,0x2000,
	0x602A,0x19FE,
	0x6F12,0x0E1C,
	0x602A,0x4D92,
	0x6F12,0x0100,
	0x602A,0x8104,
	0x6F12,0x0100,
	0x602A,0x4D94,
	0x6F12,0x0005,
	0x6F12,0x000A,
	0x6F12,0x0010,
	0x6F12,0x1510,
	0x6F12,0x000A,
	0x6F12,0x0040,
	0x6F12,0x1510,
	0x6F12,0x1510,
	0x602A,0x3570,
	0x6F12,0x0000,
	0x602A,0x3574,
	0x6F12,0xD803,
	0x602A,0x21E4,
	0x6F12,0x0400,
	0x602A,0x21EC,
	0x6F12,0x2A01,
	0x602A,0x2080,
	0x6F12,0x0100,
	0x6F12,0xFF00,
	0x602A,0x2086,
	0x6F12,0x0001,
	0x602A,0x208E,
	0x6F12,0x14F4,
	0x602A,0x208A,
	0x6F12,0xD244,
	0x6F12,0xD244,
	0x602A,0x120E,
	0x6F12,0x1000,
	0x602A,0x212E,
	0x6F12,0x0200,
	0x602A,0x13AE,
	0x6F12,0x0101,
	0x602A,0x0718,
	0x6F12,0x0001,
	0x602A,0x0710,
	0x6F12,0x0002,
	0x6F12,0x0804,
	0x6F12,0x0100,
	0x602A,0x1B5C,
	0x6F12,0x0000,
	0x602A,0x0786,
	0x6F12,0x7701,
	0x602A,0x2022,
	0x6F12,0x0500,
	0x6F12,0x0500,
	0x602A,0x1360,
	0x6F12,0x0100,
	0x602A,0x1376,
	0x6F12,0x0100,
	0x6F12,0x6038,
	0x6F12,0x7038,
	0x6F12,0x8038,
	0x602A,0x1386,
	0x6F12,0x0B00,
	0x602A,0x06FA,
	0x6F12,0x1000,
	0x602A,0x4A94,
	0x6F12,0x0600,
	0x6F12,0x0600,
	0x6F12,0x0600,
	0x6F12,0x0600,
	0x6F12,0x0600,
	0x6F12,0x0600,
	0x6F12,0x0600,
	0x6F12,0x0600,
	0x6F12,0x0600,
	0x6F12,0x0600,
	0x6F12,0x0600,
	0x6F12,0x0600,
	0x6F12,0x0600,
	0x6F12,0x0600,
	0x6F12,0x0600,
	0x6F12,0x0600,
	0x602A,0x0A76,
	0x6F12,0x1000,
	0x602A,0x0AEE,
	0x6F12,0x1000,
	0x602A,0x0B66,
	0x6F12,0x1000,
	0x602A,0x0BDE,
	0x6F12,0x1000,
	0x602A,0x0C56,
	0x6F12,0x1000,
	0x602A,0x0CF2,
	0x6F12,0x0001,
	0x602A,0x0CF0,
	0x6F12,0x0101,
	0x602A,0x11B8,
	0x6F12,0x0100,
	0x602A,0x11F6,
	0x6F12,0x0020,
	0x602A,0x4A74,
	0x6F12,0x0000,
	0x6F12,0x0000,
	0x6F12,0xD8FF,
	0x6F12,0x0000,
	0x6F12,0x0000,
	0x6F12,0x0000,
	0x6F12,0x0000,
	0x6F12,0x0000,
	0x6F12,0xD8FF,
	0x6F12,0x0000,
	0x6F12,0x0000,
	0x6F12,0x0000,
	0x6F12,0x0000,
	0x6F12,0x0000,
	0x6F12,0x0000,
	0x6F12,0x0000,
	0x6028,0x4000,
	0xF46A,0xAE80,
	0x0344,0x0000,
	0x0346,0x0000,
	0x0348,0x1FFF,
	0x034A,0x181F,
	0x034C,0x0FF0,
	0x034E,0x0C00,
	0x0350,0x0008,
	0x0352,0x0008,
	0x0900,0x0122,
	0x0380,0x0002,
	0x0382,0x0002,
	0x0384,0x0002,
	0x0386,0x0002,
	0x0110,0x1002,
	0x0114,0x0300,
	0x0116,0x3000,
	0x0136,0x1800,
	0x013E,0x0000,
	0x0300,0x0006,
	0x0302,0x0001,
	0x0304,0x0004,
	0x0306,0x008C,
	0x0308,0x0008,
	0x030A,0x0001,
	0x030C,0x0000,
	0x030E,0x0004,
	0x0310,0x008A,
	0x0312,0x0000,
	0x0340,0x0FE0,
	0x0342,0x11E8,
	0x0702,0x0000,
	0x0202,0x0100,
	0x0200,0x0100,
	0x0D00,0x0101,
	0x0D02,0x0001,
	0x0D04,0x0102,
	0x6226,0x0000,
};

static kal_uint16 addr_data_pair_hs_video[] = {
    0x6028,0x2400,
	0x602A,0x1A28,
	0x6F12,0x4C00,
	0x602A,0x065A,
	0x6F12,0x0000,
	0x602A,0x139E,
	0x6F12,0x0300,
	0x602A,0x139C,
	0x6F12,0x0000,
	0x602A,0x13A0,
	0x6F12,0x0A00,
	0x6F12,0x0020,
	0x602A,0x2072,
	0x6F12,0x0000,
	0x602A,0x1A64,
	0x6F12,0x0301,
	0x6F12,0xFF00,
	0x602A,0x19E6,
	0x6F12,0x0201,
	0x602A,0x1A30,
	0x6F12,0x3401,
	0x602A,0x19FC,
	0x6F12,0x0B00,
	0x602A,0x19F4,
	0x6F12,0x0606,
	0x602A,0x19F8,
	0x6F12,0x1010,
	0x602A,0x1B26,
	0x6F12,0x6F80,
	0x6F12,0xA020,
	0x602A,0x1A3C,
	0x6F12,0x5207,
	0x602A,0x1A48,
	0x6F12,0x5207,
	0x602A,0x1444,
	0x6F12,0x2100,
	0x6F12,0x2100,
	0x602A,0x144C,
	0x6F12,0x4200,
	0x6F12,0x4200,
	0x602A,0x7F6C,
	0x6F12,0x0100,
	0x6F12,0x3100,
	0x6F12,0xF700,
	0x6F12,0x2600,
	0x6F12,0xE100,
	0x602A,0x0650,
	0x6F12,0x0600,
	0x602A,0x0654,
	0x6F12,0x0000,
	0x602A,0x1A46,
	0x6F12,0x8900,
	0x602A,0x1A52,
	0x6F12,0xBF00,
	0x602A,0x0674,
	0x6F12,0x0500,
	0x6F12,0x0500,
	0x6F12,0x0500,
	0x6F12,0x0500,
	0x602A,0x0668,
	0x6F12,0x0800,
	0x6F12,0x0800,
	0x6F12,0x0800,
	0x6F12,0x0800,
	0x602A,0x0684,
	0x6F12,0x4001,
	0x602A,0x0688,
	0x6F12,0x4001,
	0x602A,0x147C,
	0x6F12,0x1000,
	0x602A,0x1480,
	0x6F12,0x1000,
	0x602A,0x19F6,
	0x6F12,0x0904,
	0x602A,0x0812,
	0x6F12,0x0010,
	0x602A,0x2148,
	0x6F12,0x0100,
	0x602A,0x2042,
	0x6F12,0x1A00,
	0x602A,0x0874,
	0x6F12,0x1100,
	0x602A,0x09C0,
	0x6F12,0x1803,
	0x602A,0x09C4,
	0x6F12,0x1803,
	0x602A,0x19FE,
	0x6F12,0x0E1C,
	0x602A,0x4D92,
	0x6F12,0x0000,
	0x602A,0x8104,
	0x6F12,0x0000,
	0x602A,0x4D94,
	0x6F12,0x0000,
	0x6F12,0x0000,
	0x6F12,0x0000,
	0x6F12,0x0000,
	0x6F12,0x0000,
	0x6F12,0x0000,
	0x6F12,0x0000,
	0x6F12,0x0000,
	0x602A,0x3570,
	0x6F12,0x0000,
	0x602A,0x3574,
	0x6F12,0x3801,
	0x602A,0x21E4,
	0x6F12,0x0400,
	0x602A,0x21EC,
	0x6F12,0x6801,
	0x602A,0x2080,
	0x6F12,0x0100,
	0x6F12,0xFF01,
	0x602A,0x2086,
	0x6F12,0x0002,
	0x602A,0x208E,
	0x6F12,0x14F4,
	0x602A,0x208A,
	0x6F12,0xC244,
	0x6F12,0xD244,
	0x602A,0x120E,
	0x6F12,0x1000,
	0x602A,0x212E,
	0x6F12,0x0A00,
	0x602A,0x13AE,
	0x6F12,0x0102,
	0x602A,0x0718,
	0x6F12,0x0005,
	0x602A,0x0710,
	0x6F12,0x0004,
	0x6F12,0x0401,
	0x6F12,0x0100,
	0x602A,0x1B5C,
	0x6F12,0x0300,
	0x602A,0x0786,
	0x6F12,0x7701,
	0x602A,0x2022,
	0x6F12,0x0101,
	0x6F12,0x0101,
	0x602A,0x1360,
	0x6F12,0x0100,
	0x602A,0x1376,
	0x6F12,0x0200,
	0x6F12,0x6038,
	0x6F12,0x7038,
	0x6F12,0x8038,
	0x602A,0x1386,
	0x6F12,0x0B00,
	0x602A,0x06FA,
	0x6F12,0x1000,
	0x602A,0x4A94,
	0x6F12,0x0600,
	0x6F12,0x0600,
	0x6F12,0x0600,
	0x6F12,0x0600,
	0x6F12,0x0600,
	0x6F12,0x0600,
	0x6F12,0x0600,
	0x6F12,0x0600,
	0x6F12,0x0600,
	0x6F12,0x0600,
	0x6F12,0x0600,
	0x6F12,0x0600,
	0x6F12,0x0600,
	0x6F12,0x0600,
	0x6F12,0x0600,
	0x6F12,0x0600,
	0x602A,0x0A76,
	0x6F12,0x1000,
	0x602A,0x0AEE,
	0x6F12,0x1000,
	0x602A,0x0B66,
	0x6F12,0x1000,
	0x602A,0x0BDE,
	0x6F12,0x1000,
	0x602A,0x0C56,
	0x6F12,0x1000,
	0x602A,0x0CF2,
	0x6F12,0x0001,
	0x602A,0x0CF0,
	0x6F12,0x0101,
	0x602A,0x11B8,
	0x6F12,0x0000,
	0x602A,0x11F6,
	0x6F12,0x0010,
	0x602A,0x4A74,
	0x6F12,0x0000,
	0x6F12,0x0000,
	0x6F12,0x0000,
	0x6F12,0x0000,
	0x6F12,0x0000,
	0x6F12,0x0000,
	0x6F12,0x0000,
	0x6F12,0x0000,
	0x6F12,0x0000,
	0x6F12,0x0000,
	0x6F12,0x0000,
	0x6F12,0x0000,
	0x6F12,0x0000,
	0x6F12,0x0000,
	0x6F12,0x0000,
	0x6F12,0x0000,
	0x6028,0x4000,
	0xF46A,0xAE80,
	0x0344,0x05F0,
	0x0346,0x0660,
	0x0348,0x1A0F,
	0x034A,0x11BF,
	0x034C,0x0500,
	0x034E,0x02D0,
	0x0350,0x0004,
	0x0352,0x0004,
	0x0900,0x0144,
	0x0380,0x0002,
	0x0382,0x0006,
	0x0384,0x0002,
	0x0386,0x0006,
	0x0110,0x1002,
	0x0114,0x0300,
	0x0116,0x3000,
	0x0136,0x1800,
	0x013E,0x0000,
	0x0300,0x0006,
	0x0302,0x0001,
	0x0304,0x0004,
	0x0306,0x0096,
	0x0308,0x0008,
	0x030A,0x0001,
	0x030C,0x0000,
	0x030E,0x0003,
	0x0310,0x0064,
	0x0312,0x0000,
	0x080E,0x0000,
	0x0340,0x094C,
	0x0342,0x0830,
	0x0702,0x0000,
	0x0202,0x0100,
	0x0200,0x0100,
	0x0D00,0x0101,
	0x0D02,0x0001,
	0x0D04,0x0002,
	0x6226,0x0000,
};
#endif

static void sensor_init(void)
{
	LOG_INF("E\n");
	#if 0
	write_cmos_sensor(0x6028, 0x4000);
	write_cmos_sensor(0x0000, 0x0009);
	write_cmos_sensor(0x0000, 0x08D1);
	write_cmos_sensor(0x6010, 0x0001);
	mdelay(5);
	write_cmos_sensor(0x6214, 0x7971);
	write_cmos_sensor(0x6218, 0x7150);
	write_cmos_sensor(0x0A02, 0x0074);
	#endif
	#ifdef USE_TNP_BURST
	table_write_cmos_sensor(addr_data_pair_init,
		sizeof(addr_data_pair_init) / sizeof(kal_uint16));
	#else
	
	#endif
}	/*	  sensor_init  */


static void preview_setting(void)
{
	//Preview 2320*1744 30fps 24M MCLK 4lane 1200Mbps/lane
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
}


static void normal_video_setting(kal_uint16 currefps)
{
	LOG_INF("normal_video_setting!\n");
	// full size 30fps
	#ifdef USE_TNP_BURST
	table_write_cmos_sensor(addr_data_pair_normal_video,
		sizeof(addr_data_pair_normal_video) / sizeof(kal_uint16));
	#else

	#endif
}
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

#ifdef VENDOR_EDIT
static kal_uint16 read_module_id(void)
{
	kal_uint16 get_byte = 0;
	char pusendcmd[2] = {(char)(MODULE_ID_OFFSET >> 8), (char)(MODULE_ID_OFFSET & 0xFF)};

	iReadRegI2C(pusendcmd, 2, (u8 *)&get_byte, 1, 0xA0/*EEPROM_READ_ID*/);
	pr_err("the module id is %d\n", get_byte);
	return get_byte;
}
#endif

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
	kal_uint16 module_id = 0;
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
			pr_err("read_0x0000=0x%x, 0x0001=0x%x,0x0000_0001=0x%x\n",read_cmos_sensor_8(0x0000),read_cmos_sensor_8(0x0001),read_cmos_sensor(0x0000));
			if (*sensor_id == imgsensor_info.sensor_id) {
				#ifdef VENDOR_EDIT
				module_id = read_module_id();
				read_eeprom_SN();
				if(deviceInfo_register_value == 0x00){
				//add by wq 	register_imgsensor_deviceinfo("Cam_b", DEVICE_VERSION_S5KJN1, module_id);
					deviceInfo_register_value = 0x01;
				}
				#endif
				pr_err("i2c write id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id,*sensor_id);
		
				return ERROR_NONE;
			}
			pr_err("Read sensor id fail, id: 0x%x ensor id: 0x%x\n", imgsensor.i2c_write_id,*sensor_id);
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
	default:
		break;
	}

	return ERROR_NONE;
}

static kal_uint32 set_test_pattern_mode(kal_bool enable)
{
	LOG_INF("enable: %d\n", enable);

	if (enable) {

		write_cmos_sensor(0x0600, 0x0002);
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

	LOG_INF("feature_id = %d", feature_id);
	switch (feature_id) 
	{
		// prize add for its:test_sensor_fusion 20201029 start
		case SENSOR_FEATURE_GET_OFFSET_TO_START_OF_EXPOSURE:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = 9900000;
			break;
		// prize add for its:test_sensor_fusion 20201029 end
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
				case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
				default:
				memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[0],sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
				break;
			}
			break;
		case SENSOR_FEATURE_GET_VC_INFO:
		pr_debug("SENSOR_FEATURE_GET_VC_INFO %d\n",
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
