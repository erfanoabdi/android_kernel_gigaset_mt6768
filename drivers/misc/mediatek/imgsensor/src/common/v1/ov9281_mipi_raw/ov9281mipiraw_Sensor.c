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

/*****************************************************************************
 *
 * Filename:
 * ---------
 *	 ov9281mipiraw_sensor.c
 *
 * Project:
 * --------
 *	 ALPS MT6580 2lane ov9281r2a
 *
 * Description:
 * ------------
 *	 Source code of Sensor driver
 *	PengtaoFan
 *  0528:??get_info,??mipi ??? *  0604:??init preview ????0ms
 *  0604:???init setting??stream on,?otp????
 *  0608:???ov???setting
 *  0703:for  ???non continue mode 4800
 *  0703:for  crc test 0x5002=00
 *  ---RD ??????? *  0714 ???capture setting full size@30fps 24fps 15fps
 *  15072115172729: ???nick??full size 30fps setting,for ???size ????????
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
/*#include <asm/atomic.h>*/

#include "kd_camera_typedef.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"
#include "kd_camera_feature.h"

#include "ov9281mipiraw_Sensor.h"


#define SENSORDB LOG_INF

/****************************Modify Following Strings for Debug****************************/
#define PFX "ov92812LANE"
//#define LOG_INF(format, args...)    pr_debug(PFX "[%s] " format, __func__, ##args)
#define LOG_INF(format, args...)    //printk(PFX "[%s] " format, __func__, ##args)
/****************************   Modify end    *******************************************/
static first_boot = 0;
extern void scan_led_laser_ctrl(int enable);
extern void scan_3v3_switch_ctrl_function(int enable);


static DEFINE_SPINLOCK(imgsensor_drv_lock);


static struct imgsensor_info_struct imgsensor_info = {
	.sensor_id = OV9281_SENSOR_ID,	/* record sensor id defined in Kd_imgsensor.h */

	.checksum_value = 0xc2ded17b,	/* checksum value for Camera Auto Test */

	.pre = {
		.pclk = 80000000,	/* record different mode's pclk */
		.linelength = 1456,	/* record different mode's linelength */
		.framelength = 910,	/* record different mode's framelength */
		.startx = 0,	/* record different mode's startx of grabwindow */
		.starty = 0,	/* record different mode's starty of grabwindow */
		.grabwindow_width = 1280,	/* record different mode's width of grabwindow */
		.grabwindow_height = 800,	/* record different mode's height of grabwindow */
		/*       following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario   */
		.mipi_data_lp2hs_settle_dc = 30,
		/*       following for GetDefaultFramerateByScenario()  */
		.max_framerate = 600,
		.mipi_pixel_rate = 288600000,
		},
	.cap = {
		.pclk = 87360000,
		.linelength = 1456,
		.framelength = 910,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 1280,
		.grabwindow_height = 800,
		.mipi_data_lp2hs_settle_dc = 30,
		.max_framerate = 600,
		.mipi_pixel_rate = 288600000,
		},
	.cap1 = {		/* capture for PIP 15fps */
		.pclk = 87360000,
		.linelength = 1456,
		.framelength = 910,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 1280,
		.grabwindow_height = 800,
		.mipi_data_lp2hs_settle_dc = 30,
		.max_framerate = 240,
		.mipi_pixel_rate = 288600000,
		 },
	.normal_video = {
		.pclk = 87360000,
		.linelength = 1456,
		.framelength = 910,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 1280,
		.grabwindow_height = 800,
		.mipi_data_lp2hs_settle_dc = 30,
		.max_framerate = 600,
		.mipi_pixel_rate = 288600000,
			 },
	.hs_video = {
		.pclk = 87360000,
		.linelength = 1456,
		.framelength = 910,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 1280,
		.grabwindow_height = 800,
		.mipi_data_lp2hs_settle_dc = 30,
		.max_framerate = 600,
		.mipi_pixel_rate = 288600000,
		     },
	.slim_video = {
		.pclk = 87360000,
		.linelength = 1456,
		.framelength = 910,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 1280,
		.grabwindow_height = 800,
		.mipi_data_lp2hs_settle_dc = 30,
		.max_framerate = 600,
		.mipi_pixel_rate = 288600000,
		       },
	// huangjiwu for  captrue black --begin 
	.min_gain = 73,
	.max_gain = 4096,
	.min_gain_iso = 100,
	.exp_step = 2,
	.gain_step = 1,
	.gain_type = 1,
	.temperature_support=0,
	.margin = 4,		/* sensor framelength & shutter margin */
	.min_shutter = 4,	/* min shutter */
	.max_frame_length = 0x90f7,	/* max framelength by sensor register's limitation */
	.ae_shut_delay_frame = 0,
	/*shutter delay frame for AE cycle, 2 frame with ispGain_delay-shut_delay=2-0=2 */
	.ae_sensor_gain_delay_frame = 0,
	/* sensor gain delay frame for AE cycle,2 frame with ispGain_delay-sensor_gain_delay=2-0=2 */
	.ae_ispGain_delay_frame = 2,	/* isp gain delay frame for AE cycle */
	.ihdr_support = 0,	/* 1, support; 0,not support */
	.ihdr_le_firstline = 0,	/* 1,le first ; 0, se first */
	.sensor_mode_num = 5,	/* support sensor mode num ,don't support Slow motion */

	.cap_delay_frame = 1,	/* enter capture delay frame num */
	.pre_delay_frame = 1,	/* enter preview delay frame num */
	.video_delay_frame = 1,	/* enter video delay frame num */
	.hs_video_delay_frame = 3,	/* enter high speed video  delay frame num */
	.slim_video_delay_frame = 3,	/* enter slim video delay frame num */

	.isp_driving_current = ISP_DRIVING_8MA,	/* mclk driving current */
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,	/* sensor_interface_type */
	.mipi_sensor_type = MIPI_OPHY_NCSI2,	/* 0,MIPI_OPHY_NCSI2;  1,MIPI_OPHY_CSI2 */
	.mipi_settle_delay_mode = 1,
	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_MONO,//SENSOR_OUTPUT_FORMAT_RAW_Gb,	/* sensor output first pixel color */
	.mclk = 24,		/* mclk value, suggest 24 or 26 for 24Mhz or 26Mhz */
	.mipi_lane_num = SENSOR_MIPI_2_LANE,	/* mipi lane num */
	.i2c_addr_table = {0xc0, 0xff}, //{0xc0, 0xff},
	.i2c_speed = 400,	/* i2c read/write speed */
};


static struct imgsensor_struct imgsensor = {
	.mirror = IMAGE_NORMAL,	/* mirrorflip information */
	.sensor_mode = IMGSENSOR_MODE_INIT,
	.shutter = 0x4C00,	/* current shutter */
	.gain = 0x200,		/* current gain */
	.dummy_pixel = 0,	/* current dummypixel */
	.dummy_line = 0,	/* current dummyline */
	.current_fps = 0,	/* full size current fps : 24fps for PIP, 30fps for Normal or ZSD */
	.autoflicker_en = KAL_FALSE,
	.test_pattern = KAL_FALSE,
	.current_scenario_id = MSDK_SCENARIO_ID_CAMERA_PREVIEW,	/* current scenario id */
	.ihdr_en = 0,		/* sensor need support LE, SE with HDR feature */
	.i2c_write_id = 0,	/* record current sensor's i2c write id */
};


/* Sensor output window information*/
static struct SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[5] = {
    {1280, 800, 0, 0, 1280, 800, 1280, 800, 0, 0, 1280, 800, 0, 0, 1280, 800},	/* Preview */
    {1280, 800, 0, 0, 1280, 800, 1280, 800, 0, 0, 1280, 800, 0, 0, 1280, 800},   /* capture */
	{1280, 800, 0, 0, 1280, 800, 1280, 800, 0, 0, 1280, 800, 0, 0, 1280, 800},	/* video */
	{1280, 800, 0, 0, 1280, 800, 1280, 800, 0, 0, 1280, 800, 0, 0, 1280, 800},	/* hight speed video */
	{1280, 800, 0, 0, 1280, 800, 1280, 800, 0, 0, 1280, 800, 0, 0, 1280, 800}
};/* slim video */


enum SENSOR_DPCM_TYPE_ENUM imgsensor_dpcm_info_ov9281[10] = {
	COMP8_NONE,
	COMP8_NONE,
	COMP8_NONE,
	COMP8_NONE,
	COMP8_NONE,
	COMP8_NONE,
	COMP8_NONE,
	COMP8_NONE,
	COMP8_NONE,
	COMP8_NONE
};


#define MULTI_WRITE 1

#if MULTI_WRITE
#define I2C_BUFFER_LEN 225
#else
#define I2C_BUFFER_LEN 3

#endif
static kal_uint16 ov9281_table_write_cmos_sensor(kal_uint16 *para, kal_uint32 len)
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
			puSendCmd[tosend++] = (char)(data & 0xFF);
			IDX += 2;
			addr_last = addr;

		}
#if MULTI_WRITE
		/* Write when remain buffer size is less than 3 bytes or reach end of data */
		if ((I2C_BUFFER_LEN - tosend) < 3 || IDX == len || addr != addr_last) {
			iBurstWriteReg_multi(puSendCmd, tosend, imgsensor.i2c_write_id, 3,
					     imgsensor_info.i2c_speed);
			tosend = 0;
		}
#else
		iWriteRegI2C(puSendCmd, 3, imgsensor.i2c_write_id);
		tosend = 0;

#endif
	}
    
	return 0;
}

static kal_uint16 read_cmos_sensor(kal_uint32 addr)
{
	kal_uint16 get_byte = 0;

	char pu_send_cmd[2] = { (char)(addr >> 8), (char)(addr & 0xFF) };
	kdSetI2CSpeed(imgsensor_info.i2c_speed); // Add this func to set i2c speed by each sensor
	//iReadRegI2C(pu_send_cmd, 2, (u8 *) &get_byte, 1, imgsensor.i2c_write_id);
	iReadRegI2CTiming(pu_send_cmd, 2, (u8 *) &get_byte, 1, imgsensor.i2c_write_id, 200);
	return get_byte;
}

static void write_cmos_sensor(kal_uint32 addr, kal_uint32 para)
{
	char pu_send_cmd[3] = { (char)(addr >> 8), (char)(addr & 0xFF), (char)(para & 0xFF) };

	iWriteRegI2C(pu_send_cmd, 3, imgsensor.i2c_write_id);
}

static void set_dummy(void)
{
	LOG_INF("dummyline = %d, dummypixels = %d\n", imgsensor.dummy_line, imgsensor.dummy_pixel);
	write_cmos_sensor(0x380e, imgsensor.frame_length >> 8);
	write_cmos_sensor(0x380f, imgsensor.frame_length & 0xFF);
	write_cmos_sensor(0x380c, imgsensor.line_length >> 8);
	write_cmos_sensor(0x380d, imgsensor.line_length & 0xFF);
}


static void set_max_framerate(UINT16 framerate, kal_bool min_framelength_en)
{
	kal_uint32 frame_length;

	LOG_INF("framerate = %d, min framelength should enable?\n", framerate);

	frame_length = imgsensor.pclk / framerate * 10 / imgsensor.line_length;
	spin_lock(&imgsensor_drv_lock);
	imgsensor.frame_length =
		(frame_length > imgsensor.min_frame_length) ? frame_length : imgsensor.min_frame_length;
	imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;
	if (imgsensor.frame_length > imgsensor_info.max_frame_length) {
		imgsensor.frame_length = imgsensor_info.max_frame_length;
		imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;
	}
	if (min_framelength_en)
		imgsensor.min_frame_length = imgsensor.frame_length;
	spin_unlock(&imgsensor_drv_lock);
	set_dummy();
}				/*      set_max_framerate  */


static void set_shutter_frame_length(kal_uint16 shutter, kal_uint16 frame_length)
{
	unsigned long flags;
	kal_uint16 realtime_fps = 0;
	kal_int32 dummy_line = 0;

	spin_lock_irqsave(&imgsensor_drv_lock, flags);
	imgsensor.shutter = shutter;
	spin_unlock_irqrestore(&imgsensor_drv_lock, flags);
	spin_lock(&imgsensor_drv_lock);
	/*Change frame time*/
	dummy_line = frame_length - imgsensor.frame_length;
	imgsensor.frame_length = imgsensor.frame_length + dummy_line;
	imgsensor.min_frame_length = imgsensor.frame_length;

	if (shutter > imgsensor.min_frame_length - imgsensor_info.margin)
		imgsensor.frame_length = shutter + imgsensor_info.margin;
	else
		imgsensor.frame_length = imgsensor.min_frame_length;
	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
		imgsensor.frame_length = imgsensor_info.max_frame_length;
	spin_unlock(&imgsensor_drv_lock);
	shutter = (shutter < imgsensor_info.min_shutter) ? imgsensor_info.min_shutter : shutter;
	if (shutter > (imgsensor_info.max_frame_length - imgsensor_info.margin))
		shutter = (imgsensor_info.max_frame_length - imgsensor_info.margin);

	if (imgsensor.autoflicker_en) {
		realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 / imgsensor.frame_length;
		if (realtime_fps >= 297 && realtime_fps <= 305)
			set_max_framerate(296, 0);
		else if (realtime_fps >= 147 && realtime_fps <= 150)
			set_max_framerate(146, 0);
		else {
		/* Extend frame length*/
		write_cmos_sensor(0x380e, imgsensor.frame_length >> 8);
		write_cmos_sensor(0x380f, imgsensor.frame_length & 0xFF);
		}
	} else {
		/* Extend frame length*/
		 write_cmos_sensor(0x380e, imgsensor.frame_length >> 8);
		 write_cmos_sensor(0x380f, imgsensor.frame_length & 0xFF);
	}

	/* Update Shutter*/
	write_cmos_sensor(0x3502, (shutter << 4) & 0xFF);
	write_cmos_sensor(0x3501, (shutter >> 4) & 0xFF);
	write_cmos_sensor(0x3500, (shutter >> 12) & 0x0F);

	LOG_INF("Add for N3D! shutterlzl =%d, framelength =%d\n", shutter, imgsensor.frame_length);

}

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
static void set_shutter(kal_uint16 shutter)
{   
	unsigned long flags;
	kal_uint16 realtime_fps = 0;
	/*kal_uint32 frame_length = 0;*/
	spin_lock_irqsave(&imgsensor_drv_lock, flags);
	imgsensor.shutter = shutter;
	spin_unlock_irqrestore(&imgsensor_drv_lock, flags);

	/* write_shutter(shutter); */
	/* 0x3500, 0x3501, 0x3502 will increase VBLANK to get exposure larger than frame exposure */
	/* AE doesn't update sensor gain at capture mode, thus extra exposure lines must be updated here. */
	
	/* OV Recommend Solution */
	/* if shutter bigger than frame_length, should extend frame length first */
	spin_lock(&imgsensor_drv_lock);
	if (shutter > imgsensor.min_frame_length - imgsensor_info.margin)
		imgsensor.frame_length = shutter + imgsensor_info.margin;
	else
		imgsensor.frame_length = imgsensor.min_frame_length;
	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
		imgsensor.frame_length = imgsensor_info.max_frame_length;
	spin_unlock(&imgsensor_drv_lock);
	shutter = (shutter < imgsensor_info.min_shutter) ? imgsensor_info.min_shutter : shutter;
	if (shutter > (imgsensor_info.max_frame_length - imgsensor_info.margin))
		shutter = (imgsensor_info.max_frame_length - imgsensor_info.margin);

	if (imgsensor.autoflicker_en) {
		realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 / imgsensor.frame_length;
		if (realtime_fps >= 297 && realtime_fps <= 305)
			set_max_framerate(296, 0);
		else if (realtime_fps >= 147 && realtime_fps <= 150)
			set_max_framerate(146, 0);
		else {
		/* Extend frame length*/
		write_cmos_sensor(0x380e, imgsensor.frame_length >> 8);
		write_cmos_sensor(0x380f, imgsensor.frame_length & 0xFF);
		}
	} else {
		/* Extend frame length*/
		write_cmos_sensor(0x380e, imgsensor.frame_length >> 8);
		write_cmos_sensor(0x380f, imgsensor.frame_length & 0xFF);
	}

	/* Update Shutter*/
	write_cmos_sensor(0x3502, (shutter << 4) & 0xFF);
	write_cmos_sensor(0x3501, (shutter >> 4) & 0xFF);
	write_cmos_sensor(0x3500, (shutter >> 12) & 0x0F);
	LOG_INF("Exit! shutter =%d, framelength =%d\n", shutter, imgsensor.frame_length);

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

	if (gain < BASEGAIN || gain > 15 * BASEGAIN) {
		LOG_INF("Error gain setting");

		if (gain < BASEGAIN)
			gain = BASEGAIN;
		else if (gain > 15 * BASEGAIN)
			gain = 15 * BASEGAIN;
	}

	reg_gain = gain*2;
	spin_lock(&imgsensor_drv_lock);
	imgsensor.gain = reg_gain;
	spin_unlock(&imgsensor_drv_lock);
	LOG_INF("gain = %d , reg_gain = 0x%x\n ", gain, reg_gain);

	write_cmos_sensor(0x3508, (reg_gain>>8));
	write_cmos_sensor(0x3509, (reg_gain&0xFF));
	return gain;
}				/*      set_gain  */

static void ihdr_write_shutter_gain(kal_uint16 le, kal_uint16 se, kal_uint16 gain)
{
	LOG_INF("le:0x%x, se:0x%x, gain:0x%x\n", le, se, gain);
	if (imgsensor.ihdr_en) {
		spin_lock(&imgsensor_drv_lock);
			if (le > imgsensor.min_frame_length - imgsensor_info.margin)
				imgsensor.frame_length = le + imgsensor_info.margin;
			else
				imgsensor.frame_length = imgsensor.min_frame_length;
			if (imgsensor.frame_length > imgsensor_info.max_frame_length)
				imgsensor.frame_length = imgsensor_info.max_frame_length;
			spin_unlock(&imgsensor_drv_lock);
			if (le < imgsensor_info.min_shutter)
				le = imgsensor_info.min_shutter;
			if (se < imgsensor_info.min_shutter)
				se = imgsensor_info.min_shutter;
		/* Extend frame length first*/
		write_cmos_sensor(0x380e, imgsensor.frame_length >> 8);
		write_cmos_sensor(0x380f, imgsensor.frame_length & 0xFF);
		write_cmos_sensor(0x3502, (le << 4) & 0xFF);
		write_cmos_sensor(0x3501, (le >> 4) & 0xFF);
		write_cmos_sensor(0x3500, (le >> 12) & 0x0F);
		write_cmos_sensor(0x3512, (se << 4) & 0xFF);
		write_cmos_sensor(0x3511, (se >> 4) & 0xFF);
		write_cmos_sensor(0x3510, (se >> 12) & 0x0F);
		set_gain(gain);
	}
}


#if 1
static void set_mirror_flip(kal_uint8 image_mirror)
{
	LOG_INF("image_mirror = %d\n", image_mirror);

	/********************************************************
	   *
	   *   0x3820[2] ISP Vertical flip
	   *   0x3820[1] Sensor Vertical flip
	   *
	   *   0x3821[2] ISP Horizontal mirror
	   *   0x3821[1] Sensor Horizontal mirror
	   *
	   *   ISP and Sensor flip or mirror register bit should be the same!!
	   *
	   ********************************************************/

	switch (image_mirror) {
	case IMAGE_NORMAL:
		write_cmos_sensor(0x3820, ((read_cmos_sensor(0x3820) & 0xF9) | 0x00));
		write_cmos_sensor(0x3821, ((read_cmos_sensor(0x3821) & 0xF9) | 0x06));
		break;
	case IMAGE_H_MIRROR:
		write_cmos_sensor(0x3820, ((read_cmos_sensor(0x3820) & 0xF9) | 0x00));
		write_cmos_sensor(0x3821, ((read_cmos_sensor(0x3821) & 0xF9) | 0x00));
		break;
	case IMAGE_V_MIRROR:
		write_cmos_sensor(0x3820, ((read_cmos_sensor(0x3820) & 0xF9) | 0x06));
		write_cmos_sensor(0x3821, ((read_cmos_sensor(0x3821) & 0xF9) | 0x06));
		break;
	case IMAGE_HV_MIRROR:
		write_cmos_sensor(0x3820, ((read_cmos_sensor(0x3820) & 0xF9) | 0x06));
		write_cmos_sensor(0x3821, ((read_cmos_sensor(0x3821) & 0xF9) | 0x00));
		break;
	default:
		LOG_INF("Error image_mirror setting\n");
	}

}
#endif
/*************************************************************************
* FUNCTION
*	night_mode
*
* DESCRIPTION
*	This function night mode of sensor.
*
* PARAMETERS
*	bEnable: KAL_TRUE -> enable night mode, otherwise, disable night mode
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static void night_mode(kal_bool enable)
{
/*No Need to implement this function*/
}				/*      night_mode      */

kal_uint16 addr_data_pair_init_ov9281[] = {
	0x103, 0x01,
	0x302, 0x32,
	0x30d, 0x50,
	0x30e, 0x02,
	0x3001, 0x00,
	0x3004, 0x00,
	0x3005, 0x00,
	0x3006, 0x04,
	0x3011, 0x0a,
	0x3013, 0x18,
	0x3022, 0x01,
	0x3030, 0x10,
	0x3039, 0x32,
	0x303a, 0x00,
	0x3500, 0x00,
	0x3501, 0x2a,
	0x3502, 0x90,
	0x3503, 0x08,
	0x3505, 0x8c,
	0x3507, 0x03,
	0x3508, 0x00,
	0x3509, 0x10,
	0x3610, 0x80,
	0x3611, 0xa0,
	0x3620, 0x6f,
	0x3632, 0x56,
	0x3633, 0x78,
	0x3662, 0x05,
	0x3666, 0x00,
	0x366f, 0x5a,
	0x3680, 0x84,
	0x3712, 0x80,
	0x372d, 0x22,
	0x3731, 0x80,
	0x3732, 0x30,
	0x3778, 0x00,
	0x377d, 0x22,
	0x3788, 0x02,
	0x3789, 0xa4,
	0x378a, 0x00,
	0x378b, 0x4a,
	0x3799, 0x20,
	0x3800, 0x00,
	0x3801, 0x00,
	0x3802, 0x00,
	0x3803, 0x00,
	0x3804, 0x05,
	0x3805, 0x0f,
	0x3806, 0x03,
	0x3807, 0x2f,
	0x3808, 0x05,
	0x3809, 0x00,
	0x380a, 0x03,
	0x380b, 0x20,
	0x380c, 0x02,
	0x380d, 0xd8,
	0x380e, 0x0e,
	0x380f, 0x38,
	0x3810, 0x00,
	0x3811, 0x08,
	0x3812, 0x00,
	0x3813, 0x08,
	0x3814, 0x11,
	0x3815, 0x11,
	//0x3820, 0x40,
	//0x3821, 0x00,
	0x3881, 0x42,
	0x38b1, 0x00,
	0x3920, 0xff,
	0x4003, 0x40,
	0x4008, 0x04,
	0x4009, 0x0b,
	0x400c, 0x00,
	0x400d, 0x07,
	0x4010, 0x40,
	0x4043, 0x40,
	0x4307, 0x30,
	0x4317, 0x00,
	0x4501, 0x00,
	0x4507, 0x00,
	0x4509, 0x00,
	0x450a, 0x08,
	0x4601, 0x04,
	0x470f, 0x00,
	0x4f07, 0x00,
	0x4800, 0x00,
	0x5000, 0x9f,
	0x5001, 0x00,
	0x5e00, 0x00,
	0x5d00, 0x07,
	0x5d01, 0x00,
	0x100, 0x00,

};

kal_uint16 addr_data_pair_preview_120fps_640X400_ov9281[] = 
{
	0x0103, 0x01,
	0x0302, 0x32,
	0x030d, 0x50,
	0x030e, 0x02,
	0x3001, 0x00,
	0x3004, 0x00,
	0x3005, 0x00,
	0x3006, 0x04,
	0x3011, 0x0a,
	0x3013, 0x18,
	0x3022, 0x01,
	0x3030, 0x10,
	0x3039, 0x32,
	0x303a, 0x00,
	0x3500, 0x00,
	0x3501, 0x05,
	0x3502, 0x90,
	0x3503, 0x08,
	0x3505, 0x8c,
	0x3507, 0x03,
	0x3508, 0x00,
	0x3509, 0x10,
	0x3610, 0x80,
	0x3611, 0xa0,
	0x3620, 0x6f,
	0x3632, 0x56,
	0x3633, 0x78,
	0x3662, 0x05,
	0x3666, 0x00,
	0x366f, 0x5a,
	0x3680, 0x84,
	0x3712, 0x80,
	0x372d, 0x22,
	0x3731, 0x80,
	0x3732, 0x30,
	0x3778, 0x00,
	0x377d, 0x22,
	0x3788, 0x02,
	0x3789, 0xa4,
	0x378a, 0x00,
	0x378b, 0x4a,
	0x3799, 0x20,
	0x3800, 0x00,
	0x3801, 0x00,
	0x3802, 0x00,
	0x3803, 0x00,
	0x3804, 0x05,
	0x3805, 0x0f,
	0x3806, 0x03,
	0x3807, 0x2f,
	0x3808, 0x05,
	0x3809, 0x00,
	0x380a, 0x03,
	0x380b, 0x20,
	0x380c, 0x05,
	0x380d, 0xb0,
	0x380e, 0x03,
	0x380f, 0x8e,
	0x3810, 0x00,
	0x3811, 0x08,
	0x3812, 0x00,
	0x3813, 0x08,
	0x3814, 0x11,
	0x3815, 0x11,
	//0x3820, 0x40,
	//0x3821, 0x00,
	0x3881, 0x42,
	0x38b1, 0x00,
	0x3920, 0xff,
	0x4003, 0x40,
	0x4008, 0x04,
	0x4009, 0x0b,
	0x400c, 0x00,
	0x400d, 0x07,
	0x4010, 0x40,
	0x4043, 0x40,
	0x4307, 0x30,
	0x4317, 0x00,
	0x4501, 0x00,
	0x4507, 0x00,
	0x4509, 0x00,
	0x450a, 0x08,
	0x4601, 0x04,
	0x470f, 0x00,
	0x4f07, 0x00,
	0x4800, 0x00,
	0x5000, 0x9f,
	0x5001, 0x00,
	0x5e00, 0x00,
	0x5d00, 0x07,
	0x5d01, 0x00,
	0x100, 0x01,
};
static void sensor_init(void)
{
	LOG_INF("sensor_init E\n");
	/* ov9281 R2A setting */
	/* 3.1 Initialization (Global Setting) */
	/* XVCLK=24Mhz, SCLK=72Mhz, MIPI 720Mbps, DACCLK=180Mhz */
	write_cmos_sensor(0x103, 0x01);
	mdelay(5);			/* MULTI_WRITE */
	ov9281_table_write_cmos_sensor(addr_data_pair_init_ov9281,
				       sizeof(addr_data_pair_init_ov9281) / sizeof(kal_uint16));

	LOG_INF("sensor_init X\n");
}				/*      sensor_init  */

static kal_uint32 streaming_control(kal_bool enable)
{
	LOG_INF("streaming_enable(0=Sw Standby,1=streaming): %d\n", enable);
	if (enable)
		write_cmos_sensor(0x0100, 0X01);
	else
		write_cmos_sensor(0x0100, 0x00);
	return ERROR_NONE;
}
static void preview_setting(void)
{
	LOG_INF("wysdebug: E\n");

		ov9281_table_write_cmos_sensor(addr_data_pair_preview_120fps_640X400_ov9281,
				       sizeof(addr_data_pair_preview_120fps_640X400_ov9281) / sizeof(kal_uint16));
}				/*      preview_setting  */

kal_uint16 addr_data_pair_capture_30fps_ov9281[] = {
	0x103, 0x01,
	0x302, 0x32,
	0x30d, 0x50,
	0x30e, 0x02,
	0x3001, 0x00,
	0x3004, 0x00,
	0x3005, 0x00,
	0x3006, 0x04,
	0x3011, 0x0a,
	0x3013, 0x18,
	0x3022, 0x01,
	0x3030, 0x10,
	0x3039, 0x32,
	0x303a, 0x00,
	0x3500, 0x00,
	0x3501, 0x2a,
	0x3502, 0x90,
	0x3503, 0x08,
	0x3505, 0x8c,
	0x3507, 0x03,
	0x3508, 0x00,
	0x3509, 0x10,
	0x3610, 0x80,
	0x3611, 0xa0,
	0x3620, 0x6f,
	0x3632, 0x56,
	0x3633, 0x78,
	0x3662, 0x05,
	0x3666, 0x00,
	0x366f, 0x5a,
	0x3680, 0x84,
	0x3712, 0x80,
	0x372d, 0x22,
	0x3731, 0x80,
	0x3732, 0x30,
	0x3778, 0x00,
	0x377d, 0x22,
	0x3788, 0x02,
	0x3789, 0xa4,
	0x378a, 0x00,
	0x378b, 0x4a,
	0x3799, 0x20,
	0x3800, 0x00,
	0x3801, 0x00,
	0x3802, 0x00,
	0x3803, 0x00,
	0x3804, 0x05,
	0x3805, 0x0f,
	0x3806, 0x03,
	0x3807, 0x2f,
	0x3808, 0x05,
	0x3809, 0x00,
	0x380a, 0x03,
	0x380b, 0x20,
	0x380c, 0x02,
	0x380d, 0xd8,
	0x380e, 0x0e,
	0x380f, 0x38,
	0x3810, 0x00,
	0x3811, 0x08,
	0x3812, 0x00,
	0x3813, 0x08,
	0x3814, 0x11,
	0x3815, 0x11,
	//0x3820, 0x40,
	//0x3821, 0x00,
	0x3881, 0x42,
	0x38b1, 0x00,
	0x3920, 0xff,
	0x4003, 0x40,
	0x4008, 0x04,
	0x4009, 0x0b,
	0x400c, 0x00,
	0x400d, 0x07,
	0x4010, 0x40,
	0x4043, 0x40,
	0x4307, 0x30,
	0x4317, 0x00,
	0x4501, 0x00,
	0x4507, 0x00,
	0x4509, 0x00,
	0x450a, 0x08,
	0x4601, 0x04,
	0x470f, 0x00,
	0x4f07, 0x00,
	0x4800, 0x00,
	0x5000, 0x9f,
	0x5001, 0x00,
	0x5e00, 0x00,
	0x5d00, 0x07,
	0x5d01, 0x00,
	0x100, 0x01,
};

static void capture_setting(kal_uint16 currefps)
{
	LOG_INF("qydebug: E! currefps:%d\n", currefps);
	if (currefps == 300) {
		ov9281_table_write_cmos_sensor(addr_data_pair_capture_30fps_ov9281,
					       sizeof(addr_data_pair_capture_30fps_ov9281) /
					       sizeof(kal_uint16));
	}
}


static void normal_video_setting(kal_uint16 currefps)
{
	LOG_INF("E! currefps:%d\n", currefps);
	LOG_INF("E! video just has 30fps preview size setting ,NOT HAS 24FPS SETTING!\n");
	preview_setting();
}

static void hs_video_setting(void)
{
	LOG_INF("E\n");
/* @@640x480 120fps 2lane 720M bps/lane */
/* ;;MIPI=720Mbps, SysClk=144Mhz,Dac Clock=360Mhz */
/* ;;2306x520x120 */
/* 100 99 640 480 */
	write_cmos_sensor(0x103, 0x01);
	write_cmos_sensor(0x302, 0x32);
	write_cmos_sensor(0x30d, 0x50);
	write_cmos_sensor(0x30e, 0x02);
	write_cmos_sensor(0x3001, 0x00);
	write_cmos_sensor(0x3004, 0x00);
	write_cmos_sensor(0x3005, 0x00);
	write_cmos_sensor(0x3006, 0x04);
	write_cmos_sensor(0x3011, 0x0a);
	write_cmos_sensor(0x3013, 0x18);
	write_cmos_sensor(0x3022, 0x01);
	write_cmos_sensor(0x3030, 0x10);
	write_cmos_sensor(0x3039, 0x32);
	write_cmos_sensor(0x303a, 0x00);
	write_cmos_sensor(0x3500, 0x00);
	write_cmos_sensor(0x3501, 0x2a);
	write_cmos_sensor(0x3502, 0x90);
	write_cmos_sensor(0x3503, 0x08);
	write_cmos_sensor(0x3505, 0x8c);
	write_cmos_sensor(0x3507, 0x03);
	write_cmos_sensor(0x3508, 0x00);
	write_cmos_sensor(0x3509, 0x10);
	write_cmos_sensor(0x3610, 0x80);
	write_cmos_sensor(0x3611, 0xa0);
	write_cmos_sensor(0x3620, 0x6f);
	write_cmos_sensor(0x3632, 0x56);
	write_cmos_sensor(0x3633, 0x78);
	write_cmos_sensor(0x3662, 0x05);
	write_cmos_sensor(0x3666, 0x00);
	write_cmos_sensor(0x366f, 0x5a);
	write_cmos_sensor(0x3680, 0x84);
	write_cmos_sensor(0x3712, 0x80);
	write_cmos_sensor(0x372d, 0x22);
	write_cmos_sensor(0x3731, 0x80);
	write_cmos_sensor(0x3732, 0x30);
	write_cmos_sensor(0x3778, 0x00);
	write_cmos_sensor(0x377d, 0x22);
	write_cmos_sensor(0x3788, 0x02);
	write_cmos_sensor(0x3789, 0xa4);
	write_cmos_sensor(0x378a, 0x00);
	write_cmos_sensor(0x378b, 0x4a);
	write_cmos_sensor(0x3799, 0x20);
	write_cmos_sensor(0x3800, 0x00);
	write_cmos_sensor(0x3801, 0x00);
	write_cmos_sensor(0x3802, 0x00);
	write_cmos_sensor(0x3803, 0x00);
	write_cmos_sensor(0x3804, 0x05);
	write_cmos_sensor(0x3805, 0x0f);
	write_cmos_sensor(0x3806, 0x03);
	write_cmos_sensor(0x3807, 0x2f);
	write_cmos_sensor(0x3808, 0x05);
	write_cmos_sensor(0x3809, 0x00);
	write_cmos_sensor(0x380a, 0x03);
	write_cmos_sensor(0x380b, 0x20);
	write_cmos_sensor(0x380c, 0x02);
	write_cmos_sensor(0x380d, 0xd8);
	write_cmos_sensor(0x380e, 0x0e);
	write_cmos_sensor(0x380f, 0x38);
	write_cmos_sensor(0x3810, 0x00);
	write_cmos_sensor(0x3811, 0x08);
	write_cmos_sensor(0x3812, 0x00);
	write_cmos_sensor(0x3813, 0x08);
	write_cmos_sensor(0x3814, 0x11);
	write_cmos_sensor(0x3815, 0x11);
	//write_cmos_sensor(0x3820, 0x40);
	//write_cmos_sensor(0x3821, 0x00);
	write_cmos_sensor(0x3881, 0x42);
	write_cmos_sensor(0x38b1, 0x00);
	write_cmos_sensor(0x3920, 0xff);
	write_cmos_sensor(0x4003, 0x40);
	write_cmos_sensor(0x4008, 0x04);
	write_cmos_sensor(0x4009, 0x0b);
	write_cmos_sensor(0x400c, 0x00);
	write_cmos_sensor(0x400d, 0x07);
	write_cmos_sensor(0x4010, 0x40);
	write_cmos_sensor(0x4043, 0x40);
	write_cmos_sensor(0x4307, 0x30);
	write_cmos_sensor(0x4317, 0x00);
	write_cmos_sensor(0x4501, 0x00);
	write_cmos_sensor(0x4507, 0x00);
	write_cmos_sensor(0x4509, 0x00);
	write_cmos_sensor(0x450a, 0x08);
	write_cmos_sensor(0x4601, 0x04);
	write_cmos_sensor(0x470f, 0x00);
	write_cmos_sensor(0x4f07, 0x00);
	write_cmos_sensor(0x4800, 0x00);
	write_cmos_sensor(0x5000, 0x9f);
	write_cmos_sensor(0x5001, 0x00);
	write_cmos_sensor(0x5e00, 0x00);
	write_cmos_sensor(0x5d00, 0x07);
	write_cmos_sensor(0x5d01, 0x00);
	write_cmos_sensor(0x100, 0x01);
}

static void slim_video_setting(void)
{
	LOG_INF("E\n");
	preview_setting();
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
		/*sensor have two i2c address 0x6c 0x6d & 0x21 0x20, we should detect the module used i2c address*/
		while (imgsensor_info.i2c_addr_table[i] != 0xff) {
			spin_lock(&imgsensor_drv_lock);
			imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
			spin_unlock(&imgsensor_drv_lock);
			do {
				*sensor_id = ((read_cmos_sensor(0x300A) << 8) | read_cmos_sensor(0x300B));
				printk("===wys===ov9281: *sensor_id = 0x%x ==\n", *sensor_id);
				if (*sensor_id == imgsensor_info.sensor_id) {
					printk("ov9281 i2c write id: 0x%x, sensor id: 0x%x, ver = 0x%x<0xB1=r2a,0xB0=r1a>\n",
						imgsensor.i2c_write_id, *sensor_id, read_cmos_sensor(0x302A));
					return ERROR_NONE;
				}
				printk("ov9281 Read sensor id fail,write_id:0x%x, id: 0x%x\n", imgsensor.i2c_write_id, *sensor_id);
				retry--;
			} while (retry > 0);
			i++;
			retry = 2;
		}
		if (*sensor_id != imgsensor_info.sensor_id) {
			/* if Sensor ID is not correct, Must set *sensor_id to 0xFFFFFFFF */
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
	/* const kal_uint8 i2c_addr[] = {IMGSENSOR_WRITE_ID_1, IMGSENSOR_WRITE_ID_2}; */
	kal_uint8 i = 0;
	kal_uint8 retry = 2;
	kal_uint32 sensor_id = 0;

	printk("ov9281 open E\n");
	/* sensor have two i2c address 0x6c 0x6d & 0x21 0x20, we should detect the module used i2c address */
	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
			sensor_id = ((read_cmos_sensor(0x300A) << 8) | read_cmos_sensor(0x300B));
			if (sensor_id == imgsensor_info.sensor_id) {
				printk("===wys==open==ov9281 i2c write id: 0x%x, sensor id: 0x%x\n",
					imgsensor.i2c_write_id, sensor_id);
				break;
			}
			LOG_INF("ov9281 Read sensor id fail, id: 0x%x,sensor_id =0x%x\n",
				imgsensor.i2c_write_id, sensor_id);
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
    
#ifdef ov9281R2AOTP
	write_cmos_sensor(0x100, 0x01);
	mdelay(10);

	LOG_INF("Apply the sensor OTP\n");
	struct otp_struct *otp_ptr =
				kzalloc(sizeof(struct otp_struct), GFP_KERNEL);
	read_otp(otp_ptr);
	apply_otp(otp_ptr);
	kfree(otp_ptr);
	write_cmos_sensor(0x100, 0x00);
	mdelay(20);
#endif

	spin_lock(&imgsensor_drv_lock);

	imgsensor.autoflicker_en = KAL_FALSE;
	imgsensor.sensor_mode = IMGSENSOR_MODE_INIT;
	imgsensor.shutter = 0x2D00;
	imgsensor.gain = 0x100;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	imgsensor.frame_length = imgsensor_info.pre.framelength;
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.dummy_pixel = 0;
	imgsensor.dummy_line = 0;
	imgsensor.ihdr_en = 0;
	imgsensor.test_pattern = KAL_FALSE;
	imgsensor.current_fps = imgsensor_info.pre.max_framerate;
	spin_unlock(&imgsensor_drv_lock);
	set_mirror_flip(imgsensor.mirror);
	if(first_boot==0){
		first_boot++;
	}else{
		scan_led_laser_ctrl(1);
	}
	
	LOG_INF("ov9281 open X\n");
	return ERROR_NONE;
}				/*      open  */


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

	printk("===ov9281===close===");
	scan_led_laser_ctrl(0);
	scan_3v3_switch_ctrl_function(0);
	/*No Need to implement this function */

	return ERROR_NONE;
}				/*      close  */


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
	LOG_INF("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_PREVIEW;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	/* imgsensor.video_mode = KAL_FALSE; */
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.frame_length = imgsensor_info.pre.framelength;
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	preview_setting();
	mdelay(10);
    
	return ERROR_NONE;
}				/*      preview   */

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
	LOG_INF("E! current %d fps\n", imgsensor.current_fps);
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CAPTURE;
	if (imgsensor.current_fps == imgsensor_info.cap1.max_framerate) {
		imgsensor.pclk = imgsensor_info.cap1.pclk;
		imgsensor.line_length = imgsensor_info.cap1.linelength;
		imgsensor.frame_length = imgsensor_info.cap1.framelength;
		imgsensor.min_frame_length = imgsensor_info.cap1.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	} else {
		if (imgsensor.current_fps != imgsensor_info.cap.max_framerate)
			LOG_INF("Warning: current_fps %d fps is not support, so use cap1's setting: %d fps!\n",
				imgsensor.current_fps, imgsensor_info.cap1.max_framerate/10);
		imgsensor.pclk = imgsensor_info.cap.pclk;
		imgsensor.line_length = imgsensor_info.cap.linelength;
		imgsensor.frame_length = imgsensor_info.cap.framelength;
		imgsensor.min_frame_length = imgsensor_info.cap.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
		imgsensor.current_fps = imgsensor_info.cap.max_framerate;
	}
	spin_unlock(&imgsensor_drv_lock);
	capture_setting(imgsensor.current_fps);
	return ERROR_NONE;
}				/* capture() */

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
	/* imgsensor.current_fps = 300; */
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	normal_video_setting(30);
	mdelay(10);


	return ERROR_NONE;
}				/*      normal_video   */

static kal_uint32 hs_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			   MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_HIGH_SPEED_VIDEO;
	imgsensor.pclk = imgsensor_info.hs_video.pclk;
	/* imgsensor.video_mode = KAL_TRUE; */
	imgsensor.line_length = imgsensor_info.hs_video.linelength;
	imgsensor.frame_length = imgsensor_info.hs_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.hs_video.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	/* imgsensor.current_fps = 300; */
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	hs_video_setting();
	return ERROR_NONE;
}				/*      hs_video   */

static kal_uint32 slim_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			     MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_SLIM_VIDEO;
	imgsensor.pclk = imgsensor_info.slim_video.pclk;
	/* imgsensor.video_mode = KAL_TRUE; */
	imgsensor.line_length = imgsensor_info.slim_video.linelength;
	imgsensor.frame_length = imgsensor_info.slim_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.slim_video.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	/* imgsensor.current_fps = 300; */
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	slim_video_setting();
	return ERROR_NONE;
}				/*      slim_video       */



static kal_uint32 get_resolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *sensor_resolution)
{
	LOG_INF("E\n");
	sensor_resolution->SensorFullWidth = imgsensor_info.cap.grabwindow_width;
	sensor_resolution->SensorFullHeight = imgsensor_info.cap.grabwindow_height;

	sensor_resolution->SensorPreviewWidth = imgsensor_info.pre.grabwindow_width;
	sensor_resolution->SensorPreviewHeight = imgsensor_info.pre.grabwindow_height;

	sensor_resolution->SensorVideoWidth = imgsensor_info.normal_video.grabwindow_width;
	sensor_resolution->SensorVideoHeight = imgsensor_info.normal_video.grabwindow_height;


	sensor_resolution->SensorHighSpeedVideoWidth = imgsensor_info.hs_video.grabwindow_width;
	sensor_resolution->SensorHighSpeedVideoHeight = imgsensor_info.hs_video.grabwindow_height;

	sensor_resolution->SensorSlimVideoWidth = imgsensor_info.slim_video.grabwindow_width;
	sensor_resolution->SensorSlimVideoHeight = imgsensor_info.slim_video.grabwindow_height;
	return ERROR_NONE;
}				/*      get_resolution  */

static kal_uint32 get_info(enum MSDK_SCENARIO_ID_ENUM scenario_id,
			   MSDK_SENSOR_INFO_STRUCT *sensor_info,
			   MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("scenario_id = %d\n", scenario_id);

	sensor_info->SensorClockPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorClockFallingPolarity = SENSOR_CLOCK_POLARITY_LOW;	/* not use */
	sensor_info->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;	/* inverse with datasheet */
	sensor_info->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorInterruptDelayLines = 4;	/* not use */
	sensor_info->SensorResetActiveHigh = FALSE;	/* not use */
	sensor_info->SensorResetDelayCount = 5;	/* not use */

	sensor_info->SensroInterfaceType = imgsensor_info.sensor_interface_type;
	sensor_info->MIPIsensorType = imgsensor_info.mipi_sensor_type;
	sensor_info->SettleDelayMode = imgsensor_info.mipi_settle_delay_mode;
	sensor_info->SensorOutputDataFormat = imgsensor_info.sensor_output_dataformat;

	sensor_info->CaptureDelayFrame = imgsensor_info.cap_delay_frame;
	sensor_info->PreviewDelayFrame = imgsensor_info.pre_delay_frame;
	sensor_info->VideoDelayFrame = imgsensor_info.video_delay_frame;
	sensor_info->HighSpeedVideoDelayFrame = imgsensor_info.hs_video_delay_frame;
	sensor_info->SlimVideoDelayFrame = imgsensor_info.slim_video_delay_frame;

	sensor_info->SensorMasterClockSwitch = 0;	/* not use */
	sensor_info->SensorDrivingCurrent = imgsensor_info.isp_driving_current;

	sensor_info->AEShutDelayFrame = imgsensor_info.ae_shut_delay_frame;
	sensor_info->AESensorGainDelayFrame = imgsensor_info.ae_sensor_gain_delay_frame;
	sensor_info->AEISPGainDelayFrame = imgsensor_info.ae_ispGain_delay_frame;
	sensor_info->IHDR_Support = imgsensor_info.ihdr_support;
	sensor_info->IHDR_LE_FirstLine = imgsensor_info.ihdr_le_firstline;
	sensor_info->SensorModeNum = imgsensor_info.sensor_mode_num;

	sensor_info->SensorMIPILaneNumber = imgsensor_info.mipi_lane_num;
	sensor_info->SensorClockFreq = imgsensor_info.mclk;
	sensor_info->SensorClockDividCount = 3;	/* not use */
	sensor_info->SensorClockRisingCount = 0;
	sensor_info->SensorClockFallingCount = 2;	/* not use */
	sensor_info->SensorPixelClockCount = 3;	/* not use */
	sensor_info->SensorDataLatchCount = 2;	/* not use */

	sensor_info->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
	sensor_info->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
	sensor_info->SensorWidthSampling = 0;	/* 0 is default 1x */
	sensor_info->SensorHightSampling = 0;	/* 0 is default 1x */
	sensor_info->SensorPacketECCOrder = 1;

	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;
		sensor_info->DPCM_INFO = imgsensor_dpcm_info_ov9281[0];
		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
		    imgsensor_info.pre.mipi_data_lp2hs_settle_dc;

		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		sensor_info->SensorGrabStartX = imgsensor_info.cap.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.cap.starty;
		sensor_info->DPCM_INFO = imgsensor_dpcm_info_ov9281[1];
		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
		    imgsensor_info.cap.mipi_data_lp2hs_settle_dc;

		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:

		sensor_info->SensorGrabStartX = imgsensor_info.normal_video.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.normal_video.starty;
		sensor_info->DPCM_INFO = imgsensor_dpcm_info_ov9281[2];
		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
		    imgsensor_info.normal_video.mipi_data_lp2hs_settle_dc;

		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		sensor_info->SensorGrabStartX = imgsensor_info.hs_video.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.hs_video.starty;
		sensor_info->DPCM_INFO = imgsensor_dpcm_info_ov9281[3];
		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
		    imgsensor_info.hs_video.mipi_data_lp2hs_settle_dc;

		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		sensor_info->SensorGrabStartX = imgsensor_info.slim_video.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.slim_video.starty;
		sensor_info->DPCM_INFO = imgsensor_dpcm_info_ov9281[4];
		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
		    imgsensor_info.slim_video.mipi_data_lp2hs_settle_dc;

		break;
	default:
		sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;
		sensor_info->DPCM_INFO = imgsensor_dpcm_info_ov9281[0];
		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
		    imgsensor_info.pre.mipi_data_lp2hs_settle_dc;
		break;
	}

	return ERROR_NONE;
}				/*      get_info  */


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
}				/* control() */



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
	if (enable)		/* enable auto flicker */
		imgsensor.autoflicker_en = KAL_TRUE;
	else			/* Cancel Auto flick */
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
		imgsensor.dummy_line = (frame_length > imgsensor_info.pre.framelength) ?
			(frame_length - imgsensor_info.pre.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		set_dummy();
		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		if (framerate == 0)
			return ERROR_NONE;
		frame_length = imgsensor_info.normal_video.pclk / framerate * 10 /
			imgsensor_info.normal_video.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.normal_video.framelength) ?
			(frame_length - imgsensor_info.normal_video.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.normal_video.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		set_dummy();
		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		if (imgsensor.current_fps == imgsensor_info.cap1.max_framerate) {
			frame_length = imgsensor_info.cap1.pclk / framerate * 10 / imgsensor_info.cap1.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.cap1.framelength) ?
				(frame_length - imgsensor_info.cap1.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.cap1.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
		} else {
			if (imgsensor.current_fps != imgsensor_info.cap.max_framerate)
				LOG_INF("Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n",
					framerate, imgsensor_info.cap.max_framerate / 10);
			frame_length = imgsensor_info.cap.pclk / framerate * 10 / imgsensor_info.cap.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.cap.framelength) ?
				(frame_length - imgsensor_info.cap.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.cap.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
		}
		set_dummy();
		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		frame_length = imgsensor_info.hs_video.pclk / framerate * 10 / imgsensor_info.hs_video.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.hs_video.framelength) ?
			(frame_length - imgsensor_info.hs_video.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.hs_video.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		set_dummy();
		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		frame_length = imgsensor_info.slim_video.pclk / framerate * 10 / imgsensor_info.slim_video.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.slim_video.framelength) ?
			(frame_length - imgsensor_info.slim_video.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.slim_video.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		set_dummy();
		break;
	default:  /* coding with  preview scenario by default */
		frame_length = imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.pre.framelength) ?
			(frame_length - imgsensor_info.pre.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		set_dummy();
		LOG_INF("error scenario_id = %d, we use preview scenario\n", scenario_id);
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
		/* 0x5E00[8]: 1 enable,  0 disable */
		/* 0x5E00[1:0]; 00 Color bar, 01 Random Data, 10 Square, 11 BLACK */
		write_cmos_sensor(0x5E00, 0x80);
		write_cmos_sensor(0x5000, 0x7e);
		write_cmos_sensor(0x5002, 0x00);
		write_cmos_sensor(0x3500, 0x00);
		write_cmos_sensor(0x3501, 0x2f);
		write_cmos_sensor(0x3502, 0xf0);
		write_cmos_sensor(0x3508, 0x00);
		write_cmos_sensor(0x3509, 0x80);
	} else {
		/* 0x5E00[8]: 1 enable,  0 disable */
		/* 0x5E00[1:0]; 00 Color bar, 01 Random Data, 10 Square, 11 BLACK */
		write_cmos_sensor(0x5E00, 0x00);
	}

	spin_lock(&imgsensor_drv_lock);
	imgsensor.test_pattern = enable;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}

static kal_uint32 feature_control(MSDK_SENSOR_FEATURE_ENUM feature_id,
				  UINT8 *feature_para, UINT32 *feature_para_len)
{
	UINT16 *feature_return_para_16 = (UINT16 *) feature_para;
	UINT16 *feature_data_16 = (UINT16 *) feature_para;
	UINT32 *feature_return_para_32 = (UINT32 *) feature_para;
	UINT32 *feature_data_32 = (UINT32 *) feature_para;
	unsigned long long *feature_data = (unsigned long long *)feature_para;
/* unsigned long long *feature_return_para=(unsigned long long *) feature_para; */

	struct SENSOR_WINSIZE_INFO_STRUCT *wininfo;
	MSDK_SENSOR_REG_INFO_STRUCT *sensor_reg_data = (MSDK_SENSOR_REG_INFO_STRUCT *) feature_para;

	/* LOG_INF("feature_id = %d\n", feature_id); */
	switch (feature_id) {
				// huangjiwu for  captrue black --begin	
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
							case MSDK_SCENARIO_ID_CUSTOM4:		
								default:			
								*feature_return_para_32 = 1; /*BINNING_AVERAGED*/			
								break;	
						}		
					pr_debug("SENSOR_FEATURE_GET_BINNING_TYPE AE_binning_type:%d,\n",*feature_return_para_32);		
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
				// huangjiwu for  captrue black --end	
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
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:

			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= imgsensor_info.pre.pclk;
			break;
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
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= (imgsensor_info.pre.framelength << 16)
				+ imgsensor_info.pre.linelength;
			break;
		default:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.pre.framelength << 16)
				+ imgsensor_info.pre.linelength;
			break;
		}
		break;
	case SENSOR_FEATURE_GET_PERIOD:
		*feature_return_para_16++ = imgsensor.line_length;
		*feature_return_para_16 = imgsensor.frame_length;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
		*feature_return_para_32 = imgsensor.pclk;
		*feature_para_len = 4;
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
	case SENSOR_FEATURE_SET_ESHUTTER:
		set_shutter(*feature_data);
		break;
	case SENSOR_FEATURE_SET_NIGHTMODE:
		night_mode((BOOL) (*feature_data));
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
		/* get the lens driver ID from EEPROM or just return LENS_DRIVER_ID_DO_NOT_CARE */
		/* if EEPROM does not exist in camera module. */
		*feature_return_para_32 = LENS_DRIVER_ID_DO_NOT_CARE;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_SET_VIDEO_MODE:
		set_video_mode(*feature_data);
		break;
	case SENSOR_FEATURE_CHECK_SENSOR_ID:
		get_imgsensor_id(feature_return_para_32);
		break;
	case SENSOR_FEATURE_SET_AUTO_FLICKER_MODE:
		set_auto_flicker_mode((BOOL) *feature_data_16, *(feature_data_16 + 1));
		break;
	case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
		set_max_framerate_by_scenario((enum MSDK_SCENARIO_ID_ENUM)*feature_data, *(feature_data+1));
		break;
	case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
		get_default_framerate_by_scenario(
			(enum MSDK_SCENARIO_ID_ENUM)*(feature_data),
			(MUINT32 *)(uintptr_t)(*(feature_data+1)));
		break;
	case SENSOR_FEATURE_SET_TEST_PATTERN:
		set_test_pattern_mode((BOOL)*feature_data);
		break;
	case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE: /*for factory mode auto testing*/
		*feature_return_para_32 = imgsensor_info.checksum_value;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_SET_FRAMERATE:
		LOG_INF("current fps :%d\n", *feature_data_32);
		spin_lock(&imgsensor_drv_lock);
		imgsensor.current_fps = (UINT16)*feature_data_32;
		spin_unlock(&imgsensor_drv_lock);
		break;
	case SENSOR_FEATURE_SET_HDR:
		LOG_INF("ihdr enable :%d\n", *feature_data_32);
		spin_lock(&imgsensor_drv_lock);
		imgsensor.ihdr_en = (UINT8)*feature_data_32;
		spin_unlock(&imgsensor_drv_lock);
		break;
	case SENSOR_FEATURE_GET_CROP_INFO:
		LOG_INF("SENSOR_FEATURE_GET_CROP_INFO scenarioId:%d\n", (UINT32)*feature_data);

		wininfo = (struct SENSOR_WINSIZE_INFO_STRUCT *)(uintptr_t)(*(feature_data+1));

		switch (*feature_data_32) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[1], sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[2], sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[3], sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[4], sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[0], sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		}
		break;
	case SENSOR_FEATURE_SET_IHDR_SHUTTER_GAIN:
		LOG_INF("SENSOR_SET_SENSOR_IHDR LE=%d, SE=%d, Gain=%d\n",
			(UINT16)*feature_data, (UINT16)*(feature_data+1),
			(UINT16)*(feature_data+2));
		ihdr_write_shutter_gain((UINT16)*feature_data, (UINT16)*(feature_data+1), (UINT16)*(feature_data+2));
		break;
	case SENSOR_FEATURE_SET_SHUTTER_FRAME_TIME:
		set_shutter_frame_length((UINT16) (*feature_data), (UINT16) (*(feature_data + 1)));
		break;
	case SENSOR_FEATURE_GET_TEMPERATURE_VALUE:
		LOG_INF("This sensor can't support temperature get\n");
		break;
	case SENSOR_FEATURE_SET_STREAMING_SUSPEND:
		LOG_INF("SENSOR_FEATURE_SET_STREAMING_SUSPEND\n");
		streaming_control(KAL_FALSE);
		break;
	case SENSOR_FEATURE_SET_STREAMING_RESUME:
		LOG_INF("SENSOR_FEATURE_SET_STREAMING_RESUME\n");
		streaming_control(KAL_TRUE);
		break;
	default:
		break;
	}

	return ERROR_NONE;
}				/*    feature_control()  */


static struct SENSOR_FUNCTION_STRUCT sensor_func = {
	open,
	get_info,
	get_resolution,
	feature_control,
	control,
	close
};

UINT32 OV9281_MIPI_RAW_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc)
{
	/* To Do : Check Sensor status here */
	if (pfFunc != NULL)
		*pfFunc = &sensor_func;
	return ERROR_NONE;
}				/*      OV5693_MIPI_RAW_SensorInit      */
