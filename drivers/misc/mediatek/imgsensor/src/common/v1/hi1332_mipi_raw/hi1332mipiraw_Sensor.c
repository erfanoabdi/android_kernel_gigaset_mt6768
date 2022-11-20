/*
 * Copyright (C) 2018 MediaTek Inc.
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

#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/atomic.h>
#include <linux/types.h>

#include "hi1332mipiraw_Sensor.h"

#define PFX "hi1332_camera_sensor"
#define LOG_INF(format, args...)    \
	pr_debug(PFX "[%s] " format, __func__, ##args)

//PDAF
#define ENABLE_PDAF 0
#define e2prom 0

#define per_frame 0
#define HI1332_MIRROR_FLIP_ENABLE 0

//extern bool read_hi1332_eeprom( kal_uint16 addr, BYTE *data, kal_uint32 size); 
//extern bool read_eeprom( kal_uint16 addr, BYTE * data, kal_uint32 size);

#define MULTI_WRITE 1
static DEFINE_SPINLOCK(imgsensor_drv_lock);

static struct imgsensor_info_struct imgsensor_info = { 
	.sensor_id = HI1332_SENSOR_ID,
	
	.checksum_value = 0x4f1b1d5e,       //0x6d01485c // Auto Test Mode ÃßÈÄ..
	.pre = {
        .pclk = 352000000,              //record different mode's pclk
        .linelength = 4800,             //record different mode's linelength
        .framelength = 2400,            //record different mode's framelength
        .startx =0,                     //record different mode's startx of grabwindow
        .starty = 0,                    //record different mode's starty of grabwindow
        .grabwindow_width = 2104,       //record different mode's width of grabwindow
        .grabwindow_height = 1560,      //record different mode's height of grabwindow
        /*   following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario   */
        .mipi_data_lp2hs_settle_dc = 85,
        /*   following for GetDefaultFramerateByScenario()  */
        .max_framerate = 300,
		.mipi_pixel_rate = 285600000, //(714M*4/10)
	},
	.cap = {
        .pclk = 352000000,
        .linelength = 4800,
        .framelength = 3258,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 4208,
        .grabwindow_height = 3120,
        .mipi_data_lp2hs_settle_dc = 85,
        .max_framerate = 230,
		.mipi_pixel_rate = 571200000, //(1428M * 4 / 10 )
	},
	// need to setting
    .cap1 = {                            
		.pclk = 600000000,
		.linelength = 12000,	 
		.framelength = 3328,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4208,
		.grabwindow_height = 3120,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 150,
		.mipi_pixel_rate = 285600000,//(714M*4/10)
    },
	.normal_video = {
        .pclk = 352000000,
        .linelength = 4800,
        .framelength = 3258,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 4208,
        .grabwindow_height = 3120,
        .mipi_data_lp2hs_settle_dc = 85,
        .max_framerate = 230,
		.mipi_pixel_rate = 571200000, //(1428M * 4 / 10 )
	},
	.hs_video = {
        .pclk = 352000000,
        .linelength = 4800,
        .framelength = 2404,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 640 ,
        .grabwindow_height = 480 ,
        .mipi_data_lp2hs_settle_dc = 85,//unit , ns
        .max_framerate = 300,
		.mipi_pixel_rate = 95200000, //( 238M*4/10)
	},
    .slim_video = {
		.pclk = 600000000,
		.linelength = 6004,
		.framelength = 832,
		.startx = 0,
		.starty = 0,
    	.grabwindow_width = 1280,
    	.grabwindow_height = 720,
    	.mipi_data_lp2hs_settle_dc = 85,//unit , ns
    	.max_framerate = 1200, 
    	.mipi_pixel_rate = 190400000, //( 467M * 4 / 10 )
    },

	.margin = 6,
	.min_shutter = 6,
	.max_frame_length = 0xffff,
#if per_frame
	.ae_shut_delay_frame = 0,
	.ae_sensor_gain_delay_frame = 1,
	.ae_ispGain_delay_frame = 2,
#else
	.ae_shut_delay_frame = 0,
	.ae_sensor_gain_delay_frame = 1,
	.ae_ispGain_delay_frame = 2,
#endif

	.ihdr_support = 0,      //1, support; 0,not support
	.ihdr_le_firstline = 0,  //1,le first ; 0, se first
	.sensor_mode_num = 5,	  //support sensor mode num

	.cap_delay_frame = 2, 
	.pre_delay_frame = 2, 
	.video_delay_frame = 2, 
	.hs_video_delay_frame = 3,
	.slim_video_delay_frame = 3,

	.isp_driving_current = ISP_DRIVING_6MA,
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
	.mipi_sensor_type = MIPI_OPHY_NCSI2,
	.mipi_settle_delay_mode = MIPI_SETTLEDELAY_AUTO, //0,MIPI_SETTLEDELAY_AUTO; 1,MIPI_SETTLEDELAY_MANNUAL
	#if HI1332_MIRROR_FLIP_ENABLE
	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_Gr,
	#else
	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_Gb,
	#endif
	.mclk = 24,
	.mipi_lane_num = SENSOR_MIPI_4_LANE,
	.i2c_addr_table = {0x40, 0xff},
	.i2c_speed = 400,
};

static struct imgsensor_struct imgsensor = {
	.mirror = IMAGE_NORMAL,
	.sensor_mode = IMGSENSOR_MODE_INIT,
	.shutter = 0x0100,
	.gain = 0xe0,
	.dummy_pixel = 0,
	.dummy_line = 0,
//full size current fps : 24fps for PIP, 30fps for Normal or ZSD
	.current_fps = 300,
	.autoflicker_en = KAL_FALSE,
	.test_pattern = KAL_FALSE,
	.current_scenario_id = MSDK_SCENARIO_ID_CAMERA_PREVIEW,
	.ihdr_en = 0,
	.i2c_write_id = 0x40,
};

/* Sensor output window information */
static struct SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[5] = {
 { 4224, 3120,    16, 0, 4208, 3120,     2104, 1560,     0, 0, 2104, 1560,     0, 0, 2104, 1560},      // preview (2104 x 1560)
 { 4224, 3120,    16, 0, 4208, 3120,     4208, 3120,     0, 0, 4208, 3120,     0, 0, 4208, 3120},      // capture (4208 x 3120)
 { 4224, 3120,    16, 0, 4208, 3120,     4208, 3120,     0, 0, 4208, 3120,     0, 0, 4208, 3120},      // video   (4208 x 3120)
 { 3856, 2880,    16, 0, 3840, 2880,     640,  480,      0, 0,  640,  480,     0, 0,  640, 480},      // hight speed video (640 x 480)
 { 3856, 2160,    16, 0, 3840, 2160,     1280,  720,     0, 0, 1280,  720,     0, 0, 1280,  720},        // slim video (1280 x 720)
};


#if ENABLE_PDAF

static struct SENSOR_VC_INFO_STRUCT SENSOR_VC_INFO[3]=
{
	/* Preview mode setting */
	 {0x02, //VC_Num
	  0x0a, //VC_PixelNum	
	  0x00, //ModeSelect	/* 0:auto 1:direct */
	  0x00, //EXPO_Ratio	/* 1/1, 1/2, 1/4, 1/8 */
	  0x00, //0DValue		/* 0D Value */
	  0x00, //RG_STATSMODE	/* STATS divistion mode 0:16x16  1:8x8	2:4x4  3:1x1 */
	  0x00, 0x2B, 0x0838, 0x0618,	// VC0 Maybe image data?
	  0x00, 0x00, 0x0000, 0x0000,	// VC1 MVHDR
	  0x01, 0x30, 0x0000, 0x0000,   // VC2 PDAF
	  0x00, 0x00, 0x0000, 0x0000},	// VC3 ??
	/* Capture mode setting */ 
	 {0x02, //VC_Num
	  0x0a, //VC_PixelNum	
	  0x00, //ModeSelect	/* 0:auto 1:direct */
	  0x00, //EXPO_Ratio	/* 1/1, 1/2, 1/4, 1/8 */
	  0x00, //0DValue		/* 0D Value */
	  0x00, //RG_STATSMODE	/* STATS divistion mode 0:16x16  1:8x8	2:4x4  3:1x1 */
	  0x00, 0x2B, 0x1070, 0x0C30,	// VC0 Maybe image data?
	  0x00, 0x00, 0x0000, 0x0000,	// VC1 MVHDR
	  0x01, 0x30, 0x0140, 0x0300,   // VC2 PDAF
	  0x00, 0x00, 0x0000, 0x0000},	// VC3 ??
	/* Video mode setting */
	 {0x02, //VC_Num
	  0x0a, //VC_PixelNum	
	  0x00, //ModeSelect	/* 0:auto 1:direct */
	  0x00, //EXPO_Ratio	/* 1/1, 1/2, 1/4, 1/8 */
	  0x00, //0DValue		/* 0D Value */
	  0x00, //RG_STATSMODE	/* STATS divistion mode 0:16x16  1:8x8	2:4x4  3:1x1 */
	  0x00, 0x2B, 0x1070, 0x0C30,	// VC0 Maybe image data?
	  0x00, 0x00, 0x0000, 0x0000,	// VC1 MVHDR
	  0x01, 0x30, 0x0140, 0x0300,   // VC2 PDAF
	  0x00, 0x00, 0x0000, 0x0000},	// VC3 ??

};

static struct SET_PD_BLOCK_INFO_T imgsensor_pd_info =
{
	.i4OffsetX	= 56,
	.i4OffsetY	= 24,
	.i4PitchX	= 32,
	.i4PitchY	= 32,
	.i4PairNum	= 8,
	.i4SubBlkW	= 16,
	.i4SubBlkH	= 8,
	.i4BlockNumX = 128,
	.i4BlockNumY = 96,
	#if HI1332_MIRROR_FLIP_ENABLE
	.iMirrorFlip = 3,
	#else
	.iMirrorFlip = 0,
	#endif
	.i4PosR =	{
						{61,24}, {61,40}, {69,36}, {69,52},
						{77,24}, {77,40}, {85,36}, {85,52},
				},
	.i4PosL =	{
						{61,28}, {61,44}, {69,32}, {69,48}, 
						{77,28}, {77,44}, {85,32}, {85,48}, 
				}


};
#endif


#if MULTI_WRITE
#define I2C_BUFFER_LEN 1020

static kal_uint16 hi1332_table_write_cmos_sensor(
					kal_uint16 *para, kal_uint32 len)
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

		if ((I2C_BUFFER_LEN - tosend) < 4 ||
			len == IDX ||
			addr != addr_last) {
			iBurstWriteReg_multi(puSendCmd, tosend,
				imgsensor.i2c_write_id,
				4, imgsensor_info.i2c_speed);

			tosend = 0;
		}
	}
	return 0;
}
#endif

static kal_uint16 read_cmos_sensor(kal_uint32 addr)
{
	kal_uint16 get_byte = 0;
	char pu_send_cmd[2] = {(char)(addr >> 8), (char)(addr & 0xFF) };

	iReadRegI2C(pu_send_cmd, 2, (u8 *)&get_byte, 1, imgsensor.i2c_write_id);

	return get_byte;
}

static void write_cmos_sensor(kal_uint32 addr, kal_uint32 para)
{
	char pu_send_cmd[4] = {(char)(addr >> 8),
		(char)(addr & 0xFF), (char)(para >> 8), (char)(para & 0xFF)};

	iWriteRegI2C(pu_send_cmd, 4, imgsensor.i2c_write_id);
}

static void write_cmos_sensor_8(kal_uint32 addr, kal_uint32 para)
{
	char pu_send_cmd[4] = {(char)(addr >> 8),
		(char)(addr & 0xFF), (char)(para & 0xFF)};

	iWriteRegI2C(pu_send_cmd, 3, imgsensor.i2c_write_id);
}

static void write_cmos_sensor1D(kal_uint32 addr, kal_uint32 para)
{
    char pu_send_cmd[3] = {(char)(addr >> 8), (char)(addr & 0xFF), (char)(para & 0xFF)};
    iWriteRegI2C(pu_send_cmd, 3, imgsensor.i2c_write_id);
}
static void set_dummy(void)
{
	LOG_INF("dummyline = %d, dummypixels = %d\n",
		imgsensor.dummy_line, imgsensor.dummy_pixel);
	write_cmos_sensor(0x0006, imgsensor.frame_length); 
	write_cmos_sensor(0x0008, imgsensor.line_length);

}	/*	set_dummy  */

//static kal_uint32 return_sensor_id(void)
//{
//	return ((read_cmos_sensor(0x0716) << 8) | read_cmos_sensor(0x0717));

//}


static void set_max_framerate(UINT16 framerate, kal_bool min_framelength_en)
{
	kal_uint32 frame_length = imgsensor.frame_length;

	frame_length = imgsensor.pclk / framerate * 10 / imgsensor.line_length;
	spin_lock(&imgsensor_drv_lock);
	imgsensor.frame_length = (frame_length > imgsensor.min_frame_length) ?
			frame_length : imgsensor.min_frame_length;
	imgsensor.dummy_line = imgsensor.frame_length -
		imgsensor.min_frame_length;

	if (imgsensor.frame_length > imgsensor_info.max_frame_length) {
		imgsensor.frame_length = imgsensor_info.max_frame_length;
		imgsensor.dummy_line = imgsensor.frame_length -
			imgsensor.min_frame_length;
	}
	if (min_framelength_en)
		imgsensor.min_frame_length = imgsensor.frame_length;

	spin_unlock(&imgsensor_drv_lock);
	set_dummy();
}	/*	set_max_framerate  */

static void write_shutter(kal_uint32 shutter)
{
	kal_uint32 realtime_fps = 0;

	spin_lock(&imgsensor_drv_lock);

	if (shutter > imgsensor.min_frame_length - imgsensor_info.margin)
		imgsensor.frame_length = shutter + imgsensor_info.margin;
	else
		imgsensor.frame_length = imgsensor.min_frame_length;
	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
		imgsensor.frame_length = imgsensor_info.max_frame_length;
	spin_unlock(&imgsensor_drv_lock);

	LOG_INF("shutter = %d, imgsensor.frame_length = %d, imgsensor.min_frame_length = %d\n",
		shutter, imgsensor.frame_length, imgsensor.min_frame_length);


	shutter = (shutter < imgsensor_info.min_shutter) ?
		imgsensor_info.min_shutter : shutter;
	shutter = (shutter >
		(imgsensor_info.max_frame_length - imgsensor_info.margin)) ?
		(imgsensor_info.max_frame_length - imgsensor_info.margin) :
		shutter;
	if (imgsensor.autoflicker_en) {
		realtime_fps = imgsensor.pclk /
			(imgsensor.line_length * imgsensor.frame_length) * 10;
		if (realtime_fps >= 297 && realtime_fps <= 305)
			set_max_framerate(296, 0);
		else if (realtime_fps >= 147 && realtime_fps <= 150)
			set_max_framerate(146, 0);
		else
		    write_cmos_sensor(0x0006, imgsensor.frame_length);

	} else{
		    write_cmos_sensor(0x0006, imgsensor.frame_length);
	}

    write_cmos_sensor1D(0x0003, (shutter & 0x0F0000) >> 16 );
	write_cmos_sensor1D(0x0004, (shutter & 0x00FF00) >> 8 );
	write_cmos_sensor1D(0x0005, (shutter & 0x0000FF) );

	LOG_INF("frame_length = %d , shutter = %d \n", imgsensor.frame_length, shutter);


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

	LOG_INF("set_shutter");
	spin_lock_irqsave(&imgsensor_drv_lock, flags);
	imgsensor.shutter = shutter;
	spin_unlock_irqrestore(&imgsensor_drv_lock, flags);

	write_shutter(shutter);
}	/*	set_shutter */


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
static kal_uint16 gain2reg(kal_uint16 gain)
{
    kal_uint16 reg_gain = 0x0000;
    reg_gain = gain / 4 - 16;

    return (kal_uint16)reg_gain;

}


static kal_uint16 set_gain(kal_uint16 gain)
{
	kal_uint16 reg_gain;

    /* 0x350A[0:1], 0x350B[0:7] AGC real gain */
    /* [0:3] = N meams N /16 X    */
    /* [4:9] = M meams M X         */
    /* Total gain = M + N /16 X   */

    if (gain < BASEGAIN || gain > 16 * BASEGAIN) {
        LOG_INF("Error gain setting");

        if (gain < BASEGAIN)
            gain = BASEGAIN;
        else if (gain > 16 * BASEGAIN)
            gain = 16 * BASEGAIN;
    }

    reg_gain = gain2reg(gain);
    spin_lock(&imgsensor_drv_lock);
    imgsensor.gain = reg_gain;
    spin_unlock(&imgsensor_drv_lock);
    LOG_INF("gain = %d , reg_gain = 0x%x\n ", gain, reg_gain);
	
    reg_gain = reg_gain & 0x00FF;
    write_cmos_sensor(0x003A,reg_gain);
	return gain;

}

#if 0
static void ihdr_write_shutter_gain(kal_uint16 le,
				kal_uint16 se, kal_uint16 gain)
{
	LOG_INF("le:0x%x, se:0x%x, gain:0x%x\n", le, se, gain);
	if (imgsensor.ihdr_en) {
		spin_lock(&imgsensor_drv_lock);
		if (le > imgsensor.min_frame_length - imgsensor_info.margin)
			imgsensor.frame_length = le + imgsensor_info.margin;
		else
			imgsensor.frame_length = imgsensor.min_frame_length;
		if (imgsensor.frame_length > imgsensor_info.max_frame_length)
			imgsensor.frame_length =
				imgsensor_info.max_frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (le < imgsensor_info.min_shutter)
			le = imgsensor_info.min_shutter;
		if (se < imgsensor_info.min_shutter)
			se = imgsensor_info.min_shutter;
		// Extend frame length first
		write_cmos_sensor(0x0006, imgsensor.frame_length);
		write_cmos_sensor(0x3502, (le << 4) & 0xFF);
		write_cmos_sensor(0x3501, (le >> 4) & 0xFF);
		write_cmos_sensor(0x3500, (le >> 12) & 0x0F);
		write_cmos_sensor(0x3508, (se << 4) & 0xFF);
		write_cmos_sensor(0x3507, (se >> 4) & 0xFF);
		write_cmos_sensor(0x3506, (se >> 12) & 0x0F);
		set_gain(gain);
	}
}
#endif


#if 0
static void set_mirror_flip(kal_uint8 image_mirror)
{
	LOG_INF("image_mirror = %d", image_mirror);

	switch (image_mirror) {
	case IMAGE_NORMAL:
		write_cmos_sensor(0x0000, 0x0000);
		break;
	case IMAGE_H_MIRROR:
		write_cmos_sensor(0x0000, 0x0100);

		break;
	case IMAGE_V_MIRROR:
		write_cmos_sensor(0x0000, 0x0200);

		break;
	case IMAGE_HV_MIRROR:
		write_cmos_sensor(0x0000, 0x0300);

		break;
	default:
		LOG_INF("Error image_mirror setting");
		break;
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
}	/*	night_mode	*/

#if MULTI_WRITE
kal_uint16 addr_data_pair_init_hi1332[] = {
	
	//	0x0a00, 0x0000,
		0x2ffe, 0xe000,
		0x3048, 0x0140,
		0x404a, 0x0000,
		0x304c, 0x370b,
		0x404c, 0xf808,
		0x304e, 0x0088,
		0x404e, 0x0000,
		0x3050, 0xFB80,
		0x4050, 0x0300,
		0x0c00, 0x1190,
		0x0c02, 0x0011,
		0x0c04, 0x0000,
		0x0c06, 0x01b0,
		0x0c10, 0x0040,
		0x0c12, 0x0040,
		0x0c14, 0x0040,
		0x0c16, 0x0040,
		0x0c18, 0x8000,
		0x0000, 0x0100,
		0x003c, 0x0001,
		0x000a, 0x0000,
		0x0322, 0x0101,
		0x0012, 0x0000,
		0x0018, 0x108f,
		0x0804, 0x0008,
		0x0008, 0x0258,
		0x0024, 0x002e,
		0x002a, 0x003d,
		0x0026, 0x0040,
		0x002c, 0x0c6f,
		0x005c, 0x0202,
		0x002e, 0x1111,
		0x0032, 0x1111,
		0x0006, 0x0CBA,
		0x0a0e, 0x0001,
		0x0a10, 0x400c,
		0x0a12, 0x1070,
		0x0a14, 0x0c30,
		0x003e, 0x0000,
		0x0004, 0x0cfa,
		0x0054, 0x012c,
		0x0a02, 0x0100,
		0x0a04, 0x014a,
		0x0046, 0x0000,
		0x003a, 0x0000,
		0x0050, 0x0300,
		0x0722, 0x0300,
		0x0756, 0x003f,
		0x005e, 0xf000,
		0x0060, 0x0000,
		0x0062, 0x0000,
		0x0208, 0x0c30,
		0x021c, 0x0003,
		0x021e, 0x0535,
		0x051a, 0x0100,
		0x0518, 0x0200,
		0x0900, 0x0300,
		0x0928, 0x0000,
		0x0902, 0xc31a,
		0x0914, 0xc105,
		0x0916, 0x0414,
		0x0918, 0x0205,
		0x091a, 0x0406,
		0x091c, 0x0c04,
		0x091e, 0x0a0a,
		0x090c, 0x0855,
		0x090e, 0x0026,
		0x004c, 0x0100,
//		0x0800, 0x0000,

};
#endif

static void sensor_init(void)
{
#if MULTI_WRITE
	hi1332_table_write_cmos_sensor(
		addr_data_pair_init_hi1332,
		sizeof(addr_data_pair_init_hi1332) /
		sizeof(kal_uint16));
#else
//		write_cmos_sensor(0x0a00, 0x0000);

		write_cmos_sensor(0x2ffe, 0xe000);
		write_cmos_sensor(0x3048, 0x0140);
		write_cmos_sensor(0x404a, 0x0000);
		write_cmos_sensor(0x304c, 0x370b);
		write_cmos_sensor(0x404c, 0xf808);
		write_cmos_sensor(0x304e, 0x0088);
		write_cmos_sensor(0x404e, 0x0000);
		write_cmos_sensor(0x3050, 0xFB80);
		write_cmos_sensor(0x4050, 0x0300);
		write_cmos_sensor(0x0c00, 0x1190);
		write_cmos_sensor(0x0c02, 0x0011);
		write_cmos_sensor(0x0c04, 0x0000);
		write_cmos_sensor(0x0c06, 0x01b0);
		write_cmos_sensor(0x0c10, 0x0040);
		write_cmos_sensor(0x0c12, 0x0040);
		write_cmos_sensor(0x0c14, 0x0040);
		write_cmos_sensor(0x0c16, 0x0040);
		write_cmos_sensor(0x0c18, 0x8000);
		write_cmos_sensor(0x0000, 0x0100);
		write_cmos_sensor(0x003c, 0x0001);
		write_cmos_sensor(0x000a, 0x0000);
		write_cmos_sensor(0x0322, 0x0101);
		write_cmos_sensor(0x0012, 0x0000);
		write_cmos_sensor(0x0018, 0x108f);
		write_cmos_sensor(0x0804, 0x0008);
		write_cmos_sensor(0x0008, 0x0258);
		write_cmos_sensor(0x0024, 0x002e);
		write_cmos_sensor(0x002a, 0x003d);
		write_cmos_sensor(0x0026, 0x0040);
		write_cmos_sensor(0x002c, 0x0c6f);
		write_cmos_sensor(0x005c, 0x0202);
		write_cmos_sensor(0x002e, 0x1111);
		write_cmos_sensor(0x0032, 0x1111);
		write_cmos_sensor(0x0006, 0x0CBA);
		write_cmos_sensor(0x0a0e, 0x0001);
		write_cmos_sensor(0x0a10, 0x400c);
		write_cmos_sensor(0x0a12, 0x1070);
		write_cmos_sensor(0x0a14, 0x0c30);
		write_cmos_sensor(0x003e, 0x0000);
		write_cmos_sensor(0x0004, 0x0cfa);
		write_cmos_sensor(0x0054, 0x012c);
		write_cmos_sensor(0x0a02, 0x0100);
		write_cmos_sensor(0x0a04, 0x014a);
		write_cmos_sensor(0x0046, 0x0000);
		write_cmos_sensor(0x003a, 0x0000);
		write_cmos_sensor(0x0050, 0x0300);
		write_cmos_sensor(0x0722, 0x0300);
		write_cmos_sensor(0x0756, 0x003f);
		write_cmos_sensor(0x005e, 0xf000);
		write_cmos_sensor(0x0060, 0x0000);
		write_cmos_sensor(0x0062, 0x0000);
		write_cmos_sensor(0x0208, 0x0c30);
		write_cmos_sensor(0x021c, 0x0003);
		write_cmos_sensor(0x021e, 0x0535);
		write_cmos_sensor(0x051a, 0x0100);
		write_cmos_sensor(0x0518, 0x0200);
		write_cmos_sensor(0x0900, 0x0300);
		write_cmos_sensor(0x0928, 0x0000);
		write_cmos_sensor(0x0902, 0xc31a);
		write_cmos_sensor(0x0914, 0xc105);
		write_cmos_sensor(0x0916, 0x0414);
		write_cmos_sensor(0x0918, 0x0205);
		write_cmos_sensor(0x091a, 0x0406);
		write_cmos_sensor(0x091c, 0x0c04);
		write_cmos_sensor(0x091e, 0x0a0a);
		write_cmos_sensor(0x090c, 0x0855);
		write_cmos_sensor(0x090e, 0x0026);
		write_cmos_sensor(0x004c, 0x0100);
		write_cmos_sensor(0x0800, 0x0000);

//		write_cmos_sensor(0x0a00, 0x0100);
#endif
}

#if MULTI_WRITE
kal_uint16 addr_data_pair_preview_hi1332[] = {
	//LOG_INF("E preview\n"); 
//	0x0a00, 0x0000,
	0x404c, 0xf828,
	0x000a, 0x0000,
	0x0012, 0x0000,
	0x0018, 0x108f,
	0x0804, 0x0004,
	0x0024, 0x002c,
	0x002a, 0x003b,
	0x0026, 0x0040,
	0x002c, 0x0c6f,
	0x005c, 0x0404,
	0x002e, 0x3311,
	0x0032, 0x3311,
	0x0a0e, 0x0002,
	0x0a12, 0x0838,
	0x0a14, 0x0618,
	0x0062, 0x0000,
	0x0a04, 0x016a,
	0x0050, 0x0300,
	0x0722, 0x0301,
	0x0756, 0x003f,
	0x0208, 0x0618,
	0x0006, 0x0960,
	0x0008, 0x0258,
	0x0928, 0x0100,
	0x0914, 0xc103,
	0x0916, 0x0207,
	0x0918, 0x0202,
	0x091a, 0x0304,
	0x091c, 0x0803,
	0x091e, 0x0506,
	0x090c, 0x041d,
	0x090e, 0x0008,
	0x0800, 0x0400,
	//0x0a00, 0x0100,
};
#endif

static void preview_setting(void)
{
#if MULTI_WRITE
	hi1332_table_write_cmos_sensor(
		addr_data_pair_preview_hi1332,
		sizeof(addr_data_pair_preview_hi1332) /
		sizeof(kal_uint16));
#else
	LOG_INF("E preview\n"); 
//	write_cmos_sensor(0x0a00, 0x0000);

	write_cmos_sensor(0x404c, 0xf828);
	write_cmos_sensor(0x000a, 0x0000);
	write_cmos_sensor(0x0012, 0x0000);
	write_cmos_sensor(0x0018, 0x108f);
	write_cmos_sensor(0x0804, 0x0004);
	write_cmos_sensor(0x0024, 0x002c);
	write_cmos_sensor(0x002a, 0x003b);
	write_cmos_sensor(0x0026, 0x0040);
	write_cmos_sensor(0x002c, 0x0c6f);
	write_cmos_sensor(0x005c, 0x0404);
	write_cmos_sensor(0x002e, 0x3311);
	write_cmos_sensor(0x0032, 0x3311);
	write_cmos_sensor(0x0a0e, 0x0002);
	write_cmos_sensor(0x0a12, 0x0838);
	write_cmos_sensor(0x0a14, 0x0618);
	write_cmos_sensor(0x0062, 0x0000);
	write_cmos_sensor(0x0a04, 0x016a);
	write_cmos_sensor(0x0050, 0x0300);
	write_cmos_sensor(0x0722, 0x0301);
	write_cmos_sensor(0x0756, 0x003f);
	write_cmos_sensor(0x0208, 0x0618);
	write_cmos_sensor(0x0006, 0x0960);
	write_cmos_sensor(0x0008, 0x0258);
	write_cmos_sensor(0x0928, 0x0100);
	write_cmos_sensor(0x0914, 0xc103);
	write_cmos_sensor(0x0916, 0x0207);
	write_cmos_sensor(0x0918, 0x0202);
	write_cmos_sensor(0x091a, 0x0304);
	write_cmos_sensor(0x091c, 0x0803);
	write_cmos_sensor(0x091e, 0x0506);
	write_cmos_sensor(0x090c, 0x041d);
	write_cmos_sensor(0x090e, 0x0008);
	write_cmos_sensor(0x0800, 0x0400);

//	write_cmos_sensor(0x0a00, 0x0100);
#endif
}

#if MULTI_WRITE
kal_uint16 addr_data_pair_capture_30fps_hi1332[] = {
//0x0a00, 0x0000,
0x404c, 0xf808,
0x000a, 0x0000,
0x0012, 0x0000,
0x0018, 0x108f,
0x0804, 0x0008,
0x0024, 0x002e,
0x002a, 0x003d,
0x0026, 0x0040,
0x002c, 0x0c6f,
0x005c, 0x0202,
0x002e, 0x1111,
0x0032, 0x1111,
0x0a0e, 0x0001,
0x0a12, 0x1070,
0x0a14, 0x0c30,
0x0062, 0x0000,
0x0a04, 0x014a,
0x0050, 0x0300,
0x0722, 0x0300,
0x0756, 0x003f,
0x0208, 0x0c30,
0x0006, 0x0CBA,
0x0008, 0x0258,
0x0928, 0x0000,
0x0914, 0xc105,
0x0916, 0x0414,
0x0918, 0x0205,
0x091a, 0x0406,
0x091c, 0x0c04,
0x091e, 0x0a0a,
0x090c, 0x0855,
0x090e, 0x0026,
0x0800, 0x0000,
//0x0a00, 0x0100,
};

kal_uint16 addr_data_pair_capture_15fps_hi1332[] = {
//0x0a00, 0x0000,
0x404c, 0xf808,
0x000a, 0x0000,
0x0012, 0x0000,
0x0018, 0x108f,
0x0804, 0x0008,
0x0024, 0x002e,
0x002a, 0x003d,
0x0026, 0x0040,
0x002c, 0x0c6f,
0x005c, 0x0202,
0x002e, 0x1111,
0x0032, 0x1111,
0x0a0e, 0x0001,
0x0a12, 0x1070,
0x0a14, 0x0c30,
0x0062, 0x0000,
0x0a04, 0x014a,
0x0050, 0x0300,
0x0722, 0x0300,
0x0756, 0x003f,
0x0208, 0x0c30,
0x0006, 0x0CBA,
0x0008, 0x0258,
0x0928, 0x0000,
0x0914, 0xc105,
0x0916, 0x0414,
0x0918, 0x0205,
0x091a, 0x0406,
0x091c, 0x0c04,
0x091e, 0x0a0a,
0x090c, 0x0855,
0x090e, 0x0026,
0x0800, 0x0000,
//0x0a00, 0x0100,


	};
#endif


static void capture_setting(kal_uint16 currefps)
{
#if MULTI_WRITE
	if (currefps == 300) {
	hi1332_table_write_cmos_sensor(
		addr_data_pair_capture_30fps_hi1332,
		sizeof(addr_data_pair_capture_30fps_hi1332) /
		sizeof(kal_uint16));

	} else {
	hi1332_table_write_cmos_sensor(
		addr_data_pair_capture_15fps_hi1332,
		sizeof(addr_data_pair_capture_15fps_hi1332) /
		sizeof(kal_uint16));
	}
#else
  if( currefps == 300) {
//write_cmos_sensor(0x0a00, 0x0000);

write_cmos_sensor(0x404c, 0xf808);
write_cmos_sensor(0x000a, 0x0000);
write_cmos_sensor(0x0012, 0x0000);
write_cmos_sensor(0x0018, 0x108f);
write_cmos_sensor(0x0804, 0x0008);
write_cmos_sensor(0x0024, 0x002e);
write_cmos_sensor(0x002a, 0x003d);
write_cmos_sensor(0x0026, 0x0040);
write_cmos_sensor(0x002c, 0x0c6f);
write_cmos_sensor(0x005c, 0x0202);
write_cmos_sensor(0x002e, 0x1111);
write_cmos_sensor(0x0032, 0x1111);
write_cmos_sensor(0x0a0e, 0x0001);
write_cmos_sensor(0x0a12, 0x1070);
write_cmos_sensor(0x0a14, 0x0c30);
write_cmos_sensor(0x0062, 0x0000);
write_cmos_sensor(0x0a04, 0x014a);
write_cmos_sensor(0x0050, 0x0300);
write_cmos_sensor(0x0722, 0x0300);
write_cmos_sensor(0x0756, 0x003f);
write_cmos_sensor(0x0208, 0x0c30);
write_cmos_sensor(0x0006, 0x0CBA);
write_cmos_sensor(0x0008, 0x0258);
write_cmos_sensor(0x0928, 0x0000);
write_cmos_sensor(0x0914, 0xc105);
write_cmos_sensor(0x0916, 0x0414);
write_cmos_sensor(0x0918, 0x0205);
write_cmos_sensor(0x091a, 0x0406);
write_cmos_sensor(0x091c, 0x0c04);
write_cmos_sensor(0x091e, 0x0a0a);
write_cmos_sensor(0x090c, 0x0855);
write_cmos_sensor(0x090e, 0x0026);
write_cmos_sensor(0x0800, 0x0000);

//write_cmos_sensor(0x0a00, 0x0100);

// PIP
  } else	{
//write_cmos_sensor(0x0a00, 0x0000);

write_cmos_sensor(0x404c, 0xf808);
write_cmos_sensor(0x000a, 0x0000);
write_cmos_sensor(0x0012, 0x0000);
write_cmos_sensor(0x0018, 0x108f);
write_cmos_sensor(0x0804, 0x0008);
write_cmos_sensor(0x0024, 0x002e);
write_cmos_sensor(0x002a, 0x003d);
write_cmos_sensor(0x0026, 0x0040);
write_cmos_sensor(0x002c, 0x0c6f);
write_cmos_sensor(0x005c, 0x0202);
write_cmos_sensor(0x002e, 0x1111);
write_cmos_sensor(0x0032, 0x1111);
write_cmos_sensor(0x0a0e, 0x0001);
write_cmos_sensor(0x0a12, 0x1070);
write_cmos_sensor(0x0a14, 0x0c30);
write_cmos_sensor(0x0062, 0x0000);
write_cmos_sensor(0x0a04, 0x014a);
write_cmos_sensor(0x0050, 0x0300);
write_cmos_sensor(0x0722, 0x0300);
write_cmos_sensor(0x0756, 0x003f);
write_cmos_sensor(0x0208, 0x0c30);
write_cmos_sensor(0x0006, 0x0CBA);
write_cmos_sensor(0x0008, 0x0258);
write_cmos_sensor(0x0928, 0x0000);
write_cmos_sensor(0x0914, 0xc105);
write_cmos_sensor(0x0916, 0x0414);
write_cmos_sensor(0x0918, 0x0205);
write_cmos_sensor(0x091a, 0x0406);
write_cmos_sensor(0x091c, 0x0c04);
write_cmos_sensor(0x091e, 0x0a0a);
write_cmos_sensor(0x090c, 0x0855);
write_cmos_sensor(0x090e, 0x0026);
write_cmos_sensor(0x0800, 0x0000);

//write_cmos_sensor(0x0a00, 0x0100);
	}
#endif
}

#if MULTI_WRITE
kal_uint16 addr_data_pair_video_hi1332[] = {
//0x0a00, 0x0000,
0x404c, 0xf808,
0x000a, 0x0000,
0x0012, 0x0000,
0x0018, 0x108f,
0x0804, 0x0008,
0x0024, 0x002e,
0x002a, 0x003d,
0x0026, 0x0040,
0x002c, 0x0c6f,
0x005c, 0x0202,
0x002e, 0x1111,
0x0032, 0x1111,
0x0a0e, 0x0001,
0x0a12, 0x1070,
0x0a14, 0x0c30,
0x0062, 0x0000,
0x0a04, 0x014a,
0x0050, 0x0300,
0x0722, 0x0300,
0x0756, 0x003f,
0x0208, 0x0c30,
0x0006, 0x0CBA,
0x0008, 0x0258,
0x0928, 0x0000,
0x0914, 0xc105,
0x0916, 0x0414,
0x0918, 0x0205,
0x091a, 0x0406,
0x091c, 0x0c04,
0x091e, 0x0a0a,
0x090c, 0x0855,
0x090e, 0x0026,
0x0800, 0x0000,
//0x0a00, 0x0100,
};
#endif

static void normal_video_setting(void)
{
#if MULTI_WRITE
	hi1332_table_write_cmos_sensor(
		addr_data_pair_video_hi1332,
		sizeof(addr_data_pair_video_hi1332) /
		sizeof(kal_uint16));
#else
//write_cmos_sensor(0x0a00, 0x0000);

write_cmos_sensor(0x404c, 0xf808);
write_cmos_sensor(0x000a, 0x0000);
write_cmos_sensor(0x0012, 0x0000);
write_cmos_sensor(0x0018, 0x108f);
write_cmos_sensor(0x0804, 0x0008);
write_cmos_sensor(0x0024, 0x002e);
write_cmos_sensor(0x002a, 0x003d);
write_cmos_sensor(0x0026, 0x0040);
write_cmos_sensor(0x002c, 0x0c6f);
write_cmos_sensor(0x005c, 0x0202);
write_cmos_sensor(0x002e, 0x1111);
write_cmos_sensor(0x0032, 0x1111);
write_cmos_sensor(0x0a0e, 0x0001);
write_cmos_sensor(0x0a12, 0x1070);
write_cmos_sensor(0x0a14, 0x0c30);
write_cmos_sensor(0x0062, 0x0000);
write_cmos_sensor(0x0a04, 0x014a);
write_cmos_sensor(0x0050, 0x0300);
write_cmos_sensor(0x0722, 0x0300);
write_cmos_sensor(0x0756, 0x003f);
write_cmos_sensor(0x0208, 0x0c30);
write_cmos_sensor(0x0006, 0x0CBA);
write_cmos_sensor(0x0008, 0x0258);
write_cmos_sensor(0x0928, 0x0000);
write_cmos_sensor(0x0914, 0xc105);
write_cmos_sensor(0x0916, 0x0414);
write_cmos_sensor(0x0918, 0x0205);
write_cmos_sensor(0x091a, 0x0406);
write_cmos_sensor(0x091c, 0x0c04);
write_cmos_sensor(0x091e, 0x0a0a);
write_cmos_sensor(0x090c, 0x0855);
write_cmos_sensor(0x090e, 0x0026);
write_cmos_sensor(0x0800, 0x0000);

//write_cmos_sensor(0x0a00, 0x0100);
#endif
}

#if MULTI_WRITE
kal_uint16 addr_data_pair_hs_video_hi1332[] = {
	0x3250, 0xa470, //sreg8 - RAMP,B[15:10]:d2a_ramp_rng_ctrl
	0x0730, 0x770f, //pll_cfg_mipi_a PLL_CLK=750mhz b7-6:00_isp_div2(1/1) 
	0x0732, 0xe4b0, //pll_cfg_mipi_b b13-11:100_isp_div1(1/5) b10-8:100_mipi_div1(1/6) b1-0:00_mipi_div2(1/1) 
	0x1118, 0x0072, //LSC r_win_y B[11]: Bit8 of y offset in start block when cropping. B[10:8] y start index of block when cropping. B[7:0] y offset in start block when cropping.
	0x1200, 0x011f, //PDPC BYPASS : Dyna-DPC ON, PDEN flag OFF, PD-DPC ON
	0x1204, 0x1c01, //PDPC DC Counting OFF, PD Around BYPASS OFF
	0x1240, 0x0100, //pdpc_pd_cnt_max_value
	0x0b20, 0x8600, //HBIN mode
	0x0f00, 0x1400, //fmt ctrl
	0x103e, 0x0500, //mipi_tx_col_read_ctrl
	0x1020, 0xc102, //mipi_exit_seq, tlpx
	0x1022, 0x0209, //mipi_tclk_prepare, tclk_zero
	0x1024, 0x0303, //mipi_tclk_pre, ths_prepare
	0x1026, 0x0305, //mipi_ths_zero, ths_trail
	0x1028, 0x0903, //mipi_tclk_post, tclk_trail
	0x102a, 0x0404, //mipi_texit, tsync
	0x102c, 0x0400, //mipi_tpd_sync
	0x1010, 0x07d0, //mipi_vblank_delay
	0x1012, 0x0040, //mipi_ch0_hblank_delay
	0x1014, 0xffea, //mipi_hblank_short_delay1
	0x1016, 0xffea, //mipi_hblank_short_delay2
	0x101a, 0xffea, //mipi_pd_hblank_delay
	0x1038, 0x0000, //mipi_virtual_channel_ctrl
	0x1042, 0x0008, //mipi_pd_sep_ctrl_h, mipi_pd_sep_ctrl_l
	0x1048, 0x0080, //mipi_pd_max_col_size
	0x1044, 0x0100, //mipi_pd_col_size
	0x1046, 0x0004, //mipi_pd_row_size
	0x0404, 0x0008, //x addr start active
	0x0406, 0x1087, //x addr end active
	0x0220, 0x000c, //y addr start fobp
	0x022a, 0x0013, //y addr end fobp
	0x0222, 0x0c80, //y addr start dummy
	0x022c, 0x0c89, //y addr end dummy
	0x0224, 0x009c, //y addr start active
	0x022e, 0x0bed, //y addr end active
	0x0f04, 0x0020, //fmt x cropping
	0x0f06, 0x0000, //fmt y cropping
	0x023a, 0x6666, //y dummy size
	0x0234, 0x7755, //y even/odd inc tobp
	0x0238, 0x7755, //y even/odd inc active
	0x0246, 0x0020, //y read dummy address
	0x020a, 0x0339, //coarse integ time
	0x021c, 0x0008, //coarse integ time short for iHDR
	0x0206, 0x05dd, //line length pck
	0x020e, 0x0340, //frame length lines
	0x0b12, 0x0280, //x output size
	0x0b14, 0x01e0, //y output size
	0x0204, 0x0200, //d2a_row_binning_en
	0x041c, 0x0048, //pdaf patch start x-address 
	0x041e, 0x1047, //pdaf patch end x-address 
	0x0b04, 0x037e, //isp enable
	0x027e, 0x0100, //tg enable
};
#endif

static void hs_video_setting(void)
{

#if MULTI_WRITE
	hi1332_table_write_cmos_sensor(
		addr_data_pair_hs_video_hi1332,
		sizeof(addr_data_pair_hs_video_hi1332) /
		sizeof(kal_uint16));
#else
// setting ver5.0 None PD 640x480 120fps
	write_cmos_sensor(0x3250, 0xa470); //sreg8 - RAMP,B[15:10]:d2a_ramp_rng_ctrl
	write_cmos_sensor(0x0730, 0x770f); //pll_cfg_mipi_a PLL_CLK=750mhz b7-6:00_isp_div2(1/1) 
	write_cmos_sensor(0x0732, 0xe4b0); //pll_cfg_mipi_b b13-11:100_isp_div1(1/5) b10-8:100_mipi_div1(1/6) b1-0:00_mipi_div2(1/1) 
	write_cmos_sensor(0x1118, 0x0072); //LSC r_win_y B[11]: Bit8 of y offset in start block when cropping. B[10:8] y start index of block when cropping. B[7:0] y offset in start block when cropping.
	write_cmos_sensor(0x1200, 0x011f); //PDPC BYPASS : Dyna-DPC ON, PDEN flag OFF, PD-DPC ON
	write_cmos_sensor(0x1204, 0x1c01); //PDPC DC Counting OFF, PD Around BYPASS OFF
	write_cmos_sensor(0x1240, 0x0100); //pdpc_pd_cnt_max_value
	write_cmos_sensor(0x0b20, 0x8600); //HBIN mode
	write_cmos_sensor(0x0f00, 0x1400); //fmt ctrl
	write_cmos_sensor(0x103e, 0x0500); //mipi_tx_col_read_ctrl
	write_cmos_sensor(0x1020, 0xc102); //mipi_exit_seq, tlpx
	write_cmos_sensor(0x1022, 0x0209); //mipi_tclk_prepare, tclk_zero
	write_cmos_sensor(0x1024, 0x0303); //mipi_tclk_pre, ths_prepare
	write_cmos_sensor(0x1026, 0x0305); //mipi_ths_zero, ths_trail
	write_cmos_sensor(0x1028, 0x0903); //mipi_tclk_post, tclk_trail
	write_cmos_sensor(0x102a, 0x0404); //mipi_texit, tsync
	write_cmos_sensor(0x102c, 0x0400); //mipi_tpd_sync
	write_cmos_sensor(0x1010, 0x07d0); //mipi_vblank_delay
	write_cmos_sensor(0x1012, 0x0040); //mipi_ch0_hblank_delay
	write_cmos_sensor(0x1014, 0xffea); //mipi_hblank_short_delay1
	write_cmos_sensor(0x1016, 0xffea); //mipi_hblank_short_delay2
	write_cmos_sensor(0x101a, 0xffea); //mipi_pd_hblank_delay
	write_cmos_sensor(0x1038, 0x0000); //mipi_virtual_channel_ctrl
	write_cmos_sensor(0x1042, 0x0008); //mipi_pd_sep_ctrl_h, mipi_pd_sep_ctrl_l
	write_cmos_sensor(0x1048, 0x0080); //mipi_pd_max_col_size
	write_cmos_sensor(0x1044, 0x0100); //mipi_pd_col_size
	write_cmos_sensor(0x1046, 0x0004); //mipi_pd_row_size
	write_cmos_sensor(0x0404, 0x0008); //x addr start active
	write_cmos_sensor(0x0406, 0x1087); //x addr end active
	write_cmos_sensor(0x0220, 0x000c); //y addr start fobp
	write_cmos_sensor(0x022a, 0x0013); //y addr end fobp
	write_cmos_sensor(0x0222, 0x0c80); //y addr start dummy
	write_cmos_sensor(0x022c, 0x0c89); //y addr end dummy
	write_cmos_sensor(0x0224, 0x009c); //y addr start active
	write_cmos_sensor(0x022e, 0x0bed); //y addr end active
	write_cmos_sensor(0x0f04, 0x0020); //fmt x cropping
	write_cmos_sensor(0x0f06, 0x0000); //fmt y cropping
	write_cmos_sensor(0x023a, 0x6666); //y dummy size
	write_cmos_sensor(0x0234, 0x7755); //y even/odd inc tobp
	write_cmos_sensor(0x0238, 0x7755); //y even/odd inc active
	write_cmos_sensor(0x0246, 0x0020); //y read dummy address
	write_cmos_sensor(0x020a, 0x0339); //coarse integ time
	write_cmos_sensor(0x021c, 0x0008); //coarse integ time short for iHDR
	write_cmos_sensor(0x0206, 0x05dd); //line length pck
	write_cmos_sensor(0x020e, 0x0340); //frame length lines
	write_cmos_sensor(0x0b12, 0x0280); //x output size
	write_cmos_sensor(0x0b14, 0x01e0); //y output size
	write_cmos_sensor(0x0204, 0x0200); //d2a_row_binning_en
	write_cmos_sensor(0x041c, 0x0048); //pdaf patch start x-address 
	write_cmos_sensor(0x041e, 0x1047); //pdaf patch end x-address 
	write_cmos_sensor(0x0b04, 0x037e); //isp enable
	write_cmos_sensor(0x027e, 0x0100); //tg enable
#endif
}

#if MULTI_WRITE
kal_uint16 addr_data_pair_slim_video_hi1332[] = {
	0x3250, 0xa060, //sreg8 - RAMP,B[15:10]:d2a_ramp_rng_ctrl
	0x0730, 0x770f, //pll_cfg_mipi_a PLL_CLK=750mhz b7-6:00_isp_div2(1/1) 
	0x0732, 0xe2b0, //pll_cfg_mipi_b b13-11:100_isp_div1(1/5) b10-8:010_mipi_div1(1/3) b1-0:00_mipi_div2(1/1) 
	0x1118, 0x01a8, //LSC r_win_y B[11]: Bit8 of y offset in start block when cropping. B[10:8] y start index of block when cropping. B[7:0] y offset in start block when cropping.
	0x1200, 0x011f, //PDPC BYPASS : Dyna-DPC ON, PDEN flag OFF, PD-DPC ON
	0x1204, 0x1c01, //PDPC DC Counting OFF, PD Around BYPASS OFF
	0x1240, 0x0100, //pdpc_pd_cnt_max_value
	0x0b20, 0x8300, //HBIN mode
	0x0f00, 0x0800, //fmt ctrl
	0x103e, 0x0200, //mipi_tx_col_read_ctrl
	0x1020, 0xc104, //mipi_exit_seq, tlpx
	0x1022, 0x0410, //mipi_tclk_prepare, tclk_zero
	0x1024, 0x0304, //mipi_tclk_pre, ths_prepare
	0x1026, 0x0507, //mipi_ths_zero, ths_trail
	0x1028, 0x0d05, //mipi_tclk_post, tclk_trail
	0x102a, 0x0704, //mipi_texit, tsync
	0x102c, 0x1400, //mipi_tpd_sync
	0x1010, 0x07d0, //mipi_vblank_delay
	0x1012, 0x009c, //mipi_ch0_hblank_delay
	0x1014, 0x0013, //mipi_hblank_short_delay1
	0x1016, 0x0013, //mipi_hblank_short_delay2
	0x101a, 0x0013, //mipi_pd_hblank_delay
	0x1038, 0x0000, //mipi_virtual_channel_ctrl
	0x1042, 0x0008, //mipi_pd_sep_ctrl_h, mipi_pd_sep_ctrl_l
	0x1048, 0x0080, //mipi_pd_max_col_size
	0x1044, 0x0100, //mipi_pd_col_size
	0x1046, 0x0004, //mipi_pd_row_size
	0x0404, 0x0008, //x addr start active
	0x0406, 0x1087, //x addr end active
	0x0220, 0x0008, //y addr start fobp
	0x022a, 0x0017, //y addr end fobp
	0x0222, 0x0c80, //y addr start dummy
	0x022c, 0x0c89, //y addr end dummy
	0x0224, 0x020a, //y addr start active
	0x022e, 0x0a83, //y addr end active
	0x0f04, 0x0040, //fmt x cropping
	0x0f06, 0x0000, //fmt y cropping
	0x023a, 0x3333, //y dummy size
	0x0234, 0x3333, //y even/odd inc tobp
	0x0238, 0x3333, //y even/odd inc active
	0x0246, 0x0020, //y read dummy address
	0x020a, 0x0339, //coarse integ time
	0x021c, 0x0008, //coarse integ time short for iHDR
	0x0206, 0x05dd, //line length pck
	0x020e, 0x0340, //frame length lines
	0x0b12, 0x0500, //x output size
	0x0b14, 0x02d0, //y output size
	0x0204, 0x0000, //d2a_row_binning_en
	0x041c, 0x0048, //pdaf patch start x-address 
	0x041e, 0x1047, //pdaf patch end x-address 
	0x0b04, 0x037e, //isp enable
	0x027e, 0x0100, //tg enable

};
#endif


static void slim_video_setting(void)
{

#if MULTI_WRITE
	hi1332_table_write_cmos_sensor(
		addr_data_pair_slim_video_hi1332,
		sizeof(addr_data_pair_slim_video_hi1332) /
		sizeof(kal_uint16));
#else
	write_cmos_sensor(0x3250, 0xa060); //sreg8 - RAMP,B[15:10]:d2a_ramp_rng_ctrl
	write_cmos_sensor(0x0730, 0x770f); //pll_cfg_mipi_a PLL_CLK=750mhz b7-6:00_isp_div2(1/1) 
	write_cmos_sensor(0x0732, 0xe2b0); //pll_cfg_mipi_b b13-11:100_isp_div1(1/5) b10-8:010_mipi_div1(1/3) b1-0:00_mipi_div2(1/1) 
	write_cmos_sensor(0x1118, 0x01a8); //LSC r_win_y B[11]: Bit8 of y offset in start block when cropping. B[10:8] y start index of block when cropping. B[7:0] y offset in start block when cropping.
	write_cmos_sensor(0x1200, 0x011f); //PDPC BYPASS : Dyna-DPC ON, PDEN flag OFF, PD-DPC ON
	write_cmos_sensor(0x1204, 0x1c01); //PDPC DC Counting OFF, PD Around BYPASS OFF
	write_cmos_sensor(0x1240, 0x0100); //pdpc_pd_cnt_max_value
	write_cmos_sensor(0x0b20, 0x8300); //HBIN mode
	write_cmos_sensor(0x0f00, 0x0800); //fmt ctrl
	write_cmos_sensor(0x103e, 0x0200); //mipi_tx_col_read_ctrl
	write_cmos_sensor(0x1020, 0xc104); //mipi_exit_seq, tlpx
	write_cmos_sensor(0x1022, 0x0410); //mipi_tclk_prepare, tclk_zero
	write_cmos_sensor(0x1024, 0x0304); //mipi_tclk_pre, ths_prepare
	write_cmos_sensor(0x1026, 0x0507); //mipi_ths_zero, ths_trail
	write_cmos_sensor(0x1028, 0x0d05); //mipi_tclk_post, tclk_trail
	write_cmos_sensor(0x102a, 0x0704); //mipi_texit, tsync
	write_cmos_sensor(0x102c, 0x1400); //mipi_tpd_sync
	write_cmos_sensor(0x1010, 0x07d0); //mipi_vblank_delay
	write_cmos_sensor(0x1012, 0x009c); //mipi_ch0_hblank_delay
	write_cmos_sensor(0x1014, 0x0013); //mipi_hblank_short_delay1
	write_cmos_sensor(0x1016, 0x0013); //mipi_hblank_short_delay2
	write_cmos_sensor(0x101a, 0x0013); //mipi_pd_hblank_delay
	write_cmos_sensor(0x1038, 0x0000); //mipi_virtual_channel_ctrl
	write_cmos_sensor(0x1042, 0x0008); //mipi_pd_sep_ctrl_h, mipi_pd_sep_ctrl_l
	write_cmos_sensor(0x1048, 0x0080); //mipi_pd_max_col_size
	write_cmos_sensor(0x1044, 0x0100); //mipi_pd_col_size
	write_cmos_sensor(0x1046, 0x0004); //mipi_pd_row_size
	write_cmos_sensor(0x0404, 0x0008); //x addr start active
	write_cmos_sensor(0x0406, 0x1087); //x addr end active
	write_cmos_sensor(0x0220, 0x0008); //y addr start fobp
	write_cmos_sensor(0x022a, 0x0017); //y addr end fobp
	write_cmos_sensor(0x0222, 0x0c80); //y addr start dummy
	write_cmos_sensor(0x022c, 0x0c89); //y addr end dummy
	write_cmos_sensor(0x0224, 0x020a); //y addr start active
	write_cmos_sensor(0x022e, 0x0a83); //y addr end active
	write_cmos_sensor(0x0f04, 0x0040); //fmt x cropping
	write_cmos_sensor(0x0f06, 0x0000); //fmt y cropping
	write_cmos_sensor(0x023a, 0x3333); //y dummy size
	write_cmos_sensor(0x0234, 0x3333); //y even/odd inc tobp
	write_cmos_sensor(0x0238, 0x3333); //y even/odd inc active
	write_cmos_sensor(0x0246, 0x0020); //y read dummy address
	write_cmos_sensor(0x020a, 0x0339); //coarse integ time
	write_cmos_sensor(0x021c, 0x0008); //coarse integ time short for iHDR
	write_cmos_sensor(0x0206, 0x05dd); //line length pck
	write_cmos_sensor(0x020e, 0x0340); //frame length lines
	write_cmos_sensor(0x0b12, 0x0500); //x output size
	write_cmos_sensor(0x0b14, 0x02d0); //y output size
	write_cmos_sensor(0x0204, 0x0000); //d2a_row_binning_en
	write_cmos_sensor(0x041c, 0x0048); //pdaf patch start x-address 
	write_cmos_sensor(0x041e, 0x1047); //pdaf patch end x-address 
	write_cmos_sensor(0x0b04, 0x037e); //isp enable
	write_cmos_sensor(0x027e, 0x0100); //tg enable


#endif
}

static kal_uint32 get_imgsensor_id(UINT32 *sensor_id)
{
	kal_uint8 i = 0;
	kal_uint8 retry = 2;

	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
            spin_lock(&imgsensor_drv_lock);
            imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
            spin_unlock(&imgsensor_drv_lock);
            do {
                *sensor_id = ((read_cmos_sensor(0x0F17) << 8) | read_cmos_sensor(0x0F16));

                if (*sensor_id == imgsensor_info.sensor_id) {
                    LOG_INF("i2c write id  : 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id,*sensor_id);
                    return ERROR_NONE;
            }
                LOG_INF("get_imgsensor_id Read sensor id fail, id: 0x%x 0x%x\n", imgsensor.i2c_write_id,*sensor_id);
                retry--;
            } while(retry > 0);
            i++;
            retry = 2;
}
	if (*sensor_id != imgsensor_info.sensor_id) {
		LOG_INF("Read id fail,sensor id: 0x%x\n", *sensor_id);
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

	LOG_INF("[open]: hi1332@MT6765,MIPI 4LANE\n");

	while (imgsensor_info.i2c_addr_table[i] != 0xff) {

        spin_lock(&imgsensor_drv_lock);
        imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
        spin_unlock(&imgsensor_drv_lock);
        LOG_INF("SP\n");
        do {
            sensor_id = ((read_cmos_sensor(0x0F17) << 8) | read_cmos_sensor(0x0F16));
            if (sensor_id == imgsensor_info.sensor_id) {
                LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id,sensor_id);
                break;
            }
            LOG_INF("open:Read sensor id 0x%x fail open w, id: 0x%x\n", imgsensor.i2c_write_id,sensor_id);
            retry--;
        } while(retry > 0);
        i++;
        if (sensor_id == imgsensor_info.sensor_id)
            break;
        retry = 2;
    }
	if (imgsensor_info.sensor_id != sensor_id) {
		LOG_INF("open sensor id fail: 0x%x\n", sensor_id);
		return ERROR_SENSOR_CONNECT_FAIL;
	}
	/* initail sequence write in  */
	sensor_init();

	spin_lock(&imgsensor_drv_lock);
	imgsensor.autoflicker_en = KAL_FALSE;
	imgsensor.sensor_mode = IMGSENSOR_MODE_INIT;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	imgsensor.frame_length = imgsensor_info.pre.framelength;
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.dummy_pixel = 0;
	imgsensor.dummy_line = 0;
	imgsensor.ihdr_en = 0;
	imgsensor.test_pattern = KAL_FALSE;
	imgsensor.current_fps = imgsensor_info.pre.max_framerate;
	//imgsensor.pdaf_mode = 1;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}	/*	open  */
static kal_uint32 close(void)
{
	return ERROR_NONE;
}	/*	close  */


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
	LOG_INF("E");
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_PREVIEW;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.frame_length = imgsensor_info.pre.framelength;
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.current_fps = imgsensor_info.pre.max_framerate;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	preview_setting();
	return ERROR_NONE;
}	/*	preview   */

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
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CAPTURE;

	if (imgsensor.current_fps == imgsensor_info.cap.max_framerate)	{
		imgsensor.pclk = imgsensor_info.cap.pclk;
		imgsensor.line_length = imgsensor_info.cap.linelength;
		imgsensor.frame_length = imgsensor_info.cap.framelength;
		imgsensor.min_frame_length = imgsensor_info.cap.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	} else {
	 //PIP capture: 24fps for less than 13M, 20fps for 16M,15fps for 20M
		imgsensor.pclk = imgsensor_info.cap1.pclk;
		imgsensor.line_length = imgsensor_info.cap1.linelength;
		imgsensor.frame_length = imgsensor_info.cap1.framelength;
		imgsensor.min_frame_length = imgsensor_info.cap1.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	}

	spin_unlock(&imgsensor_drv_lock);
	LOG_INF("Caputre fps:%d\n", imgsensor.current_fps);
	capture_setting(imgsensor.current_fps);

	return ERROR_NONE;

}	/* capture() */
static kal_uint32 normal_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_VIDEO;
	imgsensor.pclk = imgsensor_info.normal_video.pclk;
	imgsensor.line_length = imgsensor_info.normal_video.linelength;
	imgsensor.frame_length = imgsensor_info.normal_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.normal_video.framelength;
	imgsensor.current_fps = 300;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	normal_video_setting();
	return ERROR_NONE;
}	/*	normal_video   */

static kal_uint32 hs_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_HIGH_SPEED_VIDEO;
	imgsensor.pclk = imgsensor_info.hs_video.pclk;
	//imgsensor.video_mode = KAL_TRUE;
	imgsensor.line_length = imgsensor_info.hs_video.linelength;
	imgsensor.frame_length = imgsensor_info.hs_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.hs_video.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	hs_video_setting();
	return ERROR_NONE;
}    /*    hs_video   */

static kal_uint32 slim_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
		      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_SLIM_VIDEO;
	imgsensor.pclk = imgsensor_info.slim_video.pclk;
	imgsensor.line_length = imgsensor_info.slim_video.linelength;
	imgsensor.frame_length = imgsensor_info.slim_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.slim_video.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	slim_video_setting();

	return ERROR_NONE;
}    /*    slim_video     */

static kal_uint32 get_resolution(
		MSDK_SENSOR_RESOLUTION_INFO_STRUCT * sensor_resolution)
{
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
}    /*    get_resolution    */


static kal_uint32 get_info(enum MSDK_SCENARIO_ID_ENUM scenario_id,
			MSDK_SENSOR_INFO_STRUCT *sensor_info,
			MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("scenario_id = %d\n", scenario_id);

	sensor_info->SensorClockPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorClockFallingPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorInterruptDelayLines = 4; /* not use */
	sensor_info->SensorResetActiveHigh = FALSE; /* not use */
	sensor_info->SensorResetDelayCount = 5; /* not use */

	sensor_info->SensroInterfaceType =
	imgsensor_info.sensor_interface_type;
	sensor_info->MIPIsensorType = imgsensor_info.mipi_sensor_type;
	sensor_info->SettleDelayMode = imgsensor_info.mipi_settle_delay_mode;
	sensor_info->SensorOutputDataFormat =
		imgsensor_info.sensor_output_dataformat;

	sensor_info->CaptureDelayFrame = imgsensor_info.cap_delay_frame;
	sensor_info->PreviewDelayFrame = imgsensor_info.pre_delay_frame;
	sensor_info->VideoDelayFrame =
		imgsensor_info.video_delay_frame;
	sensor_info->HighSpeedVideoDelayFrame =
		imgsensor_info.hs_video_delay_frame;
	sensor_info->SlimVideoDelayFrame =
		imgsensor_info.slim_video_delay_frame;

	sensor_info->SensorMasterClockSwitch = 0; /* not use */
	sensor_info->SensorDrivingCurrent =
		imgsensor_info.isp_driving_current;
/* The frame of setting shutter default 0 for TG int */
	sensor_info->AEShutDelayFrame =
		imgsensor_info.ae_shut_delay_frame;
/* The frame of setting sensor gain */
	sensor_info->AESensorGainDelayFrame =
		imgsensor_info.ae_sensor_gain_delay_frame;
	sensor_info->AEISPGainDelayFrame =
		imgsensor_info.ae_ispGain_delay_frame;
	sensor_info->IHDR_Support = imgsensor_info.ihdr_support;
	sensor_info->IHDR_LE_FirstLine =
		imgsensor_info.ihdr_le_firstline;
	sensor_info->SensorModeNum =
		imgsensor_info.sensor_mode_num;

	sensor_info->SensorMIPILaneNumber =
		imgsensor_info.mipi_lane_num;
	sensor_info->SensorClockFreq = imgsensor_info.mclk;
	sensor_info->SensorClockDividCount = 3; /* not use */
	sensor_info->SensorClockRisingCount = 0;
	sensor_info->SensorClockFallingCount = 2; /* not use */
	sensor_info->SensorPixelClockCount = 3; /* not use */
	sensor_info->SensorDataLatchCount = 2; /* not use */

	sensor_info->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
	sensor_info->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
	sensor_info->SensorWidthSampling = 0;  // 0 is default 1x
	sensor_info->SensorHightSampling = 0;    // 0 is default 1x
	sensor_info->SensorPacketECCOrder = 1;

#if ENABLE_PDAF
	sensor_info->PDAF_Support = PDAF_SUPPORT_CAMSV;
#endif

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
	    sensor_info->SensorGrabStartX = imgsensor_info.normal_video.startx;
	    sensor_info->SensorGrabStartY = imgsensor_info.normal_video.starty;

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
	    sensor_info->SensorGrabStartX = imgsensor_info.slim_video.startx;
	    sensor_info->SensorGrabStartY = imgsensor_info.slim_video.starty;
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
}    /*    get_info  */


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
		LOG_INF("preview\n");
		preview(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		LOG_INF("capture\n");
	//case MSDK_SCENARIO_ID_CAMERA_ZSD:
		capture(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		LOG_INF("video preview\n");
		normal_video(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		hs_video(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
	    slim_video(image_window, sensor_config_data);
		break;
	default:
		LOG_INF("default mode\n");
		preview(image_window, sensor_config_data);
		return ERROR_INVALID_SCENARIO_ID;
	}
	return ERROR_NONE;
}	/* control() */



static kal_uint32 set_video_mode(UINT16 framerate)
{
	LOG_INF("framerate = %d ", framerate);
	// SetVideoMode Function should fix framerate
	if (framerate == 0)
		// Dynamic frame rate
		return ERROR_NONE;
	spin_lock(&imgsensor_drv_lock);

	if ((framerate == 30) && (imgsensor.autoflicker_en == KAL_TRUE))
		imgsensor.current_fps = 296;
	else if ((framerate == 15) && (imgsensor.autoflicker_en == KAL_TRUE))
		imgsensor.current_fps = 146;
	else
		imgsensor.current_fps = 10 * framerate;
	spin_unlock(&imgsensor_drv_lock);
	set_max_framerate(imgsensor.current_fps, 1);
	set_dummy();
	return ERROR_NONE;
}


static kal_uint32 set_auto_flicker_mode(kal_bool enable,
			UINT16 framerate)
{
	LOG_INF("enable = %d, framerate = %d ", enable, framerate);
	spin_lock(&imgsensor_drv_lock);
	if (enable)
		imgsensor.autoflicker_en = KAL_TRUE;
	else //Cancel Auto flick
		imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}


static kal_uint32 set_max_framerate_by_scenario(
			enum MSDK_SCENARIO_ID_ENUM scenario_id,
			MUINT32 framerate)
{
	kal_uint32 frame_length;

	LOG_INF("scenario_id = %d, framerate = %d\n",
				scenario_id, framerate);

	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
	    frame_length = imgsensor_info.pre.pclk / framerate * 10 /
			imgsensor_info.pre.linelength;
	    spin_lock(&imgsensor_drv_lock);
	    imgsensor.dummy_line = (frame_length >
			imgsensor_info.pre.framelength) ?
			(frame_length - imgsensor_info.pre.framelength) : 0;
	    imgsensor.frame_length = imgsensor_info.pre.framelength +
			imgsensor.dummy_line;
	    imgsensor.min_frame_length = imgsensor.frame_length;
	    spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
	break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		if (framerate == 0)
			return ERROR_NONE;
	    frame_length = imgsensor_info.normal_video.pclk /
			framerate * 10 / imgsensor_info.normal_video.linelength;
	    spin_lock(&imgsensor_drv_lock);
	    imgsensor.dummy_line = (frame_length >
			imgsensor_info.normal_video.framelength) ?
		(frame_length - imgsensor_info.normal_video.framelength) : 0;
	    imgsensor.frame_length = imgsensor_info.normal_video.framelength +
			imgsensor.dummy_line;
	    imgsensor.min_frame_length = imgsensor.frame_length;
	    spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
	break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		if (imgsensor.current_fps ==
				imgsensor_info.cap1.max_framerate) {
		frame_length = imgsensor_info.cap1.pclk / framerate * 10 /
				imgsensor_info.cap1.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length >
			imgsensor_info.cap1.framelength) ?
			(frame_length - imgsensor_info.cap1.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.cap1.framelength +
				imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		} else {
			if (imgsensor.current_fps !=
				imgsensor_info.cap.max_framerate)
			LOG_INF("fps %d fps not support,use cap: %d fps!\n",
			framerate, imgsensor_info.cap.max_framerate/10);
			frame_length = imgsensor_info.cap.pclk /
				framerate * 10 / imgsensor_info.cap.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length >
				imgsensor_info.cap.framelength) ?
			(frame_length - imgsensor_info.cap.framelength) : 0;
			imgsensor.frame_length =
				imgsensor_info.cap.framelength +
				imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
		}
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
	break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
	    frame_length = imgsensor_info.hs_video.pclk /
			framerate * 10 / imgsensor_info.hs_video.linelength;
	    spin_lock(&imgsensor_drv_lock);
	    imgsensor.dummy_line = (frame_length >
			imgsensor_info.hs_video.framelength) ? (frame_length -
			imgsensor_info.hs_video.framelength) : 0;
	    imgsensor.frame_length = imgsensor_info.hs_video.framelength +
			imgsensor.dummy_line;
	    imgsensor.min_frame_length = imgsensor.frame_length;
	    spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
	break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
	    frame_length = imgsensor_info.slim_video.pclk /
			framerate * 10 / imgsensor_info.slim_video.linelength;
	    spin_lock(&imgsensor_drv_lock);
	    imgsensor.dummy_line = (frame_length >
			imgsensor_info.slim_video.framelength) ? (frame_length -
			imgsensor_info.slim_video.framelength) : 0;
	    imgsensor.frame_length =
			imgsensor_info.slim_video.framelength +
			imgsensor.dummy_line;
	    imgsensor.min_frame_length = imgsensor.frame_length;
	    spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
	break;
	default:  //coding with  preview scenario by default
	    frame_length = imgsensor_info.pre.pclk / framerate * 10 /
						imgsensor_info.pre.linelength;
	    spin_lock(&imgsensor_drv_lock);
	    imgsensor.dummy_line = (frame_length >
			imgsensor_info.pre.framelength) ?
			(frame_length - imgsensor_info.pre.framelength) : 0;
	    imgsensor.frame_length = imgsensor_info.pre.framelength +
				imgsensor.dummy_line;
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
				enum MSDK_SCENARIO_ID_ENUM scenario_id,
				MUINT32 *framerate)
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
	LOG_INF("set_test_pattern_mode enable: %d", enable);
	if (enable) {
		write_cmos_sensor(0x1038, 0x0000); //mipi_virtual_channel_ctrl
		write_cmos_sensor(0x1042, 0x0008); //mipi_pd_sep_ctrl_h, mipi_pd_sep_ctrl_l
		write_cmos_sensor_8(0x0213, 0x00); //analog gain 1x
		write_cmos_sensor(0x0b04, 0x0141); //bit[0]:test_pattern_enable
		write_cmos_sensor(0x0C0A, 0x0100); // solid bar

	} else {
		write_cmos_sensor(0x1038, 0x4100); //mipi_virtual_channel_ctrl
		write_cmos_sensor(0x1042, 0x0108); //mipi_pd_sep_ctrl_h, mipi_pd_sep_ctrl_l
		write_cmos_sensor(0x0b04, 0x0349);
		write_cmos_sensor(0x0C0A, 0x0000);

	}
	spin_lock(&imgsensor_drv_lock);
	imgsensor.test_pattern = enable;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}

static kal_uint32 streaming_control(kal_bool enable)
{
	pr_debug("streaming_enable(0=Sw Standby,1=streaming): %d\n", enable);

	if (enable)
		write_cmos_sensor(0x0a00, 0x0100); // stream on
	else
		write_cmos_sensor(0x0a00, 0x0000); // stream off

	mdelay(10);
	return ERROR_NONE;
}

static kal_uint32 feature_control(
			MSDK_SENSOR_FEATURE_ENUM feature_id,
			UINT8 *feature_para, UINT32 *feature_para_len)
{
	UINT16 *feature_return_para_16 = (UINT16 *) feature_para;
	UINT16 *feature_data_16 = (UINT16 *) feature_para;
	UINT32 *feature_return_para_32 = (UINT32 *) feature_para;
	UINT32 *feature_data_32 = (UINT32 *) feature_para;
	INT32 *feature_return_para_i32 = (INT32 *) feature_para;

#if ENABLE_PDAF
    struct SET_PD_BLOCK_INFO_T *PDAFinfo;
	struct SENSOR_VC_INFO_STRUCT *pvcinfo;
#endif

	unsigned long long *feature_data =
		(unsigned long long *) feature_para;

	struct SENSOR_WINSIZE_INFO_STRUCT *wininfo;
	MSDK_SENSOR_REG_INFO_STRUCT *sensor_reg_data =
		(MSDK_SENSOR_REG_INFO_STRUCT *) feature_para;

	LOG_INF("feature_id = %d\n", feature_id);
	switch (feature_id) {
	case SENSOR_FEATURE_GET_PERIOD:
	    *feature_return_para_16++ = imgsensor.line_length;
	    *feature_return_para_16 = imgsensor.frame_length;
	    *feature_para_len = 4;
	break;
	case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
	    *feature_return_para_32 = imgsensor.pclk;
	    *feature_para_len = 4;
	break;
	case SENSOR_FEATURE_SET_ESHUTTER:
	    set_shutter(*feature_data);
	break;
	case SENSOR_FEATURE_SET_NIGHTMODE:
	    night_mode((BOOL) * feature_data);
	break;
	case SENSOR_FEATURE_SET_GAIN:
	    set_gain((UINT16) *feature_data);
	break;
	case SENSOR_FEATURE_SET_FLASHLIGHT:
	break;
	case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
	break;
	case SENSOR_FEATURE_SET_REGISTER:
	    write_cmos_sensor(sensor_reg_data->RegAddr,
						sensor_reg_data->RegData);
	break;
	case SENSOR_FEATURE_GET_REGISTER:
	    sensor_reg_data->RegData =
				read_cmos_sensor(sensor_reg_data->RegAddr);
	break;
	case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
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
	    set_auto_flicker_mode((BOOL)*feature_data_16,
			*(feature_data_16+1));
	break;
	case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
	    set_max_framerate_by_scenario(
			(enum MSDK_SCENARIO_ID_ENUM)*feature_data,
			*(feature_data+1));
	break;
	case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
	    get_default_framerate_by_scenario(
			(enum MSDK_SCENARIO_ID_ENUM)*(feature_data),
			(MUINT32 *)(uintptr_t)(*(feature_data+1)));
	break;
	case SENSOR_FEATURE_SET_TEST_PATTERN:
	    set_test_pattern_mode((BOOL)*feature_data);
	break;
	case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE:
	    *feature_return_para_32 = imgsensor_info.checksum_value;
	    *feature_para_len = 4;
	break;
	case SENSOR_FEATURE_SET_FRAMERATE:
	    LOG_INF("current fps :%d\n", (UINT16)*feature_data_32);
	    spin_lock(&imgsensor_drv_lock);
	    imgsensor.current_fps = (UINT16)*feature_data_32;
	    spin_unlock(&imgsensor_drv_lock);
	break;

	case SENSOR_FEATURE_SET_HDR:
	    LOG_INF("ihdr enable :%d\n", (UINT8)*feature_data_32);
	    spin_lock(&imgsensor_drv_lock);
	    imgsensor.ihdr_en = (UINT8)*feature_data_32;
	    spin_unlock(&imgsensor_drv_lock);
	break;
	case SENSOR_FEATURE_GET_CROP_INFO:
	    LOG_INF("SENSOR_FEATURE_GET_CROP_INFO scenarioId:%d\n",
				(UINT32)*feature_data);

	    wininfo = (struct SENSOR_WINSIZE_INFO_STRUCT *)
			(uintptr_t)(*(feature_data+1));

		switch (*feature_data_32) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			memcpy((void *)wininfo,
				(void *)&imgsensor_winsize_info[1],
				sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
		break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			memcpy((void *)wininfo,
				(void *)&imgsensor_winsize_info[2],
				sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
		break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			memcpy((void *)wininfo,
				(void *)&imgsensor_winsize_info[3],
				sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
		break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			memcpy((void *)wininfo,
				(void *)&imgsensor_winsize_info[4],
				sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
		break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			memcpy((void *)wininfo,
				(void *)&imgsensor_winsize_info[0],
				sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
		break;
		}
	break;
	case SENSOR_FEATURE_SET_IHDR_SHUTTER_GAIN:
	    LOG_INF("SENSOR_SET_SENSOR_IHDR LE=%d, SE=%d, Gain=%d\n",
			(UINT16)*feature_data, (UINT16)*(feature_data+1),
			(UINT16)*(feature_data+2));
	#if 0
	    ihdr_write_shutter_gain((UINT16)*feature_data,
			(UINT16)*(feature_data+1), (UINT16)*(feature_data+2));
	#endif
	break;
	case SENSOR_FEATURE_GET_TEMPERATURE_VALUE:
		*feature_return_para_i32 = 0;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_SET_STREAMING_SUSPEND:
		streaming_control(KAL_FALSE);
		break;
	case SENSOR_FEATURE_SET_STREAMING_RESUME:
		if (*feature_data != 0)
			set_shutter(*feature_data);
		streaming_control(KAL_TRUE);
		break;
#if ENABLE_PDAF

		case SENSOR_FEATURE_GET_VC_INFO:
				LOG_INF("SENSOR_FEATURE_GET_VC_INFO %d\n", (UINT16)*feature_data);
				pvcinfo = (struct SENSOR_VC_INFO_STRUCT *)(uintptr_t)(*(feature_data+1));
				switch (*feature_data_32) 
				{
					case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
						LOG_INF("SENSOR_FEATURE_GET_VC_INFO CAPTURE_JPEG\n");
						memcpy((void *)pvcinfo,(void *)&SENSOR_VC_INFO[1],sizeof(struct SENSOR_VC_INFO_STRUCT));
						break;
					case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
						LOG_INF("SENSOR_FEATURE_GET_VC_INFO VIDEO PREVIEW\n");
						memcpy((void *)pvcinfo,(void *)&SENSOR_VC_INFO[2],sizeof(struct SENSOR_VC_INFO_STRUCT));
						break;
					case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
					default:
						LOG_INF("SENSOR_FEATURE_GET_VC_INFO DEFAULT_PREVIEW\n");
						memcpy((void *)pvcinfo,(void *)&SENSOR_VC_INFO[0],sizeof(struct SENSOR_VC_INFO_STRUCT));
						break;
				}
				break;

		case SENSOR_FEATURE_GET_PDAF_DATA:
		LOG_INF(" GET_PDAF_DATA EEPROM\n");
		// read from e2prom
// #if e2prom
		// read_eeprom((kal_uint16)(*feature_data), 
				// (char *)(uintptr_t)(*(feature_data+1)), 
				// (kal_uint32)(*(feature_data+2)) );
// #else
		// // read from file

	        // LOG_INF("READ PDCAL DATA\n");
		// read_hi1332_eeprom((kal_uint16)(*feature_data), 
				// (char *)(uintptr_t)(*(feature_data+1)), 
				// (kal_uint32)(*(feature_data+2)) );

// #endif
		break;
		case SENSOR_FEATURE_GET_PDAF_INFO:
			PDAFinfo= (struct SET_PD_BLOCK_INFO_T *)(uintptr_t)(*(feature_data+1));  
			switch( *feature_data) 
			{
		 		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		 		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
					memcpy((void *)PDAFinfo, (void *)&imgsensor_pd_info, sizeof(struct SET_PD_BLOCK_INFO_T));
					break;
		 		//case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		 		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		 		case MSDK_SCENARIO_ID_SLIM_VIDEO:
		 		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
	 	 		default:
					break;
			}
		break;

	case SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY:
		LOG_INF("SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY scenarioId:%lld\n", *feature_data);
		//PDAF capacity enable or not, 2p8 only full size support PDAF
		switch (*feature_data) {
			case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
				*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1; // type2 - VC enable
				break;
			case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
				*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
				break;
			case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
				*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
				break;
			case MSDK_SCENARIO_ID_SLIM_VIDEO:
				*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
				break;
			case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
				*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
				break;
			default:
				*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
				break;
		}
		LOG_INF("SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY scenarioId:%lld\n", *feature_data);
		break;

	case SENSOR_FEATURE_SET_PDAF:
			 	imgsensor.pdaf_mode = *feature_data_16;
	        	LOG_INF(" pdaf mode : %d \n", imgsensor.pdaf_mode);
				break;
	
#endif

	case SENSOR_FEATURE_GET_PIXEL_RATE:
	switch (*feature_data) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
			(imgsensor_info.cap.pclk /
			(imgsensor_info.cap.linelength - 80))*
			imgsensor_info.cap.grabwindow_width;
			break;

		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
			(imgsensor_info.normal_video.pclk /
			(imgsensor_info.normal_video.linelength - 80))*
			imgsensor_info.normal_video.grabwindow_width;
			break;

		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
			(imgsensor_info.hs_video.pclk /
			(imgsensor_info.hs_video.linelength - 80))*
			imgsensor_info.hs_video.grabwindow_width;
			break;

		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
			(imgsensor_info.slim_video.pclk /
			(imgsensor_info.slim_video.linelength - 80))*
			imgsensor_info.slim_video.grabwindow_width;
			break;

		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
			(imgsensor_info.pre.pclk /
			(imgsensor_info.pre.linelength - 80))*
			imgsensor_info.pre.grabwindow_width;
			break;
	}
	break;

	case SENSOR_FEATURE_GET_MIPI_PIXEL_RATE:
	{
		kal_uint32 rate;
		switch (*feature_data) {
			case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
				 rate =	imgsensor_info.cap.mipi_pixel_rate;
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
				rate = imgsensor_info.pre.mipi_pixel_rate;
			default:
				rate = 0;
				break;
			}
		*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = rate;	
	}
	break;



	default:
	break;
	}

	return ERROR_NONE;
}    /*    feature_control()  */

static struct SENSOR_FUNCTION_STRUCT sensor_func = {
	open,
	get_info,
	get_resolution,
	feature_control,
	control,
	close
};

UINT32 HI1332_MIPI_RAW_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc)
{
	/* To Do : Check Sensor status here */
	if (pfFunc != NULL)
		*pfFunc =  &sensor_func;
	return ERROR_NONE;
}	/*	HI1336_MIPI_RAW_SensorInit	*/
