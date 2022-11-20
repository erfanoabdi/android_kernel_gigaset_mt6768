/*****************************************************************************
 *
 * Filename:
 * ---------
 *   OV50A40_REAR_rear_mipiraw_Sensor.c
 *
 * Project:
 * --------
 *	 ALPS
 *
 * Description:
 * [201501] PDAF MP Version 
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

#include "ov50a40mipiraw_Sensor.h"
#include "ov50a40_Sensor_setting.h"
#define PFX "OV50A40_camera_sensor"
#define LOG_INF(format, args...)    \
	pr_debug(PFX "[%s] " format, __func__, ##args)

#define MULTI_WRITE 1

#define PDAF_SUPPORT 1
#define  OV50A40_MIRROR_FLIP_ENABLE 1
#define  REMOSAIC_SUPPORT 1  // prize add by zhuzhengjiang for disable remosaic 
#define  SENSOR_AGAIN_MAX_256 0  // prize add by zhuzhengjiang for  sensor gain 256 

static DEFINE_SPINLOCK(imgsensor_drv_lock);

static struct imgsensor_info_struct imgsensor_info = {
	.sensor_id = OV50A40_SENSOR_ID,

	.checksum_value = 0x788279ef,

	.pre = {
		.pclk = 75024000,
		.linelength = 600,
		.framelength = 4168,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4096,
		.grabwindow_height = 3072,
		.mipi_data_lp2hs_settle_dc = 85,//unit , ns
		.max_framerate = 300,
		.mipi_pixel_rate = 187200000,
	},
	
	.cap = {
		.pclk = 75024000,
		.linelength = 600,
		.framelength = 4168,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4096,
		.grabwindow_height = 3072,
		.mipi_data_lp2hs_settle_dc = 85,//unit , ns
		.max_framerate = 300,
		.mipi_pixel_rate = 187200000,
	},
		
	.cap1 = {
		.pclk = 75024000,
		.linelength = 600,
		.framelength = 4168,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4096,
		.grabwindow_height = 3072,
		.mipi_data_lp2hs_settle_dc = 85,//unit , ns
		.max_framerate = 300,
		.mipi_pixel_rate = 187200000,
	},
	.normal_video = {
		.pclk = 75012000,
		.linelength = 600,
		.framelength = 4168,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4096,
		.grabwindow_height = 2304,
		.mipi_data_lp2hs_settle_dc = 85,//unit , ns
		.max_framerate = 300,
		.mipi_pixel_rate = 187200000,
	},
	.hs_video = {
		.pclk = 74952000,
		.linelength = 225,
		.framelength = 2776,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 1920,
		.grabwindow_height = 1080,
		.mipi_data_lp2hs_settle_dc = 85,//unit , ns
		.max_framerate = 1200,
		.mipi_pixel_rate = 187200000,
	},
	.slim_video = {
		.pclk = 74952000,
		.linelength = 880,
		.framelength = 1400,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 1920,
		.grabwindow_height = 1080,
		.mipi_data_lp2hs_settle_dc = 85,//unit , ns
		.max_framerate = 600,
		.mipi_pixel_rate = 187200000,
	},
.custom1 = {
		.pclk = 75024000,
		.linelength = 600,
		.framelength = 4168,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4096,
		.grabwindow_height = 3072,
		.mipi_data_lp2hs_settle_dc = 85,//unit , ns
		.max_framerate = 300,
		.mipi_pixel_rate = 187200000,
	},
  .custom2 = {
		.pclk = 75024000,
		.linelength = 600,
		.framelength = 4168,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4096,
		.grabwindow_height = 3072,
		.mipi_data_lp2hs_settle_dc = 85,//unit , ns
		.max_framerate = 300,
		.mipi_pixel_rate = 187200000,
	},
#if REMOSAIC_SUPPORT
  .custom3 = {
		.pclk = 75024000,
		.linelength = 550,
		.framelength = 9088,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 8192,
		.grabwindow_height = 6144,
		.mipi_data_lp2hs_settle_dc = 85,//unit , ns
		.max_framerate = 150,
		.mipi_pixel_rate = 600800000,
	},
#else
  .custom3 = {
		.pclk = 75024000,
		.linelength = 600,
		.framelength = 4168,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4096,
		.grabwindow_height = 3072,
		.mipi_data_lp2hs_settle_dc = 85,//unit , ns
		.max_framerate = 300,
		.mipi_pixel_rate = 187200000,
	},
#endif
  .custom4 = {
		.pclk = 75024000,
		.linelength = 600,
		.framelength = 4168,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4096,
		.grabwindow_height = 3072,
		.mipi_data_lp2hs_settle_dc = 85,//unit , ns
		.max_framerate = 300,
		.mipi_pixel_rate = 187200000,
	},
  .custom5 = {
		.pclk = 75024000,
		.linelength = 600,
		.framelength = 4168,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4096,
		.grabwindow_height = 3072,
		.mipi_data_lp2hs_settle_dc = 85,//unit , ns
		.max_framerate = 300,
		.mipi_pixel_rate = 187200000,
	},

	.min_gain = 64,
#if SENSOR_AGAIN_MAX_256
	.max_gain = 16384,
#else
	.max_gain = 4096,
#endif
	.min_gain_iso = 100,
	.exp_step = 2,
	.gain_step = 2,
	.gain_type = 1,
	.temperature_support=0,

	.margin = 32,		/* sensor framelength & shutter margin check*/
	.min_shutter = 16,	/* min shutter */
	.max_frame_length = 0xffffDF,	/* max framelength by sensor register's limitation */
	.ae_shut_delay_frame = 0,
	.ae_sensor_gain_delay_frame = 0,
	.ae_ispGain_delay_frame = 2,
	.ihdr_support = 0,	  //1, support; 0,not support
	.ihdr_le_firstline = 0,  //1,le first ; 0, se first
	.sensor_mode_num = 10,			//support sensor mode num

	.cap_delay_frame = 3,			//enter capture delay frame num
	.pre_delay_frame = 2,			//enter preview delay frame num
	.video_delay_frame = 2,			//enter video delay frame num
	.hs_video_delay_frame = 2,		//enter high speed video  delay frame num
	.slim_video_delay_frame = 2,	//enter slim video delay frame num
	.custom1_delay_frame = 2,		//enter custom1 delay frame num
	.custom2_delay_frame = 2,		//enter custom2 delay frame num
	.custom3_delay_frame = 2,		//enter custom3 delay frame num
	.custom4_delay_frame = 2,		//enter custom4 delay frame num
	.custom5_delay_frame = 2,		//enter custom5 delay frame num

	.isp_driving_current = ISP_DRIVING_8MA,
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
	.mipi_sensor_type = MIPI_CPHY, //0,MIPI_OPHY_NCSI2;  1,MIPI_OPHY_CSI2
	.mipi_settle_delay_mode = 1,//0,MIPI_SETTLEDELAY_AUTO; 1,MIPI_SETTLEDELAY_MANNUAL
#if OV50A40_MIRROR_FLIP_ENABLE
	#if REMOSAIC_SUPPORT
    	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_4CELL_HW_BAYER_B,//sensor output first pixel color
	#else
    	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_B,//sensor output first pixel color
	#endif
#else
	#if REMOSAIC_SUPPORT
    	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_4CELL_HW_BAYER_R,//sensor output first pixel color
	#else
    	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_R,//sensor output first pixel color
	#endif
#endif
	.mclk = 24,
	
	.mipi_lane_num = SENSOR_MIPI_3_LANE,
	.i2c_addr_table = {0x20,0xff},
	.i2c_speed = 400,// i2c read/write speed
	.mipi_hs_taril_value = 100 , // prize add by zhuzhengjiang for mipi hs_trail_value 20210619
};

static  struct imgsensor_struct imgsensor = {
	.mirror = IMAGE_HV_MIRROR,//the truth is mirror and flip 
	.sensor_mode = IMGSENSOR_MODE_INIT, //IMGSENSOR_MODE enum value,record current sensor mode,such as: INIT, Preview, Capture, Video,High Speed Video, Slim Video
	.shutter = 0x3D0,
	.gain = 0x100,
	.dummy_pixel = 0,
	.dummy_line = 0,
	.current_fps = 300,  //full size current fps : 24fps for PIP, 30fps for Normal or ZSD
	.autoflicker_en = KAL_FALSE,  //auto flicker enable: KAL_FALSE for disable auto flicker, KAL_TRUE for enable auto flicker
	.test_pattern = KAL_FALSE,		//test pattern mode or not. KAL_FALSE for in test pattern mode, KAL_TRUE for normal output
	.current_scenario_id = MSDK_SCENARIO_ID_CAMERA_PREVIEW,//current scenario id
	.ihdr_en = 0,		/* sensor need support LE, SE with HDR feature */
	.i2c_write_id = 0x6c,
	.current_ae_effective_frame = 2, //Long Exp
	.max_shutter = 2000000,
};


/* Sensor output window information */
static struct SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[10] = {
	{ 8192,  6144,  0,  0,  8192,  6144,  4096,  3072,  0000,  0000,  4096,  3072,  0,  0,  4096,  3072},	/*Preview*/
	{ 8192,  6144,  0,  0,  8192,  6144,  4096,  3072,  0000,  0000,  4096,  3072,  0,  0,  4096,  3072},	/*capture*/
	{ 8192,  6144,  0,  0,  8192,  6144,  4096,  3072,  0000,  384,   4096,  2304,  0,  0,  4096,  2304},	/*normal video*/
	{ 8192,  6144,  0,  0,  8192,  6144,  2048,  1536,    64,   228,  1920,  1080,  0,  0,  1920,  1080},	/*hs video*/
	{ 8192,  6144,  0,  0,  8192,  6144,  2048,  1536,    64,   228,  1920,  1080,  0,  0,  1920,  1080},	/*slim video*/

	{ 8192,  6144,  0,  0,  8192,  6144,  4096,  3072,  0000,  0000,  4096,  3072,  0,  0,  4096,  3072},	/*custom1*/
	{ 8192,  6144,  0,  0,  8192,  6144,  4096,  3072,  0000,  0000,  4096,  3072,  0,  0,  4096,  3072},	/*custom2*/
#if REMOSAIC_SUPPORT
	{ 8192,  6144,	0,	0,	8192,  6144,  8192,  6144,	0000,  0000,  8192,  6144,	0,	0,	8192,  6144},	/*custom3 remosaic*/
#else
	{ 8192,  6144,  0,  0,  8192,  6144,  4096,  3072,  0000,  0000,  4096,  3072,  0,  0,  4096,  3072},	/*custom3*/
#endif

	{ 8192,  6144,  0,  0,  8192,  6144,  4096,  3072,  0000,  0000,  4096,  3072,  0,  0,  4096,  3072},	/*custom4*/
	{ 8192,  6144,  0,  0,  8192,  6144,  4096,  3072,  0000,  0000,  4096,  3072,  0,  0,  4096,  3072},	/*custom5*/
};

#if PDAF_SUPPORT
static struct SENSOR_VC_INFO2_STRUCT SENSOR_VC_INFO2[3] = {
	{
		0x01, 0x0a, 0x00, 0x08, 0x40, 0x00,//preview
		{
			{VC_STAGGER_NE, 0x00, 0x2b, 0x1000, 0xC00},
			{VC_PDAF_STATS, 0x01, 0x30, 0x1400, 0x300},
		},
		1
	},
	{
		0x01, 0x0a, 0x00, 0x08, 0x40, 0x00,//capture
		{
			{VC_STAGGER_NE, 0x00, 0x2b, 0x1000, 0xC00},
			{VC_PDAF_STATS, 0x01, 0x30, 0x1400, 0x300},
		},
		1
	},
	{
		0x01, 0x0a, 0x00, 0x08, 0x40, 0x00,//video
		{
			{VC_STAGGER_NE, 0x00, 0x2b, 0x1000, 0x900},
			{VC_PDAF_STATS, 0x01, 0x30, 0x1400, 0x240},
		},
		1
	},
};




/*PD information update*/
static struct SET_PD_BLOCK_INFO_T imgsensor_pd_info = {
	 .i4OffsetX = 0,
	 .i4OffsetY = 0,
	 .i4PitchX = 0,
	 .i4PitchY = 0,
	 .i4PairNum = 0,
	 .i4SubBlkW = 0,
	 .i4SubBlkH = 0,
	 .i4PosL = {{0, 0}},
	 .i4PosR = {{0, 0}},
	 .i4BlockNumX = 0,
	 .i4BlockNumY = 0,
	 .iMirrorFlip = 0,
	 .i4LeFirst =0,
	 .i4Crop = {
	 	{0,0},{0,0},{0,384},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0}
	 },

};
#endif

#if MULTI_WRITE
#define I2C_BUFFER_LEN 765	/*trans# max is 255, each 3 bytes*/
#else
#define I2C_BUFFER_LEN 3
#endif

static kal_uint16 ov50a40_table_write_cmos_sensor(
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
			puSendCmd[tosend++] = (char)(data & 0xFF);
			IDX += 2;
			addr_last = addr;
		}
#if MULTI_WRITE
		if ((I2C_BUFFER_LEN - tosend) < 3 ||
			len == IDX ||
			addr != addr_last) {
			iBurstWriteReg_multi(puSendCmd, tosend,
				imgsensor.i2c_write_id,
				3, imgsensor_info.i2c_speed);
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
	char pusendcmd[2] = {(char)(addr >> 8), (char)(addr & 0xFF) };
	iReadRegI2C(pusendcmd, 2, (u8 *)&get_byte, 1, imgsensor.i2c_write_id);
	return get_byte;
}

static void write_cmos_sensor(kal_uint32 addr, kal_uint32 para)
{
	char pusendcmd[4] = {(char)(addr >> 8),
		(char)(addr & 0xFF), (char)(para & 0xFF)};
	iWriteRegI2C(pusendcmd, 3, imgsensor.i2c_write_id);
}

static void set_dummy(void)
{
    write_cmos_sensor(0x380C, (imgsensor.line_length >> 8) & 0xFF);
    write_cmos_sensor(0x380D, (imgsensor.line_length >> 0) & 0xFF);
    write_cmos_sensor(0x380E, (imgsensor.frame_length >> 8) & 0xFF);
    write_cmos_sensor(0x380F, (imgsensor.frame_length >> 0) & 0xFF);
}

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
}
static int long_exposure_status = 0;
static void write_shutter(kal_uint32 shutter)
{
		kal_uint16 realtime_fps = 0;
	
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
			if(realtime_fps >= 297 && realtime_fps <= 305){
				realtime_fps = 296;
				set_max_framerate(realtime_fps, 0);
			} else if(realtime_fps >= 147 && realtime_fps <= 150){
				realtime_fps = 146;
				set_max_framerate(realtime_fps, 0);
			}
		}
		write_cmos_sensor(0x3208, 0x03);
		if (shutter > (imgsensor_info.max_frame_length - imgsensor_info.margin)) {
			if (shutter > imgsensor.max_shutter) {
				shutter = imgsensor.max_shutter;
			}
		}
		if (shutter > 125030) { // 1s:125031
			//Frame exposure mode customization for LE
			imgsensor.ae_frm_mode.frame_mode_1 = IMGSENSOR_AE_MODE_SE;
			imgsensor.ae_frm_mode.frame_mode_2 = IMGSENSOR_AE_MODE_SE;
			long_exposure_status = 1;
			write_cmos_sensor(0x3508, 0x01);
			write_cmos_sensor(0x3509, 0x00);
			write_cmos_sensor(0x350a, 0x01);
			write_cmos_sensor(0x350b, 0x00);
			write_cmos_sensor(0x350c, 0x00);
			write_cmos_sensor(0x5160, 0x02);
			write_cmos_sensor(0x5161, 0x02);
			write_cmos_sensor(0x5162, 0x02);
			write_cmos_sensor(0x5163, 0x02);
			write_cmos_sensor(0x5164, 0x01);
			write_cmos_sensor(0x5165, 0x00);
			write_cmos_sensor(0x516c, 0x04);
		} else if(long_exposure_status == 1){
			pr_debug("le shutter is %d exit\n",shutter);
			long_exposure_status = 0;
			write_cmos_sensor(0x3016, 0xf9);
			write_cmos_sensor(0x3016, 0xf0);
		}
	
		imgsensor.current_ae_effective_frame = 2;
		imgsensor.frame_length = (imgsensor.frame_length  >> 1) << 1;
		//write_cmos_sensor(0x3840, (imgsensor.frame_length >> 16) & 0xFF);
		write_cmos_sensor(0x380E, (imgsensor.frame_length >> 8) & 0xFF);
		write_cmos_sensor(0x380F, (imgsensor.frame_length >> 0) & 0xFF);
		write_cmos_sensor(0x3500, (shutter >> 16) & 0xFF);
		write_cmos_sensor(0x3501, (shutter >> 8) & 0xFF);
		write_cmos_sensor(0x3502, (shutter >> 0) & 0xFF);
	
		write_cmos_sensor(0x3208, 0x13);
		write_cmos_sensor(0x3208, 0xA3);
		pr_debug("shutter =%d, framelength =%d, realtime_fps =%d\n", shutter,imgsensor.frame_length, realtime_fps);
}

static void set_shutter(kal_uint32 shutter)
{
	unsigned long flags;

	spin_lock_irqsave(&imgsensor_drv_lock, flags);
	imgsensor.shutter = shutter;
	spin_unlock_irqrestore(&imgsensor_drv_lock, flags);
	write_shutter(shutter);
}

static kal_uint16 gain2reg(const kal_uint16 gain)
{
	kal_uint16 iReg = 0x0000;

	//platform 1xgain = 64, sensor driver 1*gain = 0x100
	iReg = gain*256/BASEGAIN;

	if(iReg < 0x100)	//sensor 1xGain
	{
		iReg = 0X100;
	}
#if SENSOR_AGAIN_MAX_256
	if(iReg >= 0x10000)	//sensor 256xGain
	{
		iReg = 0xFFFF;
	}
#else
	if(iReg > 0x4000)	//sensor 63.75xGain
	{
		iReg = 0x4000;
	}
#endif
	return iReg;		/* sensorGlobalGain */
}

static kal_uint16 set_gain(kal_uint16 gain)
{
	kal_uint16 reg_gain;
	unsigned long flags;

	reg_gain = gain2reg(gain);
	spin_lock_irqsave(&imgsensor_drv_lock, flags);
	imgsensor.gain = reg_gain;
	spin_unlock_irqrestore(&imgsensor_drv_lock, flags);

	LOG_INF("gain = %d , reg_gain = 0x%x\n ", gain, reg_gain);
	write_cmos_sensor(0x3508, (reg_gain >> 8));
	write_cmos_sensor(0x3509, (reg_gain&0xff));

	return gain;
}

static void set_shutter_frame_length(kal_uint16 shutter, kal_uint16 target_frame_length)
{
	spin_lock(&imgsensor_drv_lock);
	if ( imgsensor.sensor_mode == IMGSENSOR_MODE_CAPTURE ) {//spec case 24fps
		imgsensor.dummy_line = (target_frame_length > imgsensor_info.cap.framelength) ? (target_frame_length - imgsensor_info.cap.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.cap.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		pr_debug("shutter: %d, frame_length: %d, cap.framelength: %d dummy_line: %d\n", shutter, imgsensor.frame_length, imgsensor_info.cap.framelength, imgsensor.dummy_line);
	} else {
		imgsensor.dummy_line = (target_frame_length > imgsensor_info.custom1.framelength) ? (target_frame_length - imgsensor_info.custom1.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.custom1.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		pr_debug("shutter: %d, frame_length: %d, custom1.framelength: %d dummy_line: %d\n", shutter, imgsensor.frame_length, imgsensor_info.custom1.framelength, imgsensor.dummy_line);
	}
	spin_unlock(&imgsensor_drv_lock);
	set_shutter(shutter);
}


static void ihdr_write_shutter_gain(kal_uint16 le,
				kal_uint16 se, kal_uint16 gain)
{
}

static void night_mode(kal_bool enable)
{
}

static void sensor_init(void)
{
	write_cmos_sensor(0x0103, 0x01);//SW Reset, need delay
	mdelay(10);
	LOG_INF("sensor_init\n");
	ov50a40_table_write_cmos_sensor(
		addr_data_pair_init_ov50a40,
		sizeof(addr_data_pair_init_ov50a40) / sizeof(kal_uint16));
}

static void preview_setting(void)
{
	LOG_INF("preview_setting RES_4000x3000_30fps\n");
#if SENSOR_AGAIN_MAX_256
	ov50a40_table_write_cmos_sensor(
		addr_data_pair_ov50a40_4096_3072_gain256x,
		sizeof(addr_data_pair_ov50a40_4096_3072_gain256x) / sizeof(kal_uint16));
#else
	ov50a40_table_write_cmos_sensor(
		addr_data_pair_preview_ov50a40,
		sizeof(addr_data_pair_preview_ov50a40) / sizeof(kal_uint16));
#endif
}

static void capture_setting(kal_uint16 currefps)
{
	LOG_INF("capture_setting currefps = %d\n",currefps);
#if SENSOR_AGAIN_MAX_256
	ov50a40_table_write_cmos_sensor(
		addr_data_pair_ov50a40_4096_3072_gain256x,
		sizeof(addr_data_pair_ov50a40_4096_3072_gain256x) / sizeof(kal_uint16));
#else
	ov50a40_table_write_cmos_sensor(
		addr_data_pair_capture_ov50a40,
		sizeof(addr_data_pair_capture_ov50a40) /
		sizeof(kal_uint16));
#endif
}

static void normal_video_setting(kal_uint16 currefps)
{
	LOG_INF("normal_video_setting RES_4000x3000_zsl_30fps\n");
#if SENSOR_AGAIN_MAX_256
	ov50a40_table_write_cmos_sensor(
		addr_data_pair_ov50a40_1080p_30fps_gain256x,
		sizeof(addr_data_pair_ov50a40_1080p_30fps_gain256x) / sizeof(kal_uint16));
#else
	ov50a40_table_write_cmos_sensor(
		addr_data_pair_video_ov50a40,
		sizeof(addr_data_pair_video_ov50a40) /
		sizeof(kal_uint16));
#endif
}

static void hs_video_setting(void)
{
	LOG_INF("hs_video_setting RES_1280x720_160fps\n");
#if SENSOR_AGAIN_MAX_256
	ov50a40_table_write_cmos_sensor(
		addr_data_pair_ov50a40_1080p_120fps_gain256x,
		sizeof(addr_data_pair_ov50a40_1080p_120fps_gain256x) / sizeof(kal_uint16));
#else
	ov50a40_table_write_cmos_sensor(
		addr_data_pair_hs_video_ov50a40,
		sizeof(addr_data_pair_hs_video_ov50a40) /
		sizeof(kal_uint16));
#endif
}

static void slim_video_setting(void)
{
	LOG_INF("slim_video_setting RES_3840x2160_30fps\n");
#if SENSOR_AGAIN_MAX_256
		ov50a40_table_write_cmos_sensor(
			addr_data_pair_ov50a40_1080p_60fps_gain256x,
			sizeof(addr_data_pair_ov50a40_1080p_60fps_gain256x) / sizeof(kal_uint16));
#else
	ov50a40_table_write_cmos_sensor(
		addr_data_pair_slim_video_ov50a40,
		sizeof(addr_data_pair_slim_video_ov50a40) /
		sizeof(kal_uint16));
#endif
}


static void custom1_setting(void)
{
  LOG_INF("E\n");
 #if SENSOR_AGAIN_MAX_256
  ov50a40_table_write_cmos_sensor(
		addr_data_pair_ov50a40_4096_3072_gain256x,
		sizeof(addr_data_pair_ov50a40_4096_3072_gain256x) / sizeof(kal_uint16));
#else
  ov50a40_table_write_cmos_sensor(addr_data_pair_custom1_ov50a40,
		sizeof(addr_data_pair_custom1_ov50a40) / sizeof(kal_uint16));
#endif
}	/*	custom1_setting  */

static void custom2_setting(void)
{
  LOG_INF("E\n");
#if SENSOR_AGAIN_MAX_256
  ov50a40_table_write_cmos_sensor(
		addr_data_pair_ov50a40_4096_3072_gain256x,
		sizeof(addr_data_pair_ov50a40_4096_3072_gain256x) / sizeof(kal_uint16));
#else
  ov50a40_table_write_cmos_sensor(addr_data_pair_custom2_ov50a40,
		sizeof(addr_data_pair_custom2_ov50a40) / sizeof(kal_uint16));
#endif
}	/*	custom2_setting  */

static void custom3_setting(void)
{
  LOG_INF("E\n");
#if REMOSAIC_SUPPORT
  ov50a40_table_write_cmos_sensor(addr_data_pair_custom3_ov50a40,
		sizeof(addr_data_pair_custom3_ov50a40) / sizeof(kal_uint16));
#else
  ov50a40_table_write_cmos_sensor(addr_data_pair_capture_ov50a40,
		sizeof(addr_data_pair_capture_ov50a40) / sizeof(kal_uint16));
#endif
}	/*	custom3_setting  */
static void custom4_setting(void)
{
  LOG_INF("E\n");
#if SENSOR_AGAIN_MAX_256
  ov50a40_table_write_cmos_sensor(
		addr_data_pair_ov50a40_4096_3072_gain256x,
		sizeof(addr_data_pair_ov50a40_4096_3072_gain256x) / sizeof(kal_uint16));
#else
  ov50a40_table_write_cmos_sensor(addr_data_pair_custom4_ov50a40,
		sizeof(addr_data_pair_custom4_ov50a40) / sizeof(kal_uint16));
#endif
}	/*	custom4_setting  */
static void custom5_setting(void)
{
  LOG_INF("E\n");
#if SENSOR_AGAIN_MAX_256
  ov50a40_table_write_cmos_sensor(
		addr_data_pair_ov50a40_4096_3072_gain256x,
		sizeof(addr_data_pair_ov50a40_4096_3072_gain256x) / sizeof(kal_uint16));
#else
  ov50a40_table_write_cmos_sensor(addr_data_pair_custom5_ov50a40,
		sizeof(addr_data_pair_custom5_ov50a40) / sizeof(kal_uint16));
#endif
}	/*	custom5_setting  */
static kal_uint32 return_sensor_id(void)
{
	return ((read_cmos_sensor(0x300a) << 16) |
		(read_cmos_sensor(0x300b) << 8) | read_cmos_sensor(0x300c));
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
			*sensor_id = return_sensor_id();
			if (*sensor_id == imgsensor_info.sensor_id) {
				pr_info("i2c write id: 0x%x, sensor id: 0x%x\n",
					imgsensor.i2c_write_id, *sensor_id);
				return ERROR_NONE;
			}
			retry--;
		} while (retry > 0);
		i++;
		retry = 1;
	}
	if (*sensor_id != imgsensor_info.sensor_id) {
		pr_info("get_imgsensor_id: 0x%x fail\n", *sensor_id);
		*sensor_id = 0xFFFFFFFF;
		return ERROR_SENSOR_CONNECT_FAIL;
	}

	return ERROR_NONE;
}

static kal_uint32 open(void)
{
	kal_uint8 i = 0;
	kal_uint8 retry = 1;
	kal_uint32 sensor_id = 0;

	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
			sensor_id = return_sensor_id();
			if (sensor_id == imgsensor_info.sensor_id) {
				pr_info("i2c write id: 0x%x, sensor id: 0x%x\n",
					imgsensor.i2c_write_id, sensor_id);
				break;
			}
			retry--;
		} while (retry > 0);
		i++;
		if (sensor_id == imgsensor_info.sensor_id)
			break;
		retry = 2;
	}
	if (imgsensor_info.sensor_id != sensor_id) {
		pr_info("Open sensor id: 0x%x fail\n", sensor_id);
		return ERROR_SENSOR_CONNECT_FAIL;
	}

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
	imgsensor.ihdr_en = 0;
	imgsensor.test_pattern = KAL_FALSE;
	imgsensor.current_fps = imgsensor_info.pre.max_framerate;
	spin_unlock(&imgsensor_drv_lock);

	return ERROR_NONE;
}

static kal_uint32 close(void)
{
	return ERROR_NONE;
}   /*  close  */

static kal_uint32 preview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
		      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_PREVIEW;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	//imgsensor.video_mode = KAL_FALSE;
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.frame_length = imgsensor_info.pre.framelength;
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.current_fps = imgsensor.current_fps;
	//imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	preview_setting();
	return ERROR_NONE;
}

static kal_uint32 capture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CAPTURE;
	if (imgsensor.current_fps == imgsensor_info.cap1.max_framerate) {
//PIP capture: 24fps for less than 13M, 20fps for 16M,15fps for 20M
	imgsensor.pclk = imgsensor_info.cap1.pclk;
	imgsensor.line_length = imgsensor_info.cap1.linelength;
	imgsensor.frame_length = imgsensor_info.cap1.framelength;
	imgsensor.min_frame_length = imgsensor_info.cap1.framelength;
	//imgsensor.autoflicker_en = KAL_FALSE;
	} else {
	if (imgsensor.current_fps != imgsensor_info.cap.max_framerate)
		LOG_INF("current_fps %d fps is not support,use cap1: %d fps!\n",
			imgsensor.current_fps,
			imgsensor_info.cap1.max_framerate/10);
	imgsensor.pclk = imgsensor_info.cap.pclk;
	imgsensor.line_length = imgsensor_info.cap.linelength;
	imgsensor.frame_length = imgsensor_info.cap.framelength;
	imgsensor.min_frame_length = imgsensor_info.cap.framelength;
	//imgsensor.autoflicker_en = KAL_FALSE;
	}

	spin_unlock(&imgsensor_drv_lock);

	capture_setting(imgsensor.current_fps);

	return ERROR_NONE;
}   /* capture() */

static kal_uint32 normal_video(
			MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_VIDEO;
	imgsensor.pclk = imgsensor_info.normal_video.pclk;
	imgsensor.line_length = imgsensor_info.normal_video.linelength;
	imgsensor.frame_length = imgsensor_info.normal_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.normal_video.framelength;
	//imgsensor.current_fps = 300;
	//imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	normal_video_setting(imgsensor.current_fps);
	return ERROR_NONE;
}

static kal_uint32 hs_video(
			MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
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
	//imgsensor.current_fps = 300;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	hs_video_setting();
	return ERROR_NONE;
}

static kal_uint32 slim_video(
			MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_SLIM_VIDEO;
	imgsensor.pclk = imgsensor_info.slim_video.pclk;
	//imgsensor.video_mode = KAL_TRUE;
	imgsensor.line_length = imgsensor_info.slim_video.linelength;
	imgsensor.frame_length = imgsensor_info.slim_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.slim_video.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	//imgsensor.current_fps = 300;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	slim_video_setting();
	return ERROR_NONE;
}

static kal_uint32 Custom1(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("E\n");

    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM1;
    imgsensor.pclk = imgsensor_info.custom1.pclk;
    //imgsensor.video_mode = KAL_FALSE;
    imgsensor.line_length = imgsensor_info.custom1.linelength;
    imgsensor.frame_length = imgsensor_info.custom1.framelength;
    imgsensor.min_frame_length = imgsensor_info.custom1.framelength;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    custom1_setting();
    return ERROR_NONE;
}   /*  Custom1   */

static kal_uint32 Custom2(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("E\n");

    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM2;
    imgsensor.pclk = imgsensor_info.custom2.pclk;
    //imgsensor.video_mode = KAL_FALSE;
    imgsensor.line_length = imgsensor_info.custom2.linelength;
    imgsensor.frame_length = imgsensor_info.custom2.framelength;
    imgsensor.min_frame_length = imgsensor_info.custom2.framelength;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    custom2_setting();
    return ERROR_NONE;
}   /*  Custom2   */

static kal_uint32 Custom3(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("E\n");

    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM3;
    imgsensor.pclk = imgsensor_info.custom3.pclk;
    //imgsensor.video_mode = KAL_FALSE;
    imgsensor.line_length = imgsensor_info.custom3.linelength;
    imgsensor.frame_length = imgsensor_info.custom3.framelength;
    imgsensor.min_frame_length = imgsensor_info.custom3.framelength;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    custom3_setting();
    return ERROR_NONE;
}   /*  Custom3   */
static kal_uint32 Custom4(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("E\n");

    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM4;
    imgsensor.pclk = imgsensor_info.custom4.pclk;
    //imgsensor.video_mode = KAL_FALSE;
    imgsensor.line_length = imgsensor_info.custom4.linelength;
    imgsensor.frame_length = imgsensor_info.custom4.framelength;
    imgsensor.min_frame_length = imgsensor_info.custom4.framelength;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    custom4_setting();
    return ERROR_NONE;
}   /*  Custom4   */
static kal_uint32 Custom5(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("E\n");

    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM5;
    imgsensor.pclk = imgsensor_info.custom5.pclk;
    //imgsensor.video_mode = KAL_FALSE;
    imgsensor.line_length = imgsensor_info.custom5.linelength;
    imgsensor.frame_length = imgsensor_info.custom5.framelength;
    imgsensor.min_frame_length = imgsensor_info.custom5.framelength;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    custom5_setting();
    return ERROR_NONE;
}   /*  Custom5   */
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

    sensor_resolution->SensorCustom4Width =
		imgsensor_info.custom4.grabwindow_width;
    sensor_resolution->SensorCustom4Height =
		imgsensor_info.custom4.grabwindow_height;

    sensor_resolution->SensorCustom5Width =
		imgsensor_info.custom5.grabwindow_width;
    sensor_resolution->SensorCustom5Height =
		imgsensor_info.custom5.grabwindow_height;

	return ERROR_NONE;
}   /*  get_resolution  */

static kal_uint32 get_info(enum MSDK_SCENARIO_ID_ENUM scenario_id,
		      MSDK_SENSOR_INFO_STRUCT *sensor_info,
		      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	if (scenario_id == 0)
	LOG_INF("scenario_id = %d\n", scenario_id);

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


    sensor_info->Custom1DelayFrame = imgsensor_info.custom1_delay_frame;
    sensor_info->Custom2DelayFrame = imgsensor_info.custom2_delay_frame;

    sensor_info->Custom3DelayFrame = imgsensor_info.custom3_delay_frame;
    sensor_info->Custom4DelayFrame = imgsensor_info.custom4_delay_frame;
    sensor_info->Custom5DelayFrame = imgsensor_info.custom5_delay_frame;

	sensor_info->SensorMasterClockSwitch = 0; /* not use */
	sensor_info->SensorDrivingCurrent = imgsensor_info.isp_driving_current;
/* The frame of setting shutter default 0 for TG int */
	sensor_info->AEShutDelayFrame = imgsensor_info.ae_shut_delay_frame;
	/* The frame of setting sensor gain */
	sensor_info->AESensorGainDelayFrame =
		imgsensor_info.ae_sensor_gain_delay_frame;
	sensor_info->AEISPGainDelayFrame =
		imgsensor_info.ae_ispGain_delay_frame;
	sensor_info->IHDR_Support = imgsensor_info.ihdr_support;
	sensor_info->IHDR_LE_FirstLine = imgsensor_info.ihdr_le_firstline;
	sensor_info->SensorModeNum = imgsensor_info.sensor_mode_num;
#if PDAF_SUPPORT
/*0: NO PDAF, 1: PDAF Raw Data mode, 2:PDAF VC mode*/
	sensor_info->PDAF_Support = PDAF_SUPPORT_CAMSV_QPD;
#else
	sensor_info->PDAF_Support = 0;
#endif

	//sensor_info->HDR_Support = 0; /*0: NO HDR, 1: iHDR, 2:mvHDR, 3:zHDR*/
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
	sensor_info->SensorHightSampling = 0;   // 0 is default 1x
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
	case MSDK_SCENARIO_ID_CUSTOM4:
		sensor_info->SensorGrabStartX =
			imgsensor_info.custom4.startx;
		sensor_info->SensorGrabStartY =
			imgsensor_info.custom4.starty;
		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.custom4.mipi_data_lp2hs_settle_dc;
	break;
	case MSDK_SCENARIO_ID_CUSTOM5:
		sensor_info->SensorGrabStartX =
			imgsensor_info.custom5.startx;
		sensor_info->SensorGrabStartY =
			imgsensor_info.custom5.starty;
		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.custom5.mipi_data_lp2hs_settle_dc;
	break;

	default:
		sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.pre.mipi_data_lp2hs_settle_dc;
	break;
	}

	return ERROR_NONE;
}   /*  get_info  */


static kal_uint32 control(enum MSDK_SCENARIO_ID_ENUM scenario_id,
			MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
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
		Custom1(image_window, sensor_config_data);
	break;
	case MSDK_SCENARIO_ID_CUSTOM2:
		Custom2(image_window, sensor_config_data);
	break;

	case MSDK_SCENARIO_ID_CUSTOM3:
		Custom3(image_window, sensor_config_data);
	break;
	case MSDK_SCENARIO_ID_CUSTOM4:
		Custom4(image_window, sensor_config_data);
	break;
	case MSDK_SCENARIO_ID_CUSTOM5:
		Custom5(image_window, sensor_config_data);
	break;

	default:
		LOG_INF("Error ScenarioId setting");
		preview(image_window, sensor_config_data);
	return ERROR_INVALID_SCENARIO_ID;
	}

	return ERROR_NONE;
}   /* control() */

static kal_uint32 set_video_mode(UINT16 framerate)
{
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

	set_max_framerate(imgsensor.current_fps, 1);

	return ERROR_NONE;
}

static kal_uint32 set_auto_flicker_mode(kal_bool enable,
			UINT16 framerate)
{
	LOG_INF("enable = %d, framerate = %d\n",
		enable, framerate);

	spin_lock(&imgsensor_drv_lock);
	if (enable) //enable auto flicker
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
	kal_uint32 frameHeight;

	LOG_INF("scenario_id = %d, framerate = %d\n", scenario_id, framerate);

	if (framerate == 0)
		return ERROR_NONE;

	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
	    frameHeight = imgsensor_info.pre.pclk / framerate * 10 /
			imgsensor_info.pre.linelength;
	    spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line =
			(frameHeight > imgsensor_info.pre.framelength) ?
			(frameHeight - imgsensor_info.pre.framelength):0;
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
	    frameHeight = imgsensor_info.normal_video.pclk / framerate * 10 /
				imgsensor_info.normal_video.linelength;
	    spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frameHeight >
			imgsensor_info.normal_video.framelength) ?
		(frameHeight - imgsensor_info.normal_video.framelength):0;
	    imgsensor.frame_length = imgsensor_info.normal_video.framelength +
			imgsensor.dummy_line;
	    imgsensor.min_frame_length = imgsensor.frame_length;
	    spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
	break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
	    frameHeight = imgsensor_info.cap.pclk / framerate * 10 /
			imgsensor_info.cap.linelength;
	    spin_lock(&imgsensor_drv_lock);

		imgsensor.dummy_line =
			(frameHeight > imgsensor_info.cap.framelength) ?
			(frameHeight - imgsensor_info.cap.framelength):0;
	    imgsensor.frame_length = imgsensor_info.cap.framelength +
			imgsensor.dummy_line;
	    imgsensor.min_frame_length = imgsensor.frame_length;
	    spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
	break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
	    frameHeight = imgsensor_info.hs_video.pclk / framerate * 10 /
			imgsensor_info.hs_video.linelength;
	    spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line =
			(frameHeight > imgsensor_info.hs_video.framelength) ?
			(frameHeight - imgsensor_info.hs_video.framelength):0;
		imgsensor.frame_length = imgsensor_info.hs_video.framelength +
			imgsensor.dummy_line;
	    imgsensor.min_frame_length = imgsensor.frame_length;
	    spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
	break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
	    frameHeight = imgsensor_info.slim_video.pclk / framerate * 10 /
			imgsensor_info.slim_video.linelength;
	    spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frameHeight >
			imgsensor_info.slim_video.framelength) ?
			(frameHeight - imgsensor_info.slim_video.framelength):0;
	    imgsensor.frame_length = imgsensor_info.slim_video.framelength +
			imgsensor.dummy_line;
	    imgsensor.min_frame_length = imgsensor.frame_length;
	    spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
	break;
	case MSDK_SCENARIO_ID_CUSTOM1:
	    frameHeight = imgsensor_info.custom1.pclk / framerate * 10 /
			imgsensor_info.custom1.linelength;
	    spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frameHeight >
			imgsensor_info.custom1.framelength) ?
			(frameHeight - imgsensor_info.custom1.framelength):0;
	    imgsensor.frame_length = imgsensor_info.custom1.framelength +
			imgsensor.dummy_line;
	    imgsensor.min_frame_length = imgsensor.frame_length;
	    spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
	break;
	case MSDK_SCENARIO_ID_CUSTOM2:
	    frameHeight = imgsensor_info.custom2.pclk / framerate * 10 /
			imgsensor_info.custom2.linelength;
	    spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frameHeight >
			imgsensor_info.custom2.framelength) ?
			(frameHeight - imgsensor_info.custom2.framelength):0;
	    imgsensor.frame_length = imgsensor_info.custom2.framelength +
			imgsensor.dummy_line;
	    imgsensor.min_frame_length = imgsensor.frame_length;
	    spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
	break;
	case MSDK_SCENARIO_ID_CUSTOM3:
	    frameHeight = imgsensor_info.custom3.pclk / framerate * 10 /
			imgsensor_info.custom3.linelength;
	    spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frameHeight >
			imgsensor_info.custom3.framelength) ?
			(frameHeight - imgsensor_info.custom3.framelength):0;
	    imgsensor.frame_length = imgsensor_info.custom3.framelength +
			imgsensor.dummy_line;
	    imgsensor.min_frame_length = imgsensor.frame_length;
	    spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
	break;
	case MSDK_SCENARIO_ID_CUSTOM4:
	    frameHeight = imgsensor_info.custom4.pclk / framerate * 10 /
			imgsensor_info.custom4.linelength;
	    spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frameHeight >
			imgsensor_info.custom4.framelength) ?
			(frameHeight - imgsensor_info.custom4.framelength):0;
	    imgsensor.frame_length = imgsensor_info.custom4.framelength +
			imgsensor.dummy_line;
	    imgsensor.min_frame_length = imgsensor.frame_length;
	    spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
	break;
	case MSDK_SCENARIO_ID_CUSTOM5:
	    frameHeight = imgsensor_info.custom5.pclk / framerate * 10 /
			imgsensor_info.custom5.linelength;
	    spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frameHeight >
			imgsensor_info.custom5.framelength) ?
			(frameHeight - imgsensor_info.custom5.framelength):0;
	    imgsensor.frame_length = imgsensor_info.custom5.framelength +
			imgsensor.dummy_line;
	    imgsensor.min_frame_length = imgsensor.frame_length;
	    spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
	break;
	default:  //coding with  preview scenario by default
	    frameHeight = imgsensor_info.pre.pclk / framerate * 10 /
			imgsensor_info.pre.linelength;
	    spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frameHeight >
			imgsensor_info.pre.framelength) ?
			(frameHeight - imgsensor_info.pre.framelength):0;
	    imgsensor.frame_length = imgsensor_info.pre.framelength +
			imgsensor.dummy_line;
	    imgsensor.min_frame_length = imgsensor.frame_length;
	    spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
	break;
	}
	return ERROR_NONE;
}

static kal_uint32 get_default_framerate_by_scenario(
			enum MSDK_SCENARIO_ID_ENUM scenario_id,
			MUINT32 *framerate)
{
	if (scenario_id == 0)
	LOG_INF("[3058]scenario_id = %d\n", scenario_id);

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
	case MSDK_SCENARIO_ID_CUSTOM4:
	    *framerate = imgsensor_info.custom4.max_framerate;
	break;
	case MSDK_SCENARIO_ID_CUSTOM5:
	    *framerate = imgsensor_info.custom5.max_framerate;
	break;
	default:
	break;
	}

	return ERROR_NONE;
}

static kal_uint32 set_test_pattern_mode(kal_uint32 modes,
	struct SET_SENSOR_PATTERN_SOLID_COLOR *pdata)
{
	printk("modes: %d\n", modes);
	if (modes) {
		if (modes == 5 && (pdata != NULL)) {
			write_cmos_sensor(0x3510, 0x00);
			write_cmos_sensor(0x3511, 0x00);
			write_cmos_sensor(0x3512, 0x00);

			write_cmos_sensor(0x3508, 0x00);
			write_cmos_sensor(0x3509, 0x00);
	
			write_cmos_sensor(0x4019, 0x00);			
			write_cmos_sensor(0x401a, 0x00);
		}
		else
			write_cmos_sensor(0x50C1, 0x01);
	} else {
		write_cmos_sensor(0x50C1, 0x00);
		}

	spin_lock(&imgsensor_drv_lock);
	imgsensor.test_pattern = modes;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}

static kal_uint32 get_sensor_temperature(void)
{
	UINT32 temperature = 0;
	INT32 temperature_convert = 0;

	/*TEMP_SEN_CTL */
	write_cmos_sensor(0x4d12, 0x01);
	temperature = (read_cmos_sensor(0x4d13) << 8) |
		read_cmos_sensor(0x4d13);
	if (temperature < 0xc000)
		temperature_convert = temperature / 256;
	else
		temperature_convert = 192 - temperature / 256;

	if (temperature_convert > 192) {
		//LOG_INF("Temperature too high: %d\n",
				//temperature_convert);
		temperature_convert = 192;
	} else if (temperature_convert < -64) {
		//LOG_INF("Temperature too low: %d\n",
				//temperature_convert);
		temperature_convert = -64;
	}

	return 20;
	//return temperature_convert;
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
			UINT8 *feature_para, UINT32 *feature_para_len)
{
	UINT16 *feature_return_para_16 = (UINT16 *) feature_para;
	UINT16 *feature_data_16 = (UINT16 *) feature_para;
	UINT32 *feature_return_para_32 = (UINT32 *) feature_para;
	UINT32 *feature_data_32 = (UINT32 *) feature_para;
	INT32 *feature_return_para_i32 = (INT32 *) feature_para;
	unsigned long long *feature_data = (unsigned long long *) feature_para;

	struct SENSOR_WINSIZE_INFO_STRUCT *wininfo;
	struct SENSOR_VC_INFO2_STRUCT *pvcinfo2;
	MSDK_SENSOR_REG_INFO_STRUCT *sensor_reg_data =
		(MSDK_SENSOR_REG_INFO_STRUCT *) feature_para;

#if PDAF_SUPPORT
	struct SET_PD_BLOCK_INFO_T *PDAFinfo;
#endif
	if (!((feature_id == 3040) || (feature_id == 3058)))
		LOG_INF("feature_id = %d\n", feature_id);

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
						//case MSDK_SCENARIO_ID_CUSTOM3:
						//	*feature_return_para_32 = 1; /*BINNING_NONE*/
						//	break;
							case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
							case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
							case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
							case MSDK_SCENARIO_ID_SLIM_VIDEO:
							case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
							//case MSDK_SCENARIO_ID_CUSTOM4:
							default:
								*feature_return_para_32 = 1; /*BINNING_AVERAGED*/
							break;
						}
					pr_debug("SENSOR_FEATURE_GET_BINNING_TYPE AE_binning_type:%d,\n",*feature_return_para_32);
					*feature_para_len = 4;
					break;
	case SENSOR_FEATURE_GET_AWB_REQ_BY_SCENARIO:
			switch (*feature_data) {
				//case MSDK_SCENARIO_ID_CUSTOM3:
				//	*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = 1;
				//	break;
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
    //its sensorfusion fail start
    case SENSOR_FEATURE_GET_OFFSET_TO_START_OF_EXPOSURE:
        *(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = 2600000;
    break;
    //its sensorfusion fail end
	case SENSOR_FEATURE_GET_PERIOD:
	    *feature_return_para_16++ = imgsensor.line_length;
	    *feature_return_para_16 = imgsensor.frame_length;
	    *feature_para_len = 4;
	break;
	case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
	    *feature_return_para_32 = imgsensor.pclk;
	    *feature_para_len = 4;
	break;
	
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
		case MSDK_SCENARIO_ID_CUSTOM1:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
			(imgsensor_info.custom1.pclk /
			(imgsensor_info.custom1.linelength - 80))*
			imgsensor_info.custom1.grabwindow_width;

			break;
		case MSDK_SCENARIO_ID_CUSTOM2:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
			(imgsensor_info.custom2.pclk /
			(imgsensor_info.custom2.linelength - 80))*
			imgsensor_info.custom2.grabwindow_width;

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
		set_test_pattern_mode((UINT32)*feature_data,
			(struct SET_SENSOR_PATTERN_SOLID_COLOR *)(uintptr_t)(*(feature_data + 1)));
	break;
	case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE:
	    *feature_return_para_32 = imgsensor_info.checksum_value;
	    *feature_para_len = 4;
	break;
	case SENSOR_FEATURE_SET_FRAMERATE:
	    spin_lock(&imgsensor_drv_lock);
	    imgsensor.current_fps = *feature_data_32;
		spin_unlock(&imgsensor_drv_lock);
		LOG_INF("current fps :%d\n", imgsensor.current_fps);
	break;
	case SENSOR_FEATURE_GET_CROP_INFO:
	    LOG_INF("GET_CROP_INFO scenarioId:%d\n",
			*feature_data_32);

	    wininfo = (struct  SENSOR_WINSIZE_INFO_STRUCT *)
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
		case MSDK_SCENARIO_ID_CUSTOM1:
			memcpy((void *)wininfo,
				(void *)&imgsensor_winsize_info[5],
				sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
		break;
		case MSDK_SCENARIO_ID_CUSTOM2:
			memcpy((void *)wininfo,
				(void *)&imgsensor_winsize_info[6],
				sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
		break;
		case MSDK_SCENARIO_ID_CUSTOM3:
			memcpy((void *)wininfo,
				(void *)&imgsensor_winsize_info[7],
				sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
		break;
		case MSDK_SCENARIO_ID_CUSTOM4:
			memcpy((void *)wininfo,
				(void *)&imgsensor_winsize_info[8],
				sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
		break;
		case MSDK_SCENARIO_ID_CUSTOM5:
			memcpy((void *)wininfo,
				(void *)&imgsensor_winsize_info[9],
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
	    ihdr_write_shutter_gain((UINT16)*feature_data,
			(UINT16)*(feature_data+1),
				(UINT16)*(feature_data+2));
	break;
	case SENSOR_FEATURE_GET_MIPI_PIXEL_RATE:
			switch (*feature_data) {
				case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
				*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
					imgsensor_info.cap.mipi_pixel_rate;
				break;
			case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
				*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
					imgsensor_info.normal_video.mipi_pixel_rate;
				break;
			case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
				*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
					imgsensor_info.hs_video.mipi_pixel_rate;
				break;
			case MSDK_SCENARIO_ID_SLIM_VIDEO:
				*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
					imgsensor_info.slim_video.mipi_pixel_rate;
				break;
			case MSDK_SCENARIO_ID_CUSTOM1:
				*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
					imgsensor_info.custom1.mipi_pixel_rate;
				break;
			case MSDK_SCENARIO_ID_CUSTOM2:
				*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
					imgsensor_info.custom2.mipi_pixel_rate;
				break;
			case MSDK_SCENARIO_ID_CUSTOM3:
				*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
					imgsensor_info.custom3.mipi_pixel_rate;
				break;
			case MSDK_SCENARIO_ID_CUSTOM4:
				*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
					imgsensor_info.custom4.mipi_pixel_rate;
				break;
			case MSDK_SCENARIO_ID_CUSTOM5:
				*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
					imgsensor_info.custom5.mipi_pixel_rate;
				break;
			case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			default:
				*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
					imgsensor_info.pre.mipi_pixel_rate;
				break;
			}
	break;

	 case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ_BY_SCENARIO:
		switch (*feature_data) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
				*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = imgsensor_info.cap.pclk;
				 break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
				*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = imgsensor_info.normal_video.pclk;
				 break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
				*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = imgsensor_info.hs_video.pclk;
				 break;
		case MSDK_SCENARIO_ID_CUSTOM1:
				*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = imgsensor_info.custom1.pclk;
				  break;
		case MSDK_SCENARIO_ID_CUSTOM2:
				*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = imgsensor_info.custom2.pclk;
				break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
				 *(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = imgsensor_info.slim_video.pclk;
				 break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
				*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = imgsensor_info.pre.pclk;
				break;
		}
		break;
	case SENSOR_FEATURE_GET_PERIOD_BY_SCENARIO:
		switch (*feature_data) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
				*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = (imgsensor_info.cap.framelength << 16) + imgsensor_info.cap.linelength;
				 break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
				*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = (imgsensor_info.normal_video.framelength << 16) + imgsensor_info.normal_video.linelength;
				 break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
				*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = (imgsensor_info.hs_video.framelength << 16) + imgsensor_info.hs_video.linelength;
				 break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
				*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = (imgsensor_info.slim_video.framelength << 16) + imgsensor_info.slim_video.linelength;
				break;
		case MSDK_SCENARIO_ID_CUSTOM1:
				*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = (imgsensor_info.custom1.framelength << 16) + imgsensor_info.custom1.linelength;
				 break;
		case MSDK_SCENARIO_ID_CUSTOM2:
				*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = (imgsensor_info.custom2.framelength << 16) + imgsensor_info.custom2.linelength;
				break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
				*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = (imgsensor_info.pre.framelength << 16) + imgsensor_info.pre.linelength;
				break;
		}
		break;

#if PDAF_SUPPORT
/******************** PDAF START ********************/
	case SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY:
		switch (*feature_data) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
			break;
		case MSDK_SCENARIO_ID_CUSTOM4:
			*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
			break;
		default:
			*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
			break;
		}
		break;
	case SENSOR_FEATURE_GET_PDAF_INFO:
		PDAFinfo = (struct SET_PD_BLOCK_INFO_T *)
			(uintptr_t)(*(feature_data+1));

		switch (*feature_data) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			memcpy((void *)PDAFinfo, (void *)&imgsensor_pd_info,
				sizeof(struct SET_PD_BLOCK_INFO_T));
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
		default:
			break;
		}
		break;
	case SENSOR_FEATURE_GET_VC_INFO2:
		pr_debug("SENSOR_FEATURE_GET_VC_INFO %d\n",
			(UINT16) *feature_data);
	
		pvcinfo2 =
		(struct SENSOR_VC_INFO2_STRUCT *) (uintptr_t) (*(feature_data + 1));
	
		switch (*feature_data_32) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			pr_debug("Jesse+ CAPTURE_JPEG \n");
			memcpy((void *)pvcinfo2, (void *)&SENSOR_VC_INFO2[1],
				   sizeof(struct SENSOR_VC_INFO2_STRUCT));
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			pr_debug("Jesse+ VIDEO_PREVIEW \n");
			memcpy((void *)pvcinfo2, (void *)&SENSOR_VC_INFO2[2],
				   sizeof(struct SENSOR_VC_INFO2_STRUCT));
			break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			pr_debug("Jesse+ CAMERA_PREVIEW \n");
			memcpy((void *)pvcinfo2, (void *)&SENSOR_VC_INFO2[0],
				   sizeof(struct SENSOR_VC_INFO2_STRUCT));
			printk("wyh SENSOR_FEATURE_GET_VC_INFO\n");
			break;
		}
		break;
	case SENSOR_FEATURE_GET_PDAF_DATA:
		break;
	case SENSOR_FEATURE_SET_PDAF:
			imgsensor.pdaf_mode = *feature_data_16;
		break;
/******************** PDAF END ********************/
#endif
	case SENSOR_FEATURE_GET_TEMPERATURE_VALUE:
		*feature_return_para_i32 = get_sensor_temperature();
		*feature_para_len = 4;
	break;
	case SENSOR_FEATURE_SET_SHUTTER_FRAME_TIME:
		pr_debug("SENSOR_FEATURE_SET_SHUTTER_FRAME_TIME\n");
		set_shutter_frame_length((UINT16)*feature_data, (UINT16)*(feature_data+1));
	break;
	case SENSOR_FEATURE_SET_STREAMING_SUSPEND:
		streaming_control(KAL_FALSE);
		break;

	case SENSOR_FEATURE_SET_STREAMING_RESUME:
		if (*feature_data != 0)
			set_shutter(*feature_data);
		streaming_control(KAL_TRUE);
		break;
	case SENSOR_FEATURE_GET_AE_EFFECTIVE_FRAME_FOR_LE:
		*feature_return_para_32 = imgsensor.current_ae_effective_frame;
		LOG_INF("GET AE EFFECTIVE %d\n", *feature_return_para_32);
		break;
	case SENSOR_FEATURE_GET_AE_FRAME_MODE_FOR_LE:
		memcpy(feature_return_para_32, &imgsensor.ae_frm_mode, sizeof(struct IMGSENSOR_AE_FRM_MODE));
		LOG_INF("GET_AE_FRAME_MODE");
	default:
	break;
	}

	return ERROR_NONE;
}   /*  feature_control()  */

static struct SENSOR_FUNCTION_STRUCT sensor_func = {
	open,
	get_info,
	get_resolution,
	feature_control,
	control,
	close
};

UINT32 OV50A40_MIPI_RAW_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc)
{
	/* To Do : Check Sensor status here */
	if (pfFunc != NULL)
	*pfFunc =  &sensor_func;
	return ERROR_NONE;
}
