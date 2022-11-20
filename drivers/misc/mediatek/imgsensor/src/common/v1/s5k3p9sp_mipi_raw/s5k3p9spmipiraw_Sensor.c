/*****************************************************************************
 *
 * Filename:
 * ---------
 *	 S5K3P9SPmipi_Sensor.c
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
#include <asm/atomic.h>
#include <linux/types.h>

#include "kd_camera_typedef.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"
#include "s5k3p9spmipiraw_Sensor_setting.h"
#include "s5k3p9spmipiraw_Sensor.h"

#define PFX "S5K3P9SP_camera_sensor"
#define LOG_INF(format,  args...)	pr_debug(PFX "[%s] " format,  __FUNCTION__,  ##args)
#define LOG_ERR(format,  args...)	pr_err(PFX "[%s] " format,  __FUNCTION__,  ##args)

extern int iBurstWriteReg_multi(u8 *pData, u32 bytes, u16 i2cId, u16 transfer_length, u16 timing);

static DEFINE_SPINLOCK(imgsensor_drv_lock);

static imgsensor_info_struct imgsensor_info = {
    .sensor_id = S5K3P9SP_SENSOR_ID,
    .checksum_value = 0x70b13c3,
    .pre = {
        .pclk = 546290000,
        .linelength = 5088,
        .framelength = 3568,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 2320,
        .grabwindow_height = 1744,
        .mipi_data_lp2hs_settle_dc = 85,
        /* following for GetDefaultFramerateByScenario() */
        .mipi_pixel_rate = 475200000,
        .max_framerate = 300,
    },
    .cap = {
        .pclk = 560000000,
        .linelength = 5088,
        .framelength = 3668,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 4640,
        .grabwindow_height = 3488,
        .mipi_data_lp2hs_settle_dc = 85,
        .mipi_pixel_rate = 585600000,
        .max_framerate = 300,
    },
    .cap1 = {
        .pclk = 560000000,
        .linelength = 5088,
        .framelength = 4586,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 2328,
        .grabwindow_height = 1752,
        .mipi_data_lp2hs_settle_dc = 85,
        .mipi_pixel_rate = 480000000,
        .max_framerate = 240,
    },
    .cap2 = {
        .pclk = 560000000,
        .linelength = 5088,
        .framelength = 7336,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 2328,
        .grabwindow_height = 1752,
        .mipi_data_lp2hs_settle_dc = 85,
        .mipi_pixel_rate = 480000000,
        .max_framerate = 150,
    },
    .normal_video = {
        .pclk = 560000000,
        .linelength = 10036,
        .framelength = 1859,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 2320,
        .grabwindow_height = 1744,
        .mipi_data_lp2hs_settle_dc = 85,
        /* following for GetDefaultFramerateByScenario() */
        .mipi_pixel_rate = 278400000,
        .max_framerate = 300,
    },
    .hs_video = {
        .pclk = 560000000,
        .linelength = 5088,
        .framelength = 1834,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 2328,
        .grabwindow_height = 1752,
        .mipi_data_lp2hs_settle_dc = 85,

        /* following for GetDefaultFramerateByScenario() */
        .mipi_pixel_rate = 480000000,
        .max_framerate = 600,  /*1200 */
    },
    .slim_video = {
        .pclk = 560000000,
        .linelength = 5088,
        .framelength = 3668,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 2328,
        .grabwindow_height = 1752,
        .mipi_data_lp2hs_settle_dc = 85,

        /* following for GetDefaultFramerateByScenario() */
        .mipi_pixel_rate = 480000000,
        .max_framerate = 300,
    },

    .margin = 5,
    .min_shutter = 4,
    //Antaiui <AI_BSP_CAM> <yaoyc> <2020-12-01> add for 25571 android R camera hal request begin
    .min_gain = 64,
    .max_gain = 1024,
    .min_gain_iso = 100,
    //.exp_step = 2,
    .gain_step = 2,
    .gain_type = 2,
    //Antaiui <AI_BSP_CAM> <yaoyc> <2020-12-01> add for 25571 android R camera hal request end
    .max_frame_length = 0xffff,
    .ae_shut_delay_frame = 0,
    .ae_sensor_gain_delay_frame = 0,
    .ae_ispGain_delay_frame = 2,
    .ihdr_support = 0,   /*1, support; 0,not support*/
    .ihdr_le_firstline = 0,  /*1,le first; 0, se first*/
    .sensor_mode_num = 5,    /*support sensor mode num*/

    .cap_delay_frame = 1,  /*3 guanjd modify for cts*/
    .pre_delay_frame = 1,  /*3 guanjd modify for cts*/
    .video_delay_frame = 1,
    .hs_video_delay_frame = 3,
    .slim_video_delay_frame = 3,

    .isp_driving_current = ISP_DRIVING_6MA,
    .sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
    .mipi_sensor_type = MIPI_OPHY_NCSI2,  /*0,MIPI_OPHY_NCSI2;  1,MIPI_OPHY_CSI2*/
    .mipi_settle_delay_mode = 1,  /*0,MIPI_SETTLEDELAY_AUTO; 1,MIPI_SETTLEDELAY_MANNUAL*/
    .sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_Gr,
    .mclk = 24,
    .mipi_lane_num = SENSOR_MIPI_4_LANE,
    .i2c_addr_table = {0x5a, 0x20, 0xff},
    .i2c_speed = 400,
};

static imgsensor_struct imgsensor = {
    .mirror = IMAGE_NORMAL,				//mirrorflip information
    .sensor_mode = IMGSENSOR_MODE_INIT, /*IMGSENSOR_MODE enum value,record current sensor mode,such as: INIT, Preview, Capture, Video,High Speed Video, Slim Video*/
    .shutter = 0x200,					/*current shutter*/
    .gain = 0x200,						/*current gain*/
    .dummy_pixel = 0,					/*current dummypixel*/
    .dummy_line = 0,					/*current dummyline*/
    .current_fps = 0,  /*full size current fps : 24fps for PIP, 30fps for Normal or ZSD*/
    .autoflicker_en = KAL_FALSE,  /*auto flicker enable: KAL_FALSE for disable auto flicker, KAL_TRUE for enable auto flicker*/
    .test_pattern = KAL_FALSE,		/*test pattern mode or not. KAL_FALSE for in test pattern mode, KAL_TRUE for normal output*/
    .current_scenario_id = MSDK_SCENARIO_ID_CAMERA_PREVIEW,/*current scenario id*/
    .ihdr_mode = 0, /*sensor need support LE, SE with HDR feature*/
    .i2c_write_id = 0x5a,
};

/* Sensor output window information */
static struct SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[5] =
{
    {  4640, 3488,    0,    0, 4640, 3488, 2320, 1744,    0,    0, 2320, 1744,    0,    0, 2320, 1744}, /* preview */
    {  4640, 3488,    0,    0, 4640, 3488, 4640, 3488,    0,    4, 4640, 3480,    0,    0, 4640, 3488}, /* capture */
    {  4640, 3488,    0,    0, 4640, 3488, 2320, 1744,    0,    0, 2320, 1744,    0,    0, 2320, 1744}, /* video */
    {  4640, 3488,    0,    0, 4640, 3488, 2320, 1744,    0,    0, 2320, 1744,    0,    0, 2320, 1744}, /* hs_video, don't use */
    {  4640, 3488,    0,    0, 4640, 3488, 2320, 1744,    0,    0, 2320, 1744,    0,    0, 2320, 1744}, /* slim video */
};

extern int iReadReg(u16 a_u2Addr , u8 * a_puBuff , u16 i2cId);
extern int iWriteReg(u16 a_u2Addr , u32 a_u4Data , u32 a_u4Bytes , u16 i2cId);
/*extern void kdSetI2CSpeed(u16 i2cSpeed);*/
/*extern bool read_2l9_eeprom( kal_uint16 addr, BYTE* data, kal_uint32 size);*/

static kal_uint16 read_cmos_sensor_16_16(kal_uint32 addr)
{
    kal_uint16 get_byte= 0;
    char pusendcmd[2] = {(char)(addr >> 8), (char)(addr & 0xFF)};
    /*kdSetI2CSpeed(imgsensor_info.i2c_speed); Add this func to set i2c speed by each sensor*/
    iReadRegI2C(pusendcmd, 2, (u8*)&get_byte, 2, imgsensor.i2c_write_id);
    return ((get_byte << 8) & 0xff00) | ((get_byte >> 8) & 0x00ff);
}


static void write_cmos_sensor_16_16(kal_uint16 addr, kal_uint16 para)
{
    char pusendcmd[4] = {(char)(addr >> 8), (char)(addr & 0xFF), (char)(para >> 8), (char)(para & 0xFF)};
    /* kdSetI2CSpeed(imgsensor_info.i2c_speed); Add this func to set i2c speed by each sensor*/
    iWriteRegI2C(pusendcmd, 4, imgsensor.i2c_write_id);
}

static kal_uint16 read_cmos_sensor_16_8(kal_uint16 addr)
{
    kal_uint16 get_byte= 0;
    char pusendcmd[2] = {(char)(addr >> 8), (char)(addr & 0xFF)};
    /*kdSetI2CSpeed(imgsensor_info.i2c_speed);  Add this func to set i2c speed by each sensor*/
    iReadRegI2C(pusendcmd, 2, (u8*)&get_byte, 1, imgsensor.i2c_write_id);
    return get_byte;
}

static void write_cmos_sensor_16_8(kal_uint16 addr, kal_uint8 para)
{
    char pusendcmd[4] = {(char)(addr >> 8) , (char)(addr & 0xFF) ,(char)(para & 0xFF)};
    /* kdSetI2CSpeed(imgsensor_info.i2c_speed);Add this func to set i2c speed by each sensor*/
    iWriteRegI2C(pusendcmd , 3, imgsensor.i2c_write_id);
}

#define MULTI_WRITE 1

#if MULTI_WRITE
#define I2C_BUFFER_LEN 225	/* trans# max is 255, each 3 bytes */
#else
#define I2C_BUFFER_LEN 4

#endif

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
        /* Write when remain buffer size is less than 4 bytes or reach end of data */
        if ((I2C_BUFFER_LEN - tosend) < 4 || IDX == len || addr != addr_last) {
            iBurstWriteReg_multi(puSendCmd, tosend, imgsensor.i2c_write_id,
                    4, imgsensor_info.i2c_speed);
            tosend = 0;
        }
#else
        iWriteRegI2CTiming(puSendCmd, 4, imgsensor.i2c_write_id, imgsensor_info.i2c_speed);
        tosend = 0;
#endif
    }
    return 0;
}

static void set_dummy(void)
{
    LOG_INF("dummyline = %d, dummypixels = %d \n", imgsensor.dummy_line, imgsensor.dummy_pixel);
    write_cmos_sensor_16_16(0x0340, imgsensor.frame_length);
    write_cmos_sensor_16_16(0x0342, imgsensor.line_length);
}  /* set_dummy */


static void set_max_framerate(UINT16 framerate,kal_bool min_framelength_en)
{

    kal_uint32 frame_length = imgsensor.frame_length;

    LOG_INF("framerate = %d, min framelength should enable %d \n", framerate,min_framelength_en);

    frame_length = imgsensor.pclk / framerate * 10 / imgsensor.line_length;
    spin_lock(&imgsensor_drv_lock);
    if (frame_length >= imgsensor.min_frame_length) {
        imgsensor.frame_length = frame_length;
    } else {
        imgsensor.frame_length = imgsensor.min_frame_length;
    }
    imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;

    if (imgsensor.frame_length > imgsensor_info.max_frame_length) {
        imgsensor.frame_length = imgsensor_info.max_frame_length;
        imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;
    }
    if (min_framelength_en) {
        imgsensor.min_frame_length = imgsensor.frame_length;
    }
    spin_unlock(&imgsensor_drv_lock);
    set_dummy();
}  /* set_max_framerate */

static void write_shutter(kal_uint16 shutter)
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
        if (realtime_fps >= 297 && realtime_fps <= 305) {
            set_max_framerate(296,0);
        } else if (realtime_fps >= 147 && realtime_fps <= 150) {
            set_max_framerate(146,0);
        } else {
            /* Extend frame length*/
            write_cmos_sensor_16_16(0x0340, imgsensor.frame_length);
        }
    } else {
        /* Extend frame length*/
        write_cmos_sensor_16_16(0x0340, imgsensor.frame_length);
    }

    /* Update Shutter*/
    write_cmos_sensor_16_16(0x0202, shutter);
    LOG_INF("shutter = %d, framelength = %d\n", shutter,imgsensor.frame_length);

}  /* write_shutter */

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
    spin_lock_irqsave(&imgsensor_drv_lock, flags);
    imgsensor.shutter = shutter;
    spin_unlock_irqrestore(&imgsensor_drv_lock, flags);

    write_shutter(shutter);
}  /* set_shutter */

/* write_shutter */
static void set_shutter_frame_length(kal_uint16 shutter, kal_uint16 frame_length)
{
    unsigned long flags;
    kal_uint16 realtime_fps = 0;
    kal_int32 dummy_line = 0;

    spin_lock_irqsave(&imgsensor_drv_lock, flags);
    imgsensor.shutter = shutter;
    spin_unlock_irqrestore(&imgsensor_drv_lock, flags);

    spin_lock(&imgsensor_drv_lock);
    /* Change frame time */
    if (frame_length > 1) {
        dummy_line = frame_length - imgsensor.frame_length;
    }
    imgsensor.frame_length = imgsensor.frame_length + dummy_line;


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
            write_cmos_sensor_16_16(0x0340, imgsensor.frame_length & 0xFFFF);
        }
    } else {
        /* Extend frame length */
        write_cmos_sensor_16_16(0x0340, imgsensor.frame_length & 0xFFFF);
    }

    /* Update Shutter */
    write_cmos_sensor_16_16(0X0202, shutter & 0xFFFF);

    LOG_INF("shutter = %d, framelength = %d/%d, dummy_line= %d\n", shutter, imgsensor.frame_length,
            frame_length, dummy_line);

}  /* write_shutter */

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

    /*gain= 1024;for test*/
    /*return; for test*/

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

    write_cmos_sensor_16_16(0x0204,reg_gain);
    /*write_cmos_sensor_16_8(0x0204,(reg_gain>>8));*/
    /*write_cmos_sensor_16_8(0x0205,(reg_gain&0xff));*/

    return gain;
}  /* set_gain */

static void set_mirror_flip(kal_uint8 image_mirror)
{
    switch (image_mirror) {

        case IMAGE_NORMAL:
            write_cmos_sensor_16_8(0x0101, 0x00);   /* Gr*/
            break;

        case IMAGE_H_MIRROR:
            write_cmos_sensor_16_8(0x0101, 0x01);
            break;

        case IMAGE_V_MIRROR:
            write_cmos_sensor_16_8(0x0101, 0x02);
            break;

        case IMAGE_HV_MIRROR:
            write_cmos_sensor_16_8(0x0101, 0x03);  /*Gb*/
            break;
        default:
            LOG_INF("Error image_mirror setting\n");
    }
}

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
#if 0
static void night_mode(kal_bool enable)
{
    /*No Need to implement this function*/
}  /* night_mode */
#endif

// Antaiui <AI_BSP_CAM> <xieht> <2021-06-04> add streaming control begin
static void check_output_stream_off(void){
    int timeout = (10000 / imgsensor.current_fps) + 1;
    int i = 0;
    int framecnt = 0;
    for (i = 0; i < timeout; i++) {
        mDELAY(5);
        framecnt = read_cmos_sensor_16_8(0x0005);
        if (framecnt == 0xFF) {
            LOG_INF(" Stream Off OK at i=%d.\n", i);
            break;
        }
    }
    LOG_ERR("Stream Off Fail! framecnt= %d.\n", framecnt);
}

static kal_uint32 streaming_control(kal_bool enable)
{
    LOG_INF("streaming_enable(0= Sw Standby,1= streaming): %d\n", enable);
    if (enable) {
        write_cmos_sensor_16_8(0x0100, 0X01);
        mDELAY(10);
    } else {
        write_cmos_sensor_16_8(0x0100, 0x00);
        check_output_stream_off();
    }
    return ERROR_NONE;
}
// Antaiui <AI_BSP_CAM> <xieht> <2021-06-04> add streaming control end

static void sensor_init(void)
{
    /*Global setting */
    write_cmos_sensor_16_16(0x6028, 0x4000);
    write_cmos_sensor_16_16(0x0000, 0x1000);
    write_cmos_sensor_16_16(0x0000, 0x3109);
    write_cmos_sensor_16_16(0x6010, 0x0001);

    mdelay(3);

    table_write_cmos_sensor(addr_data_pair_init,
            sizeof(addr_data_pair_init) / sizeof(kal_uint16));

    // Antaiui <AI_BSP_CAM> <xieht> <2021-06-04> add streaming off check begin
    write_cmos_sensor_16_16(0x0100, 0x0000);
    check_output_stream_off();
    // Antaiui <AI_BSP_CAM> <xieht> <2021-06-04> add streaming off check end
}  /* sensor_init */

static void preview_setting(void)
{
    LOG_INF("E\n");
    // Antaiui <AI_BSP_CAM> <xieht> <2021-06-04> add streaming off check begin
    write_cmos_sensor_16_16(0x0100, 0x0000);
    check_output_stream_off();
    // Antaiui <AI_BSP_CAM> <xieht> <2021-06-04> add streaming off check end
    table_write_cmos_sensor(addr_data_pair_preview,
            sizeof(addr_data_pair_preview) / sizeof(kal_uint16));
}  /* preview_setting */

static void capture_setting(kal_uint16 currefps)
{
    LOG_INF("E! currefps:%d\n", currefps);
    // Antaiui <AI_BSP_CAM> <xieht> <2021-06-04> add streaming off check begin
    write_cmos_sensor_16_16(0x0100, 0x0000);
    check_output_stream_off();
    // Antaiui <AI_BSP_CAM> <xieht> <2021-06-04> add streaming off check end
    if (currefps == 300) {
        table_write_cmos_sensor(addr_data_pair_capture_30,
                sizeof(addr_data_pair_capture_30) / sizeof(kal_uint16));
    } else if (currefps == 240) {
        table_write_cmos_sensor(addr_data_pair_capture_24,
                sizeof(addr_data_pair_capture_24) / sizeof(kal_uint16));
    } else if (currefps == 150)  {
        table_write_cmos_sensor(addr_data_pair_capture_15,
                sizeof(addr_data_pair_capture_15) / sizeof(kal_uint16));
    } else {
        table_write_cmos_sensor(addr_data_pair_capture_30,
                sizeof(addr_data_pair_capture_30) / sizeof(kal_uint16));
    }
}  /* capture_setting */

static void normal_video_setting()
{
    LOG_INF("E\n");
    // Antaiui <AI_BSP_CAM> <xieht> <2021-06-04> add streaming off check begin
    write_cmos_sensor_16_16(0x0100, 0x0000);
    check_output_stream_off();
    // Antaiui <AI_BSP_CAM> <xieht> <2021-06-04> add streaming off check end
    /* use preview setting here */
    table_write_cmos_sensor(addr_data_pair_preview,
            sizeof(addr_data_pair_preview) / sizeof(kal_uint16));
}  /* normal_video_setting */

static void hs_video_setting(void)
{
    LOG_INF("E\n");
    // Antaiui <AI_BSP_CAM> <xieht> <2021-06-04> add streaming off check begin
    write_cmos_sensor_16_16(0x0100, 0x0000);
    check_output_stream_off();
    // Antaiui <AI_BSP_CAM> <xieht> <2021-06-04> add streaming off check end
    table_write_cmos_sensor(addr_data_pair_hs_video,
            sizeof(addr_data_pair_hs_video) / sizeof(kal_uint16));
}  /* hs_video_setting */

static void slim_video_setting(void)
{
    LOG_INF("E\n");
    // Antaiui <AI_BSP_CAM> <xieht> <2021-06-04> add streaming off check begin
    write_cmos_sensor_16_16(0x0100, 0x0000);
    check_output_stream_off();
    // Antaiui <AI_BSP_CAM> <xieht> <2021-06-04> add streaming off check end
    table_write_cmos_sensor(addr_data_pair_slim_video,
            sizeof(addr_data_pair_slim_video) / sizeof(kal_uint16));
}  /* slim_video_setting */

/*************************************************************************
 * FUNCTION
 *  get_imgsensor_id
 *
 * DESCRIPTION
 *  This function get the sensor ID
 *
 * PARAMETERS
 *  *sensorID : return the sensor ID
 *
 * RETURNS
 *  None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 get_imgsensor_id(UINT32 *sensor_id)
{
    kal_uint8 i = 0;
    kal_uint8 retry = 2;
    /*
       int I2C_BUS = -1;
       I2C_BUS = i2c_adapter_id(pgi2c_cfg_legacy->pinst->pi2c_client->adapter);
       LOG_INF("S5K3P9SPmipiraw_Sensor I2C_BUS = %d\n", I2C_BUS);
       if(I2C_BUS != 4){
     *sensor_id = 0xFFFFFFFF;
     return ERROR_SENSOR_CONNECT_FAIL;
     }
     */
    /*sensor have two i2c address 0x6c 0x6d & 0x21 0x20, we should detect the module used i2c address*/
    while (imgsensor_info.i2c_addr_table[i] != 0xff) {
        spin_lock(&imgsensor_drv_lock);
        imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
        spin_unlock(&imgsensor_drv_lock);
        do {
            *sensor_id = ((read_cmos_sensor_16_8(0x0000) << 8) | read_cmos_sensor_16_8(0x0001));
            LOG_INF("read out sensor id 0x%x \n", *sensor_id);
            if (*sensor_id == imgsensor_info.sensor_id) {
                LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id, *sensor_id);
                return ERROR_NONE;
            }
            LOG_INF("Read sensor id fail, id: 0x%x\n", imgsensor.i2c_write_id);
            retry--;
        } while(retry > 0);
        i++;
        retry = 2;
    }
    if (*sensor_id !=  imgsensor_info.sensor_id) {
        /* if Sensor ID is not correct, Must set *sensor_id to 0xFFFFFFFF*/
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
    LOG_INF("PLATFORM:MT6750,MIPI 4LANE\n");
    LOG_INF("preview 1280*960@30fps,864Mbps/lane; video 1280*960@30fps,864Mbps/lane; capture 5M@30fps,864Mbps/lane\n");

    /*sensor have two i2c address 0x6c 0x6d & 0x21 0x20, we should detect the module used i2c address*/
    while (imgsensor_info.i2c_addr_table[i] != 0xff) {
        spin_lock(&imgsensor_drv_lock);
        imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
        spin_unlock(&imgsensor_drv_lock);
        do {
            sensor_id = ((read_cmos_sensor_16_8(0x0000) << 8) | read_cmos_sensor_16_8(0x0001));
            if (sensor_id == imgsensor_info.sensor_id) {
                LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id, sensor_id);
                break;
            }
            LOG_INF("Read sensor id fail, id: 0x%x\n", imgsensor.i2c_write_id);
            retry--;
        } while(retry > 0);
        i++;
        if (sensor_id == imgsensor_info.sensor_id) {
            break;
        }
        retry = 2;
    }
    if (imgsensor_info.sensor_id !=  sensor_id) {
        return ERROR_SENSOR_CONNECT_FAIL;
    }

    /* initail sequence write in */
    sensor_init();

    spin_lock(&imgsensor_drv_lock);

    imgsensor.autoflicker_en= KAL_FALSE;
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
}  /* open */



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

    return ERROR_NONE;
}  /* close */


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
    imgsensor.line_length = imgsensor_info.pre.linelength;
    imgsensor.frame_length = imgsensor_info.pre.framelength;
    imgsensor.min_frame_length = imgsensor_info.pre.framelength;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);

    preview_setting();
    set_mirror_flip(imgsensor.mirror);
    //burst_read_to_check();
    return ERROR_NONE;
}  /* preview */

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
    if (imgsensor.current_fps == imgsensor_info.cap.max_framerate) {
        /* capture 30 fps */
        imgsensor.pclk = imgsensor_info.cap.pclk;
        imgsensor.line_length = imgsensor_info.cap.linelength;
        imgsensor.frame_length = imgsensor_info.cap.framelength;
        imgsensor.min_frame_length = imgsensor_info.cap.framelength;
    } else if (imgsensor.current_fps == imgsensor_info.cap1.max_framerate) {
        /* capture 24 fps */
        imgsensor.pclk = imgsensor_info.cap1.pclk;
        imgsensor.line_length = imgsensor_info.cap1.linelength;
        imgsensor.frame_length = imgsensor_info.cap1.framelength;
        imgsensor.min_frame_length = imgsensor_info.cap1.framelength;
    } else if (imgsensor.current_fps == imgsensor_info.cap2.max_framerate) {
        /* capture 15 fps */
        imgsensor.pclk = imgsensor_info.cap2.pclk;
        imgsensor.line_length = imgsensor_info.cap2.linelength;
        imgsensor.frame_length = imgsensor_info.cap2.framelength;
        imgsensor.min_frame_length = imgsensor_info.cap2.framelength;
    } else {
        LOG_INF("Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n",
                imgsensor.current_fps, imgsensor_info.cap.max_framerate / 10);
        /* capture default 30 fps */
        imgsensor.pclk = imgsensor_info.cap.pclk;
        imgsensor.line_length = imgsensor_info.cap.linelength;
        imgsensor.frame_length = imgsensor_info.cap.framelength;
        imgsensor.min_frame_length = imgsensor_info.cap.framelength;
    }
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);

    capture_setting(imgsensor.current_fps);
    set_mirror_flip(imgsensor.mirror);

    return ERROR_NONE;
}  /* capture() */

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

    normal_video_setting();
    set_mirror_flip(imgsensor.mirror);

    return ERROR_NONE;
}  /* normal_video */

static kal_uint32 hs_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
        MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("E\n");

    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_HIGH_SPEED_VIDEO;
    imgsensor.pclk = imgsensor_info.hs_video.pclk;
    imgsensor.line_length = imgsensor_info.hs_video.linelength;
    imgsensor.frame_length = imgsensor_info.hs_video.framelength;
    imgsensor.min_frame_length = imgsensor_info.hs_video.framelength;
    imgsensor.dummy_line = 0;
    imgsensor.dummy_pixel = 0;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);

    hs_video_setting();
    set_mirror_flip(imgsensor.mirror);

    return ERROR_NONE;
}  /* hs_video */

static kal_uint32 slim_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
        MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("E\n");

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
    set_mirror_flip(imgsensor.mirror);

    return ERROR_NONE;
}  /* slim_video */

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
}  /* get_resolution */

static kal_uint32 get_info(enum MSDK_SCENARIO_ID_ENUM scenario_id,
        MSDK_SENSOR_INFO_STRUCT *sensor_info,
        MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("scenario_id = %d\n", scenario_id);

    sensor_info->SensorClockPolarity = SENSOR_CLOCK_POLARITY_LOW;
    sensor_info->SensorClockFallingPolarity = SENSOR_CLOCK_POLARITY_LOW; /* not use */
    sensor_info->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW; /* inverse with datasheet*/
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

    sensor_info->AEShutDelayFrame = imgsensor_info.ae_shut_delay_frame; 	 /* The frame of setting shutter default 0 for TG int */
    sensor_info->AESensorGainDelayFrame = imgsensor_info.ae_sensor_gain_delay_frame;	/* The frame of setting sensor gain */
    sensor_info->AEISPGainDelayFrame = imgsensor_info.ae_ispGain_delay_frame;
    sensor_info->IHDR_Support = imgsensor_info.ihdr_support;
    sensor_info->IHDR_LE_FirstLine = imgsensor_info.ihdr_le_firstline;
    sensor_info->SensorModeNum = imgsensor_info.sensor_mode_num;
    sensor_info->PDAF_Support = 0;
    sensor_info->SensorMIPILaneNumber = imgsensor_info.mipi_lane_num;
    sensor_info->SensorClockFreq = imgsensor_info.mclk;
    sensor_info->SensorClockDividCount = 3; /* not use */
    sensor_info->SensorClockRisingCount = 0;
    sensor_info->SensorClockFallingCount = 2; /* not use */
    sensor_info->SensorPixelClockCount = 3; /* not use */
    sensor_info->SensorDataLatchCount = 2; /* not use */

    sensor_info->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
    sensor_info->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
    sensor_info->SensorWidthSampling = 0;	/* 0 is default 1x*/
    sensor_info->SensorHightSampling = 0;	/* 0 is default 1x*/
    sensor_info->SensorPacketECCOrder = 1;

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
}  /* get_info */

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
}  /* control() */

static kal_uint32 set_video_mode(UINT16 framerate)
{
    LOG_INF("framerate = %d\n ", framerate);
    /* SetVideoMode Function should fix framerate*/
    if (framerate == 0) {
        /* Dynamic frame rate*/
        return ERROR_NONE;
    }
    spin_lock(&imgsensor_drv_lock);
    if ((framerate == 300) && (imgsensor.autoflicker_en == KAL_TRUE)) {
        imgsensor.current_fps = 296;
    } else if ((framerate == 150) && (imgsensor.autoflicker_en == KAL_TRUE)) {
        imgsensor.current_fps = 146;
    } else {
        imgsensor.current_fps = framerate;
    }
    spin_unlock(&imgsensor_drv_lock);
    set_max_framerate(imgsensor.current_fps, 1);

    return ERROR_NONE;
}

static kal_uint32 set_auto_flicker_mode(kal_bool enable, UINT16 framerate)
{
    LOG_INF("enable = %d, framerate = %d \n", enable, framerate);
    spin_lock(&imgsensor_drv_lock);
    if (enable) {/*enable auto flicker*/
        imgsensor.autoflicker_en = KAL_TRUE;
    } else {/*Cancel Auto flick*/
        imgsensor.autoflicker_en = KAL_FALSE;
    }
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
            set_dummy();
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
            set_dummy();
            break;
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
            if (framerate == 300) {
                frame_length = imgsensor_info.cap.pclk / framerate * 10 / imgsensor_info.cap.linelength;
                spin_lock(&imgsensor_drv_lock);
                imgsensor.dummy_line = (frame_length > imgsensor_info.cap.framelength) ? (frame_length - imgsensor_info.cap.framelength) : 0;
                imgsensor.frame_length = imgsensor_info.cap.framelength + imgsensor.dummy_line;
                imgsensor.min_frame_length = imgsensor.frame_length;
                spin_unlock(&imgsensor_drv_lock);
            } else if (framerate == 240) {
                frame_length = imgsensor_info.cap1.pclk / framerate * 10 / imgsensor_info.cap1.linelength;
                spin_lock(&imgsensor_drv_lock);
                imgsensor.dummy_line = (frame_length > imgsensor_info.cap1.framelength) ? (frame_length - imgsensor_info.cap1.framelength) : 0;
                imgsensor.frame_length = imgsensor_info.cap1.framelength + imgsensor.dummy_line;
                imgsensor.min_frame_length = imgsensor.frame_length;
                spin_unlock(&imgsensor_drv_lock);
            } else if (framerate == 150) {
                frame_length = imgsensor_info.cap2.pclk / framerate * 10 / imgsensor_info.cap2.linelength;
                spin_lock(&imgsensor_drv_lock);
                imgsensor.dummy_line = (frame_length > imgsensor_info.cap2.framelength) ? (frame_length - imgsensor_info.cap2.framelength) : 0;
                imgsensor.frame_length = imgsensor_info.cap2.framelength + imgsensor.dummy_line;
                imgsensor.min_frame_length = imgsensor.frame_length;
                spin_unlock(&imgsensor_drv_lock);
            } else {
                LOG_INF("Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n",
                        framerate,imgsensor_info.cap.max_framerate / 10);
                frame_length = imgsensor_info.cap.pclk / framerate * 10 / imgsensor_info.cap.linelength;
                spin_lock(&imgsensor_drv_lock);
                imgsensor.dummy_line = (frame_length > imgsensor_info.cap.framelength) ? (frame_length - imgsensor_info.cap.framelength) : 0;
                imgsensor.frame_length = imgsensor_info.cap.framelength + imgsensor.dummy_line;
                imgsensor.min_frame_length = imgsensor.frame_length;
                spin_unlock(&imgsensor_drv_lock);
            }
            set_dummy();
            break;
        case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
            frame_length = imgsensor_info.hs_video.pclk / framerate * 10 / imgsensor_info.hs_video.linelength;
            spin_lock(&imgsensor_drv_lock);
            imgsensor.dummy_line = (frame_length > imgsensor_info.hs_video.framelength) ? (frame_length - imgsensor_info.hs_video.framelength) : 0;
            imgsensor.frame_length = imgsensor_info.hs_video.framelength + imgsensor.dummy_line;
            imgsensor.min_frame_length = imgsensor.frame_length;
            spin_unlock(&imgsensor_drv_lock);
            set_dummy();
            break;
        case MSDK_SCENARIO_ID_SLIM_VIDEO:
            frame_length = imgsensor_info.slim_video.pclk / framerate * 10 / imgsensor_info.slim_video.linelength;
            spin_lock(&imgsensor_drv_lock);
            imgsensor.dummy_line = (frame_length > imgsensor_info.slim_video.framelength) ? (frame_length - imgsensor_info.slim_video.framelength): 0;
            imgsensor.frame_length = imgsensor_info.slim_video.framelength + imgsensor.dummy_line;
            imgsensor.min_frame_length = imgsensor.frame_length;
            spin_unlock(&imgsensor_drv_lock);
            set_dummy();
            break;
        default:  /*coding with  preview scenario by default*/
            frame_length = imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
            spin_lock(&imgsensor_drv_lock);
            imgsensor.dummy_line = (frame_length > imgsensor_info.pre.framelength) ? (frame_length - imgsensor_info.pre.framelength) : 0;
            imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;
            imgsensor.min_frame_length = imgsensor.frame_length;
            spin_unlock(&imgsensor_drv_lock);
            set_dummy();
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
        /* 0x5E00[8]: 1 enable,  0 disable*/
        /* 0x5E00[1:0]; 00 Color bar, 01 Random Data, 10 Square, 11 BLACK*/
        write_cmos_sensor_16_16(0x0600, 0x0002);
    } else {
        /* 0x5E00[8]: 1 enable,  0 disable*/
        /* 0x5E00[1:0]; 00 Color bar, 01 Random Data, 10 Square, 11 BLACK*/
        write_cmos_sensor_16_16(0x0600, 0x0000);
    }
    spin_lock(&imgsensor_drv_lock);
    imgsensor.test_pattern = enable;
    spin_unlock(&imgsensor_drv_lock);
    return ERROR_NONE;
}
static kal_uint32 feature_control(MSDK_SENSOR_FEATURE_ENUM feature_id,
        UINT8 *feature_para,UINT32 *feature_para_len)
{
    UINT16 *feature_return_para_16= (UINT16 *) feature_para;
    UINT16 *feature_data_16= (UINT16 *) feature_para;
    UINT32 *feature_return_para_32= (UINT32 *) feature_para;
    UINT32 *feature_data_32= (UINT32 *) feature_para;
    unsigned long long *feature_data= (unsigned long long *) feature_para;

    struct SET_PD_BLOCK_INFO_T *PDAFinfo;
    struct SENSOR_WINSIZE_INFO_STRUCT *wininfo;

    MSDK_SENSOR_REG_INFO_STRUCT *sensor_reg_data= (MSDK_SENSOR_REG_INFO_STRUCT *) feature_para;

    LOG_INF("feature_id = %d\n", feature_id);
    switch (feature_id) {
        case SENSOR_FEATURE_GET_PERIOD:
            *feature_return_para_16++ = imgsensor.line_length;
            *feature_return_para_16 = imgsensor.frame_length;
            *feature_para_len= 4;
            break;
            //Antaiui <AI_BSP_CAM> <yaoyc> <2020-12-01> add for 25571 android R camera hal request begin
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
            //*(feature_data + 2) = imgsensor_info.exp_step;
            break;
        case SENSOR_FEATURE_GET_BINNING_TYPE:
            *feature_return_para_32 = 1;
            pr_debug("SENSOR_FEATURE_GET_BINNING_TYPE AE_binning_type:%d,\n",
                    *feature_return_para_32);
            *feature_para_len = 4;
            break;
        case SENSOR_FEATURE_GET_FRAME_CTRL_INFO_BY_SCENARIO:
            /*
             * 1, if driver support new sw frame sync
             * set_shutter_frame_length() support third para auto_extend_en
             */
            pr_info("SENSOR_FEATURE_GET_FRAME_CTRL_INFO_BY_SCENARIO \n");
            *(feature_data + 1) = 0;
            /* margin info by scenario */
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
                case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
                default:
                    *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                        = (imgsensor_info.pre.framelength << 16)
                        + imgsensor_info.pre.linelength;
                    break;
            }
            break;
            //Antaiui <AI_BSP_CAM> <yaoyc> <2020-12-01> add for 25571 android R camera hal request end
        case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
            *feature_return_para_32 = imgsensor.pclk;
            *feature_para_len= 4;
            break;
        case SENSOR_FEATURE_SET_ESHUTTER:
            set_shutter(*feature_data);
            break;
        case SENSOR_FEATURE_SET_NIGHTMODE:
            /*night_mode((BOOL) *feature_data);*/
            break;
        case SENSOR_FEATURE_SET_GAIN:
            set_gain((UINT16) *feature_data);
            break;
        case SENSOR_FEATURE_SET_FLASHLIGHT:
            break;
        case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
            break;
        case SENSOR_FEATURE_SET_REGISTER:
            write_cmos_sensor_16_16(sensor_reg_data->RegAddr, sensor_reg_data->RegData);
            break;
        case SENSOR_FEATURE_GET_REGISTER:
            sensor_reg_data->RegData = read_cmos_sensor_16_16(sensor_reg_data->RegAddr);
            break;
        case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
            /* get the lens driver ID from EEPROM or just return LENS_DRIVER_ID_DO_NOT_CARE*/
            /* if EEPROM does not exist in camera module.*/
            *feature_return_para_32= LENS_DRIVER_ID_DO_NOT_CARE;
            *feature_para_len= 4;
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
            set_max_framerate_by_scenario((enum MSDK_SCENARIO_ID_ENUM)*feature_data, *(feature_data + 1));
            break;
        case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
            get_default_framerate_by_scenario((enum MSDK_SCENARIO_ID_ENUM)*(feature_data), (MUINT32 *)(uintptr_t)(*(feature_data + 1)));
            break;
        case SENSOR_FEATURE_SET_TEST_PATTERN:
            set_test_pattern_mode((BOOL)*feature_data);
            break;
        case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE: /*for factory mode auto testing*/
            *feature_return_para_32 = imgsensor_info.checksum_value;
            *feature_para_len= 4;
            break;
        case SENSOR_FEATURE_SET_FRAMERATE:
            LOG_INF("current fps :%d\n", *feature_data_32);
            spin_lock(&imgsensor_drv_lock);
            imgsensor.current_fps = *feature_data_32;
            spin_unlock(&imgsensor_drv_lock);
            break;
        case SENSOR_FEATURE_GET_CROP_INFO:
            LOG_INF("SENSOR_FEATURE_GET_CROP_INFO scenarioId:%d\n", *feature_data_32);
            wininfo = (struct SENSOR_WINSIZE_INFO_STRUCT *)(uintptr_t)(*(feature_data + 1));

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
        case SENSOR_FEATURE_GET_PDAF_INFO:
            LOG_INF("SENSOR_FEATURE_GET_PDAF_INFO scenarioId:%lld\n", *feature_data);
            PDAFinfo= (struct SET_PD_BLOCK_INFO_T *)(uintptr_t)(*(feature_data + 1));

            switch (*feature_data) {
                case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
                    //memcpy((void *)PDAFinfo,(void *)&imgsensor_pd_info,sizeof(struct SET_PD_BLOCK_INFO_T));
                    break;
                case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
                case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
                case MSDK_SCENARIO_ID_SLIM_VIDEO:
                case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
                default:
                    break;
            }
            break;
        case SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY:
            LOG_INF("SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY scenarioId:%lld\n", *feature_data);
            /*PDAF capacity enable or not, 2p8 only full size support PDAF*/
            switch (*feature_data) {
                case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
                    *(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = 0;
                    break;
                case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
                    *(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = 0; /* video & capture use same setting*/
                    break;
                case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
                    *(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = 0;
                    break;
                case MSDK_SCENARIO_ID_SLIM_VIDEO:
                    *(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = 0;
                    break;
                case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
                    *(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = 0;
                    break;
                default:
                    *(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = 0;
                    break;
            }
            break;
        case SENSOR_FEATURE_SET_SHUTTER_FRAME_TIME:
            set_shutter_frame_length((UINT16) *feature_data, (UINT16) *(feature_data + 1));
            break;
        // Antaiui <AI_BSP_CAM> <xieht> <2021-06-04> add streaming control begin
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
        // Antaiui <AI_BSP_CAM> <xieht> <2021-06-04> add streaming control end
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
                        rate = imgsensor_info.pre.mipi_pixel_rate;
                        break;
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
}  /* feature_control() */

static struct SENSOR_FUNCTION_STRUCT sensor_func = {
    open,
    get_info,
    get_resolution,
    feature_control,
    control,
    close
};

UINT32 S5K3P9SP_MIPI_RAW_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc)
{
    /* To Do : Check Sensor status here */
    if (pfFunc!= NULL) {
        *pfFunc= &sensor_func;
    }
    return ERROR_NONE;
}  /* S5K3P9SP_MIPI_RAW_SensorInit */
