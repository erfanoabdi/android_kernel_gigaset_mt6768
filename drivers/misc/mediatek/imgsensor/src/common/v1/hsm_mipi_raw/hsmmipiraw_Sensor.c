/* Copyright Statement:
 *
 * This software/firmware and related documentation ("MediaTek Software") are
 * protected under relevant copyright laws. The information contained herein
 * is confidential and proprietary to MediaTek Inc. and/or its licensors.
 * Without the prior written permission of MediaTek inc. and/or its licensors,
 * any reproduction, modification, use or disclosure of MediaTek Software,
 * and information contained herein, in whole or in part, shall be strictly prohibited.
 *
 * MediaTek Inc. (C) 2010. All rights reserved.
 *
 * BY OPENING THIS FILE, RECEIVER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
 * THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
 * RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO RECEIVER ON
 * AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
 * NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
 * SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
 * SUPPLIED WITH THE MEDIATEK SOFTWARE, AND RECEIVER AGREES TO LOOK ONLY TO SUCH
 * THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. RECEIVER EXPRESSLY ACKNOWLEDGES
 * THAT IT IS RECEIVER'S SOLE RESPONSIBILITY TO OBTAIN FROM ANY THIRD PARTY ALL PROPER LICENSES
 * CONTAINED IN MEDIATEK SOFTWARE. MEDIATEK SHALL ALSO NOT BE RESPONSIBLE FOR ANY MEDIATEK
 * SOFTWARE RELEASES MADE TO RECEIVER'S SPECIFICATION OR TO CONFORM TO A PARTICULAR
 * STANDARD OR OPEN FORUM. RECEIVER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND
 * CUMULATIVE LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
 * AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
 * OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY RECEIVER TO
 * MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
 *
 * The following software/firmware and/or related documentation ("MediaTek Software")
 * have been modified by MediaTek Inc. All revisions are subject to any receiver's
 * applicable license agreements with MediaTek Inc.
 */

/*****************************************************************************
 *  Copyright Statement:
 *  --------------------
 *  This software is protected by Copyright and the information contained
 *  herein is confidential. The software may not be copied and the information
 *  contained herein may not be used or disclosed except with the written
 *  permission of MediaTek Inc. (C) 2005
 *
 *  BY OPENING THIS FILE, BUYER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
 *  THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
 *  RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO BUYER ON
 *  AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
 *  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
 *  MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
 *  NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
 *  SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
 *  SUPPLIED WITH THE MEDIATEK SOFTWARE, AND BUYER AGREES TO LOOK ONLY TO SUCH
 *  THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. MEDIATEK SHALL ALSO
 *  NOT BE RESPONSIBLE FOR ANY MEDIATEK SOFTWARE RELEASES MADE TO BUYER'S
 *  SPECIFICATION OR TO CONFORM TO A PARTICULAR STANDARD OR OPEN FORUM.
 *
 *  BUYER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND CUMULATIVE
 *  LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
 *  AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
 *  OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY BUYER TO
 *  MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE. 
 *
 *  THE TRANSACTION CONTEMPLATED HEREUNDER SHALL BE CONSTRUED IN ACCORDANCE
 *  WITH THE LAWS OF THE STATE OF CALIFORNIA, USA, EXCLUDING ITS CONFLICT OF
 *  LAWS PRINCIPLES.  ANY DISPUTES, CONTROVERSIES OR CLAIMS ARISING THEREOF AND
 *  RELATED THERETO SHALL BE SETTLED BY ARBITRATION IN SAN FRANCISCO, CA, UNDER
 *  THE RULES OF THE INTERNATIONAL CHAMBER OF COMMERCE (ICC).
 *
 *****************************************************************************/

/*****************************************************************************
 *
 * Filename:
 * ---------
 *   Sensor.c
 *
 * Project:
 * --------
 *   DUMA
 *
 * Description:
 * ------------
 *   Image sensor driver function
 *
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by PVCS VM. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/



/*#####################################################


  superpix    sensor   30m  Hsm .   sensorID = 0X0A       SLAVE ADDR= 0X42 



#####################################################*/


#include <linux/proc_fs.h>
#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/of_gpio.h>
#include <asm/atomic.h>
#include <asm/io.h>
//#include <mach/mt_gpio.h>
#include <linux/gpio.h>

//#include "kd_camera_hw.h"
#include "kd_camera_feature_id.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"
#include "kd_camera_feature.h"
#include "kd_camera_typedef.h"

#include "hsmmipiraw_Sensor.h"
#include "hsmmipiraw_Camera_Sensor_para.h"
#include "hsmmipiraw_CameraCustomized.h"
#include "hsm_imager.h"
//#include "mach/gpio_const.h"

#define NEW_POWERUP 1

#define HSM_I2C_M_TWOBYTEREG    0x02

/* N4603 SENSOR ID & REG */
#define N4603_ID_REG     0x3107
#define N4603_REVISION   0x0031

/* N5703 SENSOR ID & REG */
#define N5703_ID_REG     0x3107
#define N5703_REVISION   0x0132

/* N6703 SENSOR ID & REG */
#define AR0144_ID_REG     0x3000
#define AR0144_REVISION   0x0356

/* N5600 SENSOR ID & REG */
#define JADE_ID_REG     0xff
#define JADE_REVISION   0x0700
#define JADE_REVISION_2 0x0701

/* N360X SENSOR ID & REG */
#define MT9M114_CHIP_VERSION_REG    0x0000
#define MT9M114_REVISION            0x2481

extern int iReadRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u8 * a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
extern int iWriteRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u16 i2cId);
extern int hsm_iReadRegI2C(u8 reg , u8 * a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
extern int hsm_iReadRegI2C_MIPI(u16 addr , u8 * rxbuf, u16 len, u16 i2cId);
extern int hsm_iWriteRegI2C_MIPI(u16 addr, u8 *txbuf, u16 len, u16 i2cId);
extern int hsm_iReadRegI2C_WORD(u16 addr , u8 * rxbuf, u16 len, u16 i2cId);
extern int hsm_iWriteRegI2C_WORD(u16 addr, u8 *txbuf, u16 len, u16 i2cId);

//@local function declearation
static int mt_hsm_s_power(int en);
static long hsm_ioctl(unsigned int cmd, void *arg, u32 *len);


static MSDK_SENSOR_CONFIG_STRUCT	HsmSensorConfigData;
static struct Hsm_Sensor_Struct	Hsm_Sensor_Driver;
static struct hsm_engine_properties	hsm_props;


/* ---------- debug helpers ---------- */
/* This debug stuff is contruction zone and is not used yet */
enum hsm_log {
    HSM_LOG_DEBUG		= 0x1000,
    HSM_LOG_INFO_L2		= 0x0200,
    HSM_LOG_INFO_L1		= 0x0100,
    HSM_LOG_WARN		= 0x0010,
    HSM_LOG_ERR		= 0x0001,
    HSM_LOG_ALL		= 0xffff,
};

#define CONFIG_SOC_HSM_IMAGER_DEBUG

#ifdef CONFIG_SOC_HSM_IMAGER_DEBUG
#define HSM_LOG_DEFAULT	HSM_LOG_ALL
#else
#define HSM_LOG_DEFAULT	HSM_LOG_ALL
#endif

/*  add for mt8788 */
#define GPIO_VCC_IO_HOST	170
#define GPIO_3V3_LED_IMGR	171
#define GPIO_3V3_LED_LASER	172
#define GPIO_VCC_MIPI_SWITCH	166
#define GPIO_MIPI_SWITCH_SEL	174
#define GPIO_AIM_ON		85
#define GPIO_ILL_ON		86
#define GPIO_ENG_RESET		87
#define GPIO_POWER_ENA		88

#define GPIO_OUT_ONE 1
#define GPIO_OUT_ZERO 0
u32 g_hsm_power_pinA[10]={0};

static enum hsm_log dbg_flags = HSM_LOG_DEFAULT;

static struct hsm_platform_data	hsm_platform = {
    .pixelformat = 0, //V4L2_MBUS_FMT_Y8_1X8,
    .mount = HSM_MOUNT_UPSIDE_DOWN,
    .hsm_supply_en = &mt_hsm_s_power,

    .gpio[HSM_GPIO_POWER_ENABLE] = {
        .name = "POWER_ENABLE",
        .port = GPIO_POWER_ENA,
        .inverted = false,
        .init = true,
    },
    .gpio[HSM_GPIO_AIMER] = {
        .name = "AIMER",
        .port = GPIO_AIM_ON,
        .inverted = false,
    },
    .gpio[HSM_GPIO_ILLUMINATOR] = {
        .name = "ILLUMINATOR",
        .port = GPIO_ILL_ON,
        .inverted = false,
    },
    .gpio[HSM_GPIO_ENGINE_RESET] = {
        .name = "ENGINE_RESET",
        .port = GPIO_ENG_RESET,
        .inverted = false,
    },
   // .gpio[HSM_GPIO_MIPI_RESET] = {
        //.name = "MIPI_TC_RESET",
       // .port = GPIO_TC_RESET_PIN,
       // .inverted = false,
    //},
};

#define HSM_DEBUG(fmt, ...)						\
    do {								\
        if (dbg_flags & HSM_LOG_DEBUG)				\
        printk(KERN_DEBUG "[hsm_imager] %s " fmt "\n", __func__, ##__VA_ARGS__); \
    } while (0)

#define HSM_INFO_L2(fmt, ...)						\
    do {								\
        if (dbg_flags & HSM_LOG_INFO_L2)			\
        printk(KERN_INFO "[hsm_imager] %s " fmt "\n", __func__, ##__VA_ARGS__); \
    } while (0)

#define HSM_INFO_L1(fmt, ...)						\
    do {								\
        if (dbg_flags & HSM_LOG_INFO_L1)			\
        printk(KERN_INFO "[hsm_imager] %s " fmt "\n", __func__, ##__VA_ARGS__); \
    } while (0)

#define HSM_WARN(fmt, ...)						\
    do {								\
        if (dbg_flags & HSM_LOG_WARN)				\
        printk(KERN_WARNING "[hsm_imager] %s " fmt "\n", __func__, ##__VA_ARGS__); \
    } while (0)


#define HSM_ERROR(fmt, ...)						\
    do {								\
        if (dbg_flags & HSM_LOG_ERR)				\
        printk(KERN_ERR "[hsm_imager] %s " fmt "\n", __func__, ##__VA_ARGS__); \
    } while (0)


#define hsm_dbg(fmt, ...)	HSM_DEBUG(fmt, ##__VA_ARGS__)
#define hsm_info2(fmt, ...)	HSM_INFO_L2(fmt, ##__VA_ARGS__)
#define hsm_info1(fmt, ...)	HSM_INFO_L1(fmt, ##__VA_ARGS__)
#define hsm_warn(fmt, ...)	HSM_WARN(fmt, ##__VA_ARGS__)
#define hsm_err(fmt, ...)	HSM_ERROR(fmt, ##__VA_ARGS__)

#define HSM_MIPI_RAW_DEBUG
#ifdef HSM_MIPI_RAW_DEBUG
#define SENSORDB(fmt,args...)  hsm_dbg("[hsm_mipi_raw]"  ": " fmt,##args)
#else
#define SENSORDB(fmt...)
#endif

#define __SENSOR_CONTROL__
#ifdef __SENSOR_CONTROL__
#define CAMERA_CONTROL_FLOW(para1,para2) hsm_dbg("[%s:%d]::para1=0x%x,para1=0x%x\n\n",__FUNCTION__,__LINE__,para1,para2)
#else
#define CAMERA_CONTROL_FLOW(para1, para2)
#endif

struct hsm_iic_data32 {
	u8	i2c_addr;
	u16	reg;
	u32 buf;
	u8	len;
};
#define HSM_IIC_WRITE32	   _IOWR(_IOC_TYPE(HSM_IIC_WRITE), _IOC_NR(HSM_IIC_WRITE), struct hsm_iic_data32)
#define HSM_IIC_READ32     _IOWR(_IOC_TYPE(HSM_IIC_READ), _IOC_NR(HSM_IIC_READ), struct hsm_iic_data32)

static void compat_iic_data(struct hsm_iic_data32 *iic32, struct hsm_iic_data *iic)
{
	iic->i2c_addr = iic32->i2c_addr;
	iic->reg = iic32->reg;
	iic->buf = compat_ptr(iic32->buf);
	iic->len = iic32->len;
}

static int engine_id;
static bool isN5703_2 = 0;

static kal_uint16 N4603_read_cmos_sensor(kal_uint16 addr);
static kal_uint16 N5703_read_cmos_sensor(kal_uint16 addr);
static kal_uint16 N6700_read_cmos_sensor(kal_uint16 addr);
static kal_uint16 EX30_read_cmos_sensor(kal_uint16 addr);
static kal_uint16 N5600_read_cmos_sensor(kal_uint8 addr);
static kal_uint32 Hsm_GetSensorID(kal_uint32 *sensorID);
static kal_uint32 HsmOpen(void);

kal_uint8 isBanding = 0; // 0: 50hz  1:60hz

// Enable/Disable power to the engine (optional)
static int mt_hsm_s_power(int en)
{
	hsm_dbg("[hsm_ioctl]mt_hsm_s_power en=%d", en);
    if (en)
    {
        hsm_dbg("[hsm_ioctl]mt_hsm_s_power power on###########");
#ifdef NEW_POWERUP
        // Get out from sleep now
        gpio_set_value(g_hsm_power_pinA[8], GPIO_OUT_ONE); //GPIO_POWER_ENA high
        mdelay(10);
        //Reset the MCU
        gpio_set_value(g_hsm_power_pinA[7], GPIO_OUT_ONE); //GPIO_ENG_RESET high
        mdelay(30);
        gpio_set_value(g_hsm_power_pinA[7], GPIO_OUT_ZERO); //GPIO_ENG_RESET low
        mdelay(60);	//give the sensor time to wake up
#else	//legacy power up sequence, before N5703(HS7)
		gpio_set_value(g_hsm_power_pinA[7], GPIO_OUT_ZERO); //GPIO_ENG_RESET low
		mdelay(5);
		gpio_set_value(g_hsm_power_pinA[8], GPIO_OUT_ONE); //GPIO_POWER_ENA high
		mdelay(200);
#endif
    }
    else
    {
        hsm_dbg("[hsm_ioctl]mt_hsm_s_power power off############");
        mdelay(5);
		gpio_set_value(g_hsm_power_pinA[7], GPIO_OUT_ONE); //GPIO_ENG_RESET high
		mdelay(5);
		gpio_set_value(g_hsm_power_pinA[8], GPIO_OUT_ZERO); //GPIO_POWER_ENA low
    }

	hsm_dbg("GPIO87 = %d \n",gpio_get_value(g_hsm_power_pinA[7]));
	hsm_dbg("GPIO88 = %d \n",gpio_get_value(g_hsm_power_pinA[8]));
    return 0;
}

#if 0
void Honeywell_powerup(void)
{
	 hsm_dbg("[hsm_ioctl]Honeywell_powerup \n");	
	//POWER SWITCH
	//mt_set_gpio_mode(GPIO_HSM_POWER_SWITCH,GPIO_MODE_00);  // gpio mode
	//mt_set_gpio_pull_enable(GPIO_HSM_POWER_SWITCH,GPIO_PULL_ENABLE);
	//mt_set_gpio_dir(GPIO_HSM_POWER_SWITCH,GPIO_DIR_OUT); // output
	//LEVEL SWITCH
	//mt_set_gpio_mode(GPIO_HSM_LEVEL_SWITCH,GPIO_MODE_00);  // gpio mode
	//mt_set_gpio_pull_enable(GPIO_HSM_LEVEL_SWITCH,GPIO_PULL_ENABLE);
	//mt_set_gpio_dir(GPIO_HSM_LEVEL_SWITCH,GPIO_DIR_OUT); // output
	//POWER_EN

	mt_set_gpio_mode(GPIO_HSM_POWER_EN_PIN,GPIO_MODE_00);  // gpio mode
	mt_set_gpio_pull_enable(GPIO_HSM_POWER_EN_PIN,GPIO_PULL_ENABLE);
	mt_set_gpio_dir(GPIO_HSM_POWER_EN_PIN,GPIO_DIR_OUT); // output
	//ILL_EN
	mt_set_gpio_mode(GPIO_HSM_ILL_EN_PIN ,GPIO_MODE_00);  // gpio mode
	mt_set_gpio_pull_enable(GPIO_HSM_ILL_EN_PIN ,GPIO_PULL_ENABLE);
	mt_set_gpio_dir(GPIO_HSM_ILL_EN_PIN ,GPIO_DIR_OUT); // output
	//AIM_ON
	mt_set_gpio_mode(GPIO_HSM_AIM_ON_PIN,GPIO_MODE_00);  // gpio mode
	mt_set_gpio_pull_enable(GPIO_HSM_AIM_ON_PIN,GPIO_PULL_ENABLE);
	mt_set_gpio_dir(GPIO_HSM_AIM_ON_PIN,GPIO_DIR_OUT); // output


/*
		mt_set_gpio_out(GPIO_HSM_POWER_SWITCH,GPIO_OUT_ONE); // high
		msleep(1000);
		mt_set_gpio_out(GPIO_HSM_LEVEL_SWITCH,GPIO_OUT_ONE); // high
		msleep(100);
*/
//		mt_set_gpio_out(GPIO_HSM_AIM_ON_PIN,GPIO_OUT_ZERO);
		//mt_set_gpio_out(GPIO_HSM_RESET_PIN,GPIO_OUT_ONE);
		//msleep(10);
  //      mdelay(1000);
		


	//TC RESET
//	mt_set_gpio_mode(GPIO_TC_RESET_PIN,GPIO_MODE_00);  // gpio mode
	//mt_set_gpio_pull_enable(GPIO_TC_RESET_PIN,GPIO_PULL_ENABLE);
//	mt_set_gpio_dir(GPIO_TC_RESET_PIN,GPIO_DIR_OUT); // output
	//RESET
/*
	//POWER SWITCH
	mt_set_gpio_mode(GPIO_HSM_POWER_SWITCH,GPIO_MODE_00);  // gpio mode
	mt_set_gpio_pull_enable(GPIO_HSM_POWER_SWITCH,GPIO_PULL_ENABLE);
	mt_set_gpio_dir(GPIO_HSM_POWER_SWITCH,GPIO_DIR_OUT); // output
	//LEVEL SWITCH
	mt_set_gpio_mode(GPIO_HSM_LEVEL_SWITCH,GPIO_MODE_00);  // gpio mode
	mt_set_gpio_pull_enable(GPIO_HSM_LEVEL_SWITCH,GPIO_PULL_ENABLE);
	mt_set_gpio_dir(GPIO_HSM_LEVEL_SWITCH,GPIO_DIR_OUT); // output
*/
	//POWER_EN
	//ILL_EN
	//AIM_ON
//	mt_set_gpio_out(GPIO_HSM_POWER_SWITCH,GPIO_OUT_ZERO); // high

	msleep(1);


	//mt_set_gpio_out(GPIO_HSM_LEVEL_SWITCH,GPIO_OUT_ONE); // high
	msleep(1);
	mt_set_gpio_out(GPIO_HSM_POWER_EN_PIN,GPIO_OUT_ONE); // high
	msleep(2);
	mt_set_gpio_out(GPIO_HSM_RESET_PIN,GPIO_OUT_ZERO); 
	msleep(5);

	//mt_set_gpio_out(GPIO_TC_RESET_PIN,GPIO_OUT_ZERO); // high
	//msleep(2);
//	mt_set_gpio_out(GPIO_TC_RESET_PIN,GPIO_OUT_ONE); // high
	msleep(2);

	//mt_set_gpio_out(GPIO_HSM_AIM_ON_PIN,GPIO_OUT_ONE); 
	//msleep(500);
	//mt_set_gpio_out(GPIO_HSM_AIM_ON_PIN,GPIO_OUT_ZERO); 
	//mt_set_gpio_out(GPIO_HSM_ILL_EN_PIN ,GPIO_OUT_ZERO);

}


/* ---------- GPIO handling ---------- */

/**
 * hsm_gpio_is_valid_id - Checks whether the id is a valid GPIO
 * @platform: platform data
 * @id: index into our GPIO table
 */
static int hsm_gpio_is_valid_id(struct hsm_platform_data * platform, int id)
{
    if (id < 0 || id >= HSM_NUM_GPIO) {
        hsm_err("(id = %i) invalid GPIO", id);
        return 0;
    }

    if (!platform->gpio[id].name) {
        hsm_err("invalid platform data");
        return 0;
    }

    return 1;
}

/**
 * hsm_gpio_get_port - Retrieves the GPIO port number
 * @platform: platform data
 * @id: index into our GPIO table
 */
static inline int hsm_gpio_get_port(struct hsm_platform_data * platform, int id)
{
    return platform->gpio[id].port;
}

/**
 * hsm_gpio_get_init - init value
 * @platform: platform data
 * @id: index into our GPIO table
 */
static inline int hsm_gpio_get_init(struct hsm_platform_data * platform, int id)
{
    return platform->gpio[id].init;
}

/**
 * hsm_gpio_get_invert - Is the port inverting?
 * @platform: platform data
 * @id: index into our GPIO table
 */
static inline int hsm_gpio_get_invert(struct hsm_platform_data * platform, int id)
{
    return platform->gpio[id].inverted;
}

/**
 * hsm_gpio_get_name - Retrieves the GPIO port name
 * @platform: platform data
 * @id: index into our GPIO table
 */
inline const char * hsm_gpio_get_name(struct hsm_platform_data * platform, int id)
{
    return platform->gpio[id].name;
}

/**
 * hsm_gpio_request - Request the GPIO port from kernel
 * @platform: platform data
 * @id: index into our GPIO table
 * @output: Port is an output if true
 */
static int hsm_gpio_request(struct hsm_platform_data * platform, int id, int output)
{
    unsigned ngpio;
    int initvalue;
    int ret;

    if (!hsm_gpio_is_valid_id(platform, id))
        return -1;

    initvalue = hsm_gpio_get_init(platform, id);
    ngpio = hsm_gpio_get_port(platform, id);

    hsm_dbg("%d (gpio: %u, %s), direction: %d (%s), value: %d",
            id, hsm_gpio_get_port(platform, id), hsm_gpio_get_name(platform, id),
            output, (output == 1 ? "output" : "input"), initvalue);

    ret = gpio_request(ngpio, hsm_gpio_get_name(platform, id));
    if (ret)
        hsm_dbg("Cannot request GPIO id:%d\n", id);

    if (output) {
        initvalue ^= hsm_gpio_get_invert(platform, id);
        ret = gpio_direction_output(ngpio, initvalue);
        gpio_set_value(ngpio, initvalue);
    } else {
        ret = gpio_direction_input(ngpio);
    }

    return ret;
}

/**
 * hsm_gpio_request_output - Request the GPIO port as output from kernel
 * @platform: platform data
 * @id: index into our GPIO table
 * @initvalue: GPIO value
 */
static inline int hsm_gpio_request_output(struct hsm_platform_data * platform, int id)
{
    return hsm_gpio_request(platform, id, 1);
}

/**
 * hsm_gpio_request_input - Request the GPIO port as input from kernel
 * @platform: platform data
 * @id: index into our GPIO table
 */
static inline int hsm_gpio_request_input(struct hsm_platform_data * platform, int id)
{
    return hsm_gpio_request(platform, id, 0);
}
#endif


/* Name supply is used to avoid confusion with power enable which
 * actually should be called not-standby */
static void hsm_supply_enable (struct hsm_platform_data * platform, int en)
{
    if( platform->hsm_supply_en ) platform->hsm_supply_en(en);
}

/**
 * hsm_ioctl - IOCTL handler
 * @cmd: Command
 * @arg: arguments
 * @len: size of arguments
 */
long hsm_ioctl(unsigned int cmd, void *arg, u32 *len)
{
    long ret = 0;

	hsm_dbg("[hsm_ioctl] hsm_ioctl cmd %x\n",cmd);
    switch (cmd)
    {
        case HSM_SUPPLY_WRITE:
          //  hsm_dbg("[hsm_ioctl] HSM_SUPPLY_WRITE\n");
            {
                struct hsm_gpio_data *gpio_data=arg;
                hsm_supply_enable(&hsm_platform, gpio_data->value);
                *len = 0;
            }
            break;

        case HSM_SUPPLY_READ:  
          //  hsm_dbg("[hsm_ioctl] HSM_SUPPLY_READ\n");
            {
                //struct hsm_gpio_data *gpio_data=arg;
                //gpio_data->value = mt_get_gpio_out(GPIO133)&&mt_get_gpio_out(GPIO92);
               // hsm_supply_enable(struct hsm_platform_data * platform, gpio_data->value);
                *len = sizeof(struct hsm_gpio_data);
            }
            break;
    
        case HSM_GET_PROPERTIES:
            hsm_dbg("[hsm_ioctl] HSM_GET_PROPERTIES\n");
            hsm_dbg("[hsm_ioctl] HSM_GET_PROPERTIES, version:%d\n", hsm_props.version);
            {
                if( *len >= sizeof(hsm_props) ) 
                {
                    if (hsm_props.version == 0)
                        HsmOpen();

                	hsm_dbg("[hsm_ioctl] HSM_GET_PROPERTIES memcpy +++\n");
                    hsm_dbg("[hsm_ioctl] version = %d,imager = %d,width = %d ,height = %d i2c_addr_sensor = %d i2c_addr_psoc = %d i2c_addr_clock = %d i2c_addr_mipi = %d\n",
                        hsm_props.version,hsm_props.imager,hsm_props.width,hsm_props.height,hsm_props.i2c_addr_sensor, hsm_props.i2c_addr_psoc,hsm_props.i2c_addr_clock,hsm_props.i2c_addr_mipi);
                    memcpy(arg, &hsm_props, sizeof(hsm_props));
                    hsm_dbg("[hsm_ioctl] HSM_GET_PROPERTIES memcpy ---\n");
                    //copy_to_user(arg, &hsm_props, sizeof(hsm_props));
                    *len = sizeof(hsm_props);
                }
                else
                {
                    ret = -EINVAL;
                }
            }
            break;
        case HSM_IIC_READ: 
           hsm_dbg("[hsm_ioctl] HSM_IIC_READ\n");        
            {
                int err=0;
                struct hsm_iic_data *iic = arg;
                u8 buf[256];
                *len = 0;

                memset(buf, 0, 256);
                if( iic->len <= sizeof(buf) ) 
                {
                    hsm_dbg("[Hsm_hsm_ioctl] read iic->reg=0x%02x, iic->len=0x%02x, iic->i2c_addr=0x%02x\n", iic->reg, iic->len, iic->i2c_addr);
					if( iic->i2c_addr ==PSOC_I2C_ADDR || iic->i2c_addr == 0x10|| iic->i2c_addr ==AIMER_I2C_ADDR || iic->i2c_addr ==ILLUM_I2C_ADDR || ((engine_id == HSM_ENGINE_N5600)&& (iic->i2c_addr ==EV76C454_I2C_ADDR)) )
                    err =hsm_iReadRegI2C(iic->reg, buf, iic->len, iic->i2c_addr << 1);
					if(	iic->i2c_addr ==TC358748_I2C_ADDR ){
					   err =hsm_iReadRegI2C_MIPI(iic->reg, buf, iic->len, iic->i2c_addr << 1);
                       hsm_dbg("MIPI read senseoris %x %x\n",buf[0],buf[1]);
                    }
					if((iic->i2c_addr ==MT9M114_I2C_ADDR) || ((engine_id == HSM_ENGINE_EX30 || engine_id == HSM_ENGINE_N6700) && (iic->i2c_addr ==AR0144_I2C_ADDR || iic->i2c_addr ==AR0144_I2C_ADDR2)) || (iic->i2c_addr ==N4603_I2C_ADDR))
					err =hsm_iReadRegI2C_WORD(iic->reg, buf, iic->len, iic->i2c_addr << 1);
                    if( err >= 0 ) 
                    {
                        copy_to_user(iic->buf, buf, iic->len);             
                    }
                    else
                    {
                        hsm_dbg("[hsm_ioctl] HSM_IIC_READ IIC error\n");
                        if(err == 0)
                        {
                            ret = -EIO;
                        }
                        else 
                        {
                            ret = err;
                        }
                    }
                } 
                else 
                {
                    ret = -ENOMEM;
                }
            }         
            break;
        case HSM_IIC_WRITE: 
            hsm_dbg("[hsm_ioctl] HSM_IIC_WRITE\n"); 
            {
                int err=0;
                struct hsm_iic_data *iic = arg;
                u8 buf[256];
				u8 buf1[256];
//				int n = 0;
                *len = 0;
                if( iic->len < sizeof(buf) ) 
                { 
                    hsm_dbg("[hsm_ioctl] write iic->reg=0x%02x, iic->len=0x%02x, iic->i2c_addr=0x%02x\n", iic->reg, iic->len, iic->i2c_addr); 
            		if(iic->i2c_addr ==PSOC_I2C_ADDR || iic->i2c_addr == 0x10 || iic->i2c_addr ==AIMER_I2C_ADDR || iic->i2c_addr ==ILLUM_I2C_ADDR || ((engine_id == HSM_ENGINE_N5600)&& (iic->i2c_addr ==EV76C454_I2C_ADDR))){
						copy_from_user(buf+1, iic->buf, iic->len); 
						buf[0]= iic->reg;
                    err = iWriteRegI2C(buf, iic->len + 1, iic->i2c_addr << 1);
            			}
					if(iic->i2c_addr ==TC358748_I2C_ADDR){
							copy_from_user(buf1, iic->buf, iic->len);  
					        err = hsm_iWriteRegI2C_MIPI(iic->reg, buf1, iic->len, iic->i2c_addr << 1);
						}	
					if((iic->i2c_addr ==MT9M114_I2C_ADDR) || ((engine_id == HSM_ENGINE_EX30 || engine_id == HSM_ENGINE_N6700) && (iic->i2c_addr ==AR0144_I2C_ADDR || iic->i2c_addr == AR0144_I2C_ADDR2)) || (iic->i2c_addr ==N4603_I2C_ADDR)){
							 copy_from_user(buf1, iic->buf, iic->len);  
					err = hsm_iWriteRegI2C_WORD(iic->reg, buf1, iic->len, iic->i2c_addr << 1);
						}
					//err = hsm_iWriteRegI2C(buf, iic->len + 1, iic->i2c_addr << 1);
/*                    {
                        
              //          hsm_dbg("[hsm_ioctl] iic write buff include reg addr err = %d\n",err);
                        for (n=0; n< iic->len+1; n++)
                        {
                            hsm_dbg("[hsm_ioctl] iic write buff[%d]=0x%02x\n", n, buf[n]);
                        }
                    }
*/
                    if(err < 0) 
                    {
                        hsm_dbg("[hsm_ioctl] HSM_IIC_WRITE IIC error\n");
                        ret = -EIO;
                    }
					else
					{ /*
						for (n=0; n< iic->len+1; n++)
						{	
							
					 		buf[n]= 0;
							hsm_dbg("[hsm_ioctl] clear buffer buf[%d]=%d\n",n, buf[n]);
						}

						err =hsm_iReadRegI2C(iic->reg, buf, iic->len, iic->i2c_addr << 1);
                    	{	
							int n = 0;
							hsm_dbg("[hsm_ioctl] HSM_IIC_WRITE read err=%d \n",err);
		                    //int n = 0;
		                    for (n=0; n< iic->len; n++)
		                    {
		                        hsm_dbg("[hsm_ioctl] iic read data[%d]=0x%02x\n", n, buf[n]);
		                    }
                    	}
				*/	}
                } 
                else 
                {
                    ret = -ENOMEM;
                }
            }
            break;
		case HSM_IIC_READ32:
		    hsm_dbg("[hsm_ioctl] HSM_IIC_READ32\n");
		    {
			    struct hsm_iic_data		i2c;
			    compat_iic_data(arg, &i2c);
			    ret = hsm_ioctl(HSM_IIC_READ, &i2c, len);
			}
		    break;
	    case HSM_IIC_WRITE32:
	        hsm_dbg("[hsm_ioctl] HSM_IIC_WRITE32\n");
	        {
		        struct hsm_iic_data		i2c;
			    compat_iic_data(arg, &i2c);
			    ret = hsm_ioctl(HSM_IIC_WRITE, &i2c, len);
			}
		    break;
        case HSM_GPIO_WRITE:
            hsm_dbg("[hsm_ioctl] HSM_GPIO_WRITE\n"); 
            // {
            //     struct hsm_gpio_data *gpio_data = arg;
            //     hsm_gpio_set(&hsm_platform, gpio_data->pin, gpio_data->value);
            //     *len = 0;
            // }
            break;
        case HSM_GPIO_READ:
            hsm_dbg("[hsm_ioctl] HSM_GPIO_READ\n"); 
            // {
            //     struct hsm_gpio_data *gpio_data = arg;
            //     gpio_data->value = hsm_gpio_get(&hsm_platform, gpio_data->pin);
            //     *len = sizeof(struct hsm_gpio_data);
            // }
            break;
        default:
			 hsm_dbg("[hsm_ioctl] cmd err\n"); 
             ret = -EINVAL;
            break;
    }
    return ret;
}


/*************************************************************************
 * FUNCTION
 *    N6700_read_cmos_sensor
 *
 * DESCRIPTION
 *    This function read data from CMOS sensor through I2C.
 *
 * PARAMETERS
 *    addr: the 16bit address of register
 *
 * RETURNS
 *    8bit data read through I2C
 *
 * LOCAL AFFECTED
 *
 *************************************************************************/
static kal_uint16 N4603_read_cmos_sensor(kal_uint16 addr)
{
    kal_uint16 in_buff = {0};
	int ret;
	ret=hsm_iReadRegI2C_WORD((u16) addr , (u8*)&in_buff, sizeof(in_buff), N4603_WRITE_ID);
	hsm_dbg("reg is %x, addr is %x,read value is %x",addr,N4603_WRITE_ID,in_buff); 
	if(ret < 0)
    {
    	hsm_dbg("in_buf = %d\n", in_buff);
        SENSORDB("ERROR: n4603_read_cmos_sensor \n");

        return 0xFFFF;
    }
    hsm_dbg("in_buff = %d", in_buff);

    return swab16(in_buff) ;
}

static kal_uint16 N5703_read_cmos_sensor(kal_uint16 addr)
{
    kal_uint16 in_buff = {0};
	int ret;
	ret=hsm_iReadRegI2C_WORD((u16) addr , (u8*)&in_buff, sizeof(in_buff), N5703_WRITE_ID);
	hsm_dbg("reg is %x, addr is %x,read value is %x",addr,N5703_WRITE_ID,in_buff); 
	if(ret)
    {
        ret=hsm_iReadRegI2C_WORD((u16) addr , (u8*)&in_buff, sizeof(in_buff), N5703_2_WRITE_ID);
        hsm_dbg("reg is %x, addr is %x,read value is %x",addr,N5703_2_WRITE_ID,in_buff);
        if(ret)
        {
    	    hsm_dbg("in_buf = %d\n", in_buff);
            SENSORDB("ERROR: n5703_read_cmos_sensor \n");
            return 0xFFFF;
        }
        else
        {
            isN5703_2 = 1;
        }
    }
    else
    {
        isN5703_2 = 0;
    }

    hsm_dbg("in_buff = %d", in_buff);

    return swab16(in_buff) ;
}

static kal_uint16 EX30_read_cmos_sensor(kal_uint16 addr)
{
    kal_uint16 in_buff = {0};
	int ret;
	ret=hsm_iReadRegI2C_WORD((u16) addr , (u8*)&in_buff, sizeof(in_buff), EX30_WRITE_ID);
	hsm_dbg("reg is %x, addr is %x,read value is %x",addr,EX30_WRITE_ID,in_buff); 
	if(ret < 0)
    {
    	hsm_dbg("in_buf = %d\n", in_buff);
        SENSORDB("ERROR: ex30_read_cmos_sensor \n");

        return 0xFFFF;
    }
    hsm_dbg("in_buff = %d", in_buff);

    return swab16(in_buff) ;
}

static kal_uint16 N6700_read_cmos_sensor(kal_uint16 addr)
{
    kal_uint16 in_buff = {0};
	int ret;
	ret=hsm_iReadRegI2C_WORD((u16) addr , (u8*)&in_buff, sizeof(in_buff), N6700_WRITE_ID);
	hsm_dbg("reg is %x, addr is %x,read value is %x",addr,N6700_WRITE_ID,in_buff); 
	if(ret < 0)
    {
    	hsm_dbg("in_buf = %d", in_buff);
        SENSORDB("ERROR: N6700_read_cmos_sensor \n");

        return 0xFFFF;
    }
    hsm_dbg("in_buff = %d", in_buff);

    return swab16(in_buff) ;
}

static kal_uint16 N3600_read_cmos_sensor(kal_uint16 addr)
{
    kal_uint16 in_buff = {0};
	int ret;
	ret=hsm_iReadRegI2C_WORD((u16) addr , (u8*)&in_buff, sizeof(in_buff), N3600_WRITE_ID);
	hsm_dbg("reg is %x, addr is %x,read value is %x",addr,N3600_WRITE_ID,in_buff); 
	if(ret < 0)
    {
    	hsm_dbg("in_buf = %d", in_buff);
        SENSORDB("ERROR: N3600_read_cmos_sensor \n");

        return 0xFFFF;
    }
    hsm_dbg("in_buff = %d", in_buff);

    return swab16(in_buff) ;
}


static kal_uint16 N5600_read_cmos_sensor(kal_uint8 addr)
{
    kal_uint16 in_buff = {0};
    kal_uint8   out_buff = addr;
     SENSORDB("%s enter\n",__FUNCTION__); 
    if (0 != iReadRegI2C((u8*)&out_buff , (u16) sizeof(out_buff), (u8*)&in_buff, (u16) sizeof(in_buff), N5600_WRITE_ID)) 
    {
        SENSORDB("ERROR: N5600_read_cmos_sensor \n");
        return 0xFFFF;
    }

    return swab16(in_buff) ;
}

/*************************************************************************
 * FUNCTION
 *	Hsm_NightMode
 *
 * DESCRIPTION
 *	This function night mode of Hsm.
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
static void Hsm_night_mode(kal_bool bEnable)
{


    if (!Hsm_Sensor_Driver.MODE_CAPTURE) { 
        if(bEnable)//night mode
        {
            Hsm_Sensor_Driver.bNight_mode = KAL_TRUE;
            //Hsm_write_cmos_sensor(0xfd,0x0 );
            //Hsm_write_cmos_sensor(0xb2,0x25);
            //Hsm_write_cmos_sensor(0xb3,0x1f);
            if(Hsm_Sensor_Driver.MPEG4_encode_mode == KAL_TRUE)
            {
                if(isBanding== 0)
                {
                    //Video record night 24M 50hz 12-12FPS maxgain:0x8c
                    //dbg_print(" video 50Hz night\r\n");
                }
                else if(isBanding == 1)
                {
                    //Video record night 24M 60Hz 12-12FPS maxgain:0x8c
                    //dbg_print(" video 60Hz night\r\n");
                }
            }
            else
            {
                //	dbg_print(" Hsm_banding=%x\r\n",Hsm_banding);
                if(isBanding== 0)
                {
                    //capture preview night 24M 50hz 20-6FPS maxgain:0x78	 	
                    //dbg_print(" priview 50Hz night\r\n");	
                }  
                else if(isBanding== 1)
                {
                    //capture preview night 24M 60hz 20-6FPS maxgain:0x78
                    //	dbg_print(" priview 60Hz night\r\n");	
                }
            } 		
        }
        else    // daylight mode
        {
            Hsm_Sensor_Driver.bNight_mode = KAL_FALSE;
            if(Hsm_Sensor_Driver.MPEG4_encode_mode == KAL_TRUE)
            {
                //dbg_print(" Hsm_banding=%x\r\n",Hsm_banding);
                if(isBanding== 0)
                {
                    //Video record daylight 24M 50hz 20-20FPS maxgain:0x8c

                    //dbg_print(" video 50Hz normal\r\n");				
                }
                else if(isBanding == 1)
                {

                }
            }
            else
            {
                //	dbg_print(" Hsm_banding=%x\r\n",Hsm_banding);
                if(isBanding== 0)
                {
                    //	dbg_print(" priview 50Hz normal\r\n");
                }
                else if(isBanding== 1)
                {
                    //	dbg_print(" priview 60Hz normal\r\n");
                }
            }
        }
    }
}	/*	Hsm_NightMode	*/

static void Hsm_Sensor_Driver_Init(void)
{
   // kal_uint8 buff[10];
hsm_dbg("Hsm_Sensor_Driver_Init\n");
}


/*****************************************************************************/
/* Windows Mobile Sensor Interface */
/*****************************************************************************/
/*************************************************************************
 * FUNCTION
 *	HsmOpen
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
static kal_uint32 HsmOpen(void)
{
    kal_uint32 sensor_id=0; 
   // kal_uint16 tempid=0; 
    int retry = 10; 
	
    SENSORDB("[hsm_ioctl]HsmOpen_start \n");
	
    // check if sensor ID correct
    do {
        Hsm_GetSensorID(&sensor_id);
        if (sensor_id == HSM_SENSOR_ID) {
            break; 
        }
        SENSORDB("Read Sensor ID Fail = 0x%x\n", sensor_id); 

        retry--;  
    } while (retry > 0); 

	if (sensor_id != HSM_SENSOR_ID) {
		return ERROR_SENSOR_CONNECT_FAIL;
	}

    memset(&hsm_props, 0, sizeof(hsm_props));
    hsm_props.version = HSM_PROPERTIES_V1;
    hsm_props.mount = HSM_MOUNT_UPSIDE_DOWN;
	switch(engine_id)
	{
		case HSM_ENGINE_N3600:
			hsm_props.Reserved[0] = HSM_MIPI_1_LANE;
			hsm_props.width = N3600_IMAGE_SENSOR_WIDTH;//
			hsm_props.height = N3600_IMAGE_SENSOR_HEIGHT;//
			hsm_props.i2c_addr_sensor = MT9M114_I2C_ADDR;
			hsm_props.i2c_addr_psoc = 0;
			hsm_props.i2c_addr_clock = 0;
			hsm_props.i2c_addr_mipi  = 0;
			break;		
		case HSM_ENGINE_N3603:
			hsm_props.Reserved[0] = HSM_MIPI_1_LANE;
			hsm_props.width = N3600_IMAGE_SENSOR_WIDTH;//
			hsm_props.height = N3600_IMAGE_SENSOR_HEIGHT;//
			hsm_props.i2c_addr_sensor = MT9M114_I2C_ADDR;
			hsm_props.i2c_addr_psoc = PSOC_I2C_ADDR;
			hsm_props.i2c_addr_clock = 0;
			hsm_props.i2c_addr_mipi  = 0;
			break;
		case HSM_ENGINE_N5600:
			hsm_props.Reserved[0] = HSM_MIPI_1_LANE;
			hsm_props.width = N5600_IMAGE_SENSOR_WIDTH;//
			hsm_props.height = N5600_IMAGE_SENSOR_HEIGHT;//
			hsm_props.i2c_addr_sensor = EV76C454_I2C_ADDR;
			hsm_props.i2c_addr_psoc = PSOC_I2C_ADDR;
			hsm_props.i2c_addr_clock = IDT6P50016_I2C_ADDR;
			hsm_props.i2c_addr_mipi  = TC358748_I2C_ADDR;
			break;
		case HSM_ENGINE_N6700:
			hsm_props.Reserved[0] = HSM_MIPI_1_LANE;
			hsm_props.width = N6700_IMAGE_SENSOR_WIDTH;//
			hsm_props.height = N6700_IMAGE_SENSOR_HEIGHT;//
			hsm_props.i2c_addr_sensor = AR0144_I2C_ADDR;
			hsm_props.i2c_addr_psoc = PSOC_I2C_ADDR;
			break;
		case HSM_ENGINE_EX30:
			hsm_props.Reserved[0] = HSM_MIPI_1_LANE;
			hsm_props.width = EX30_WIDTH;//
			hsm_props.height = EX30_HEIGHT;//
			hsm_props.i2c_addr_sensor = AR0144_I2C_ADDR2;
			hsm_props.i2c_addr_psoc = PSOC_I2C_ADDR;
			break;
		case HSM_ENGINE_N4603:
			hsm_props.Reserved[0] = HSM_MIPI_1_LANE;
			hsm_props.width = N4603_WIDTH;//
			hsm_props.height = N4603_HEIGHT;//
			hsm_props.i2c_addr_sensor = N4603_I2C_ADDR;
			hsm_props.i2c_addr_psoc = PSOC_I2C_ADDR;
			break;
		case HSM_ENGINE_N5703:
			hsm_props.Reserved[0] = HSM_MIPI_1_LANE;
			hsm_props.width = N5703_WIDTH;//
			hsm_props.height = N5703_HEIGHT;//
			hsm_props.i2c_addr_sensor = isN5703_2 ? N5703_I2C_ADDR2 : N5703_I2C_ADDR;
			hsm_props.i2c_addr_psoc = PSOC_I2C_ADDR;
			break;            		
		default:
			break;
	}
	

    hsm_dbg("[HsmOpen] version = %d,imager = %d,width = %d ,height = %d i2c_addr_sensor = %d i2c_addr_psoc = %d i2c_addr_clock = %d i2c_addr_mipi = %d\n",
    hsm_props.version,hsm_props.imager,hsm_props.width,hsm_props.height,hsm_props.i2c_addr_sensor, hsm_props.i2c_addr_psoc,hsm_props.i2c_addr_clock,hsm_props.i2c_addr_mipi);
	
    memset(&Hsm_Sensor_Driver, 0, sizeof(struct Hsm_Sensor_Struct)); 
    Hsm_Sensor_Driver.MPEG4_encode_mode=KAL_FALSE;
    Hsm_Sensor_Driver.dummy_pixels=0;
    Hsm_Sensor_Driver.dummy_lines=0;
    Hsm_Sensor_Driver.extra_exposure_lines=0;
    Hsm_Sensor_Driver.exposure_lines=0;
    Hsm_Sensor_Driver.MODE_CAPTURE=KAL_FALSE;

    Hsm_Sensor_Driver.bNight_mode =KAL_FALSE; // to distinguish night mode or auto mode, default: auto mode setting
    Hsm_Sensor_Driver.bBanding_value = AE_FLICKER_MODE_50HZ; // to distinguish between 50HZ and 60HZ.

    Hsm_Sensor_Driver.fPV_PCLK = 48; //26000000;
    Hsm_Sensor_Driver.iPV_Pixels_Per_Line = 0;

    mDELAY(10);
    Hsm_Sensor_Driver_Init();		
    SENSORDB("HsmOpen_end \n");

    return ERROR_NONE;
}   /* HsmOpen  */



/*************************************************************************
 * FUNCTION
 *	Hsm_GetSensorID
 *
 * DESCRIPTION
 *	This function get the sensor ID
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
static kal_uint32 Hsm_GetSensorID(kal_uint32 *sensorID)
{
	kal_uint16 tempid=0xFFFF;
	int err=0;
	u8 data[4];
    unsigned char imagerType = 0xff;
    //mt_hsm_s_power(1);
    memset(data, 0, 4);
    hsm_dbg("Enter HsmGetSensorID");	
    //Read sensor ID to adjust I2C is OK?
    msleep(200);
	err =hsm_iReadRegI2C(0x0, data, 4, 0x80);
        hsm_dbg(" POSC check Engine ID = %x%x%x%x engine_id:%d,ret:%d",data[0],data[1],data[2],data[3],engine_id,err);
	if(!err) {
		if( !(data[0] & 0x80) ) {
			engine_id= HSM_ENGINE_IT5000;
		} else if (data[0] == 0x90){
			imagerType = (data[1]&0x7)<<1 | (data[2]>>7);
			switch (imagerType) {
			case 2:
                N5703_REVISION == N5703_read_cmos_sensor(N5703_ID_REG)? engine_id = HSM_ENGINE_N5703:HSM_ENGINE_UNSUPPORTED;
				break;
			default:
				engine_id = HSM_ENGINE_UNSUPPORTED;
				break;
			}
		}else {
			switch( data[2] >> 5 ) {
			case 0:
			case 1:
			    tempid = N5600_read_cmos_sensor(JADE_ID_REG);
			    tempid == JADE_REVISION || tempid == JADE_REVISION_2 ? engine_id = HSM_ENGINE_N5600:HSM_ENGINE_UNSUPPORTED;
			    break;
			case 2:
				AR0144_REVISION == N6700_read_cmos_sensor(AR0144_ID_REG)? engine_id = HSM_ENGINE_N6700: HSM_ENGINE_UNSUPPORTED;
				break;
			case 3:
				MT9M114_REVISION == N3600_read_cmos_sensor(MT9M114_CHIP_VERSION_REG)? engine_id = HSM_ENGINE_N3603:HSM_ENGINE_UNSUPPORTED;			
				break;
			case 5:
				AR0144_REVISION == EX30_read_cmos_sensor(AR0144_ID_REG)? engine_id = HSM_ENGINE_EX30:HSM_ENGINE_UNSUPPORTED;
				break;
			case 7:
				N4603_REVISION == N4603_read_cmos_sensor(N4603_ID_REG)? engine_id = HSM_ENGINE_N4603:HSM_ENGINE_UNSUPPORTED;
				break;
			default:
				MT9M114_REVISION ==N3600_read_cmos_sensor(MT9M114_CHIP_VERSION_REG)? engine_id = HSM_ENGINE_N3600: HSM_ENGINE_UNSUPPORTED;
			}
	    }
    }
    else {
        MT9M114_REVISION == N3600_read_cmos_sensor(MT9M114_CHIP_VERSION_REG)? engine_id = HSM_ENGINE_N3600: HSM_ENGINE_UNSUPPORTED;
    }

  	if(engine_id == HSM_ENGINE_UNSUPPORTED || err == -1){
	    *sensorID = 0xFFFFFFFF;
		return ERROR_SENSOR_CONNECT_FAIL;
	}else{
		*sensorID = HSM_SENSOR_ID;
		hsm_dbg("Hsm Sensor ID = 0x%x",*sensorID);
		return ERROR_NONE;
			}
}


static kal_uint32 HsmGetDefaultFramerateByScenario(enum MSDK_SCENARIO_ID_ENUM scenarioId, MUINT32 *pframeRate)
{
 SENSORDB("#### set fps, scenarioId = %d \n", scenarioId);

 *pframeRate = 600;

 return ERROR_NONE;
}

/*************************************************************************
 * FUNCTION
 *	HsmClose
 *
 * DESCRIPTION
 *	This function is to turn off sensor module power.
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
static kal_uint32 HsmClose(void)
{
    SENSORDB("HsmClose\n");
    return ERROR_NONE;
}   /* HsmClose */

/*************************************************************************
 * FUNCTION
 * Hsm_Preview
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
static kal_uint32 Hsm_Preview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
        MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)

{
    hsm_dbg("[hsm_mipi_raw] %s is called!\n",__FUNCTION__);
    
    Hsm_Sensor_Driver.fPV_PCLK=48000000;
    Hsm_Sensor_Driver.MODE_CAPTURE=KAL_FALSE;

    image_window->GrabStartX= HSM_SENSOR_GRAB_START_X;
    image_window->GrabStartY= HSM_SENSOR_GRAB_START_Y;
	switch(engine_id){
		case HSM_ENGINE_N3600:
		case HSM_ENGINE_N3603:
			image_window->ExposureWindowWidth = N3600_SENSOR_PV_WIDTH;
			image_window->ExposureWindowHeight = N3600_SENSOR_PV_HEIGHT;
			break;
		case HSM_ENGINE_N5600:
			image_window->ExposureWindowWidth = N5600_SENSOR_PV_WIDTH;
			image_window->ExposureWindowHeight = N5600_SENSOR_PV_HEIGHT;
			break;
		case HSM_ENGINE_N6700:
			image_window->ExposureWindowWidth = N6700_SENSOR_PV_WIDTH;
			image_window->ExposureWindowHeight = N6700_SENSOR_PV_HEIGHT;
			break;
		case HSM_ENGINE_EX30:
			image_window->ExposureWindowWidth = EX30_SENSOR_PV_WIDTH;
			image_window->ExposureWindowHeight = EX30_SENSOR_PV_HEIGHT;
			break;
		case HSM_ENGINE_N4603:
			image_window->ExposureWindowWidth = N4603_SENSOR_PV_WIDTH;
			image_window->ExposureWindowHeight = N4603_SENSOR_PV_HEIGHT;
			break;
		case HSM_ENGINE_N5703:
			image_window->ExposureWindowWidth = N5703_SENSOR_PV_WIDTH;
			image_window->ExposureWindowHeight = N5703_SENSOR_PV_HEIGHT;
			break;
		default:
			image_window->ExposureWindowWidth = N6700_SENSOR_PV_WIDTH;
			image_window->ExposureWindowHeight = N6700_SENSOR_PV_HEIGHT;						
	}
	hsm_dbg("[Hsm_Preview]  engine_id = %d,ExposureWindowWidth = %d,ExposureWindowHeight = %d\n",engine_id,image_window->ExposureWindowWidth,image_window->ExposureWindowHeight);
	    
    // copy sensor_config_data
    memcpy(&HsmSensorConfigData, sensor_config_data, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
    return ERROR_NONE;
}   /*  Hsm_Preview   */


/*************************************************************************
 * FUNCTION
 *  capture
 *
 * DESCRIPTION
 *  This function setup the CMOS sensor in capture MY_OUTPUT mode
 *
 * PARAMETERS
 *
 * RETURNS
 *  None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 capture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
            MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    return ERROR_NONE;

}   /* capture() */
static kal_uint32 normal_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
              MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    return ERROR_NONE;
}   /*  normal_video   */

static kal_uint32 hs_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
            MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    return ERROR_NONE;
}    /*    hs_video   */

static kal_uint32 slim_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
              MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    return ERROR_NONE;
}    /*    slim_video     */

static kal_uint32 HsmGetResolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *pSensorResolution)
{
    //kal_uint8 tmp1;
	switch(engine_id)
	{
		case HSM_ENGINE_N3600:
		case HSM_ENGINE_N3603:
		    pSensorResolution->SensorFullWidth= N3600_SENSOR_FULL_WIDTH;
			pSensorResolution->SensorFullHeight= N3600_SENSOR_FULL_HEIGHT;
    
			pSensorResolution->SensorPreviewWidth= N3600_SENSOR_PV_WIDTH;
			pSensorResolution->SensorPreviewHeight= N3600_SENSOR_PV_HEIGHT;
    
			pSensorResolution->SensorVideoWidth=N3600_SENSOR_VIDEO_WIDTH;
			pSensorResolution->SensorVideoHeight=N3600_SENSOR_VIDEO_HEIGHT;
			break;
		case HSM_ENGINE_N5600:
			pSensorResolution->SensorFullWidth= N5600_SENSOR_FULL_WIDTH;
			pSensorResolution->SensorFullHeight= N5600_SENSOR_FULL_HEIGHT;
    
			pSensorResolution->SensorPreviewWidth= N5600_SENSOR_PV_WIDTH;
			pSensorResolution->SensorPreviewHeight= N5600_SENSOR_PV_HEIGHT;
    
			pSensorResolution->SensorVideoWidth=N5600_SENSOR_VIDEO_WIDTH;
			pSensorResolution->SensorVideoHeight=N5600_SENSOR_VIDEO_HEIGHT;	
			break;
		case HSM_ENGINE_N6700:
			pSensorResolution->SensorFullWidth= N6700_SENSOR_FULL_WIDTH;
			pSensorResolution->SensorFullHeight= N6700_SENSOR_FULL_HEIGHT;
			
			pSensorResolution->SensorPreviewWidth= N6700_SENSOR_PV_WIDTH;
			pSensorResolution->SensorPreviewHeight= N6700_SENSOR_PV_HEIGHT;
    
			pSensorResolution->SensorVideoWidth=N6700_SENSOR_VIDEO_WIDTH;
			pSensorResolution->SensorVideoHeight=N6700_SENSOR_VIDEO_HEIGHT;	
			break;
		case HSM_ENGINE_EX30:
		    pSensorResolution->SensorFullWidth= EX30_SENSOR_FULL_WIDTH;
			pSensorResolution->SensorFullHeight= EX30_SENSOR_FULL_HEIGHT;
    
			pSensorResolution->SensorPreviewWidth= EX30_SENSOR_PV_WIDTH;
			pSensorResolution->SensorPreviewHeight= EX30_SENSOR_PV_HEIGHT;
    
			pSensorResolution->SensorVideoWidth=EX30_SENSOR_VIDEO_WIDTH;
			pSensorResolution->SensorVideoHeight=EX30_SENSOR_VIDEO_HEIGHT;
			break;
		case HSM_ENGINE_N4603:
		    pSensorResolution->SensorFullWidth= N4603_SENSOR_FULL_WIDTH;
			pSensorResolution->SensorFullHeight= N4603_SENSOR_FULL_HEIGHT;
    
			pSensorResolution->SensorPreviewWidth= N4603_SENSOR_PV_WIDTH;
			pSensorResolution->SensorPreviewHeight= N4603_SENSOR_PV_HEIGHT;
    
			pSensorResolution->SensorVideoWidth=N4603_SENSOR_VIDEO_WIDTH;
			pSensorResolution->SensorVideoHeight=N4603_SENSOR_VIDEO_HEIGHT;
			break;
		case HSM_ENGINE_N5703:
		    pSensorResolution->SensorFullWidth= N5703_SENSOR_FULL_WIDTH;
			pSensorResolution->SensorFullHeight= N5703_SENSOR_FULL_HEIGHT;
    
			pSensorResolution->SensorPreviewWidth= N5703_SENSOR_PV_WIDTH;
			pSensorResolution->SensorPreviewHeight= N5703_SENSOR_PV_HEIGHT;
    
			pSensorResolution->SensorVideoWidth=N5703_SENSOR_VIDEO_WIDTH;
			pSensorResolution->SensorVideoHeight=N5703_SENSOR_VIDEO_HEIGHT;
			break;
		default:
			pSensorResolution->SensorFullWidth= N6700_SENSOR_FULL_WIDTH;
			pSensorResolution->SensorFullHeight= N6700_SENSOR_FULL_HEIGHT;
			
			pSensorResolution->SensorPreviewWidth= N6700_SENSOR_PV_WIDTH;
			pSensorResolution->SensorPreviewHeight= N6700_SENSOR_PV_HEIGHT;
    
			pSensorResolution->SensorVideoWidth=N6700_SENSOR_VIDEO_WIDTH;
			pSensorResolution->SensorVideoHeight=N6700_SENSOR_VIDEO_HEIGHT;	
	}
    hsm_dbg("[HsmGetResolution]  engine_id = %d,SensorFullWidth = %d,SensorFullHeight = %d,SensorPreviewWidth = %d,SensorPreviewHeight = %d,SensorVideoWidth = %d,SensorVideoHeight = %d\n",
    engine_id,pSensorResolution->SensorFullWidth,pSensorResolution->SensorFullHeight,pSensorResolution->SensorPreviewWidth,pSensorResolution->SensorPreviewHeight,pSensorResolution->SensorVideoWidth,pSensorResolution->SensorVideoHeight);
    return ERROR_NONE;
}	/* HsmGetResolution() */

static kal_uint32 HsmGetInfo(enum MSDK_SCENARIO_ID_ENUM ScenarioId,
        MSDK_SENSOR_INFO_STRUCT *pSensorInfo,
        MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{

    SENSORDB("HsmGetInfo \n");
	hsm_dbg("HsmGetInfo\n");
    pSensorInfo->SensorClockPolarity=SENSOR_CLOCK_POLARITY_LOW;
    pSensorInfo->SensorClockFallingPolarity=SENSOR_CLOCK_POLARITY_LOW;
    pSensorInfo->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
    //pSensorInfo->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_HIGH;
    pSensorInfo->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
     //pSensorInfo->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_HIGH;

    pSensorInfo->MIPIsensorType = MIPI_OPHY_NCSI2;
    pSensorInfo->SensorDrivingCurrent = ISP_DRIVING_6MA;
    pSensorInfo->SettleDelayMode = MIPI_SETTLEDELAY_AUTO;

  
    pSensorInfo->SensorOutputDataFormat=SENSOR_OUTPUT_FORMAT_RAW8_B;//Don't change it! Support N3601,N3603,N6603,N6703

	
    pSensorInfo->SensorInterruptDelayLines = 4; /* not use */
     //pSensorInfo->SensorInterruptDelayLines = 1;
    pSensorInfo->SensorResetActiveHigh = FALSE; /* not use */
    pSensorInfo->SensorResetDelayCount = 5; /* not use */
    
    // pSensorInfo->SensroInterfaceType=SENSOR_INTERFACE_TYPE_PARALLEL;
    pSensorInfo->SensroInterfaceType=SENSOR_INTERFACE_TYPE_MIPI;//MIPI setting

    pSensorInfo->AEShutDelayFrame = 0;       /* The frame of setting shutter default 0 for TG int */
    pSensorInfo->AESensorGainDelayFrame = 0;    /* The frame of setting sensor gain */
    pSensorInfo->AEISPGainDelayFrame = 2;
    pSensorInfo->FrameTimeDelayFrame = 2;

    pSensorInfo->CaptureDelayFrame = 2;
    pSensorInfo->PreviewDelayFrame = 2;
    pSensorInfo->VideoDelayFrame =4;
    pSensorInfo->HighSpeedVideoDelayFrame =2;
    pSensorInfo->SlimVideoDelayFrame =2;

    pSensorInfo->IHDR_Support = 0;
    pSensorInfo->IHDR_LE_FirstLine = 0;

    pSensorInfo->SensorMasterClockSwitch = 0;
    pSensorInfo->SensorModeNum = 2;

    pSensorInfo->SensorClockDividCount = 3; /* not use */
    pSensorInfo->SensorClockRisingCount = 0;
    pSensorInfo->SensorClockFallingCount = 2; /* not use */

    pSensorInfo->SensorPixelClockCount = 3; /* not use */
    pSensorInfo->SensorDataLatchCount = 2; /* not use */
    pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0; 
    pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
    pSensorInfo->SensorWidthSampling = 0;  // 0 is default 1x
    pSensorInfo->SensorHightSampling = 0;   // 0 is default 1x 
    pSensorInfo->SensorPacketECCOrder = 1;
    pSensorInfo->SensorClockFreq=48;  //48;
    
	switch(engine_id)
	{
		case HSM_ENGINE_N3600:
		case HSM_ENGINE_N3603:
			pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_1_LANE;
			break;
		case HSM_ENGINE_N5600:
		case HSM_ENGINE_N6700:
		case HSM_ENGINE_EX30:
		case HSM_ENGINE_N4603:
 		case HSM_ENGINE_N5703:
			pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_1_LANE;  
			break;
		default:
			pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_1_LANE;
			break;
		
	}
  
    switch (ScenarioId)
    {
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
        //case MSDK_SCENARIO_ID_VIDEO_CAPTURE_MPEG4:
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
        //case MSDK_SCENARIO_ID_CAMERA_CAPTURE_MEM:
        default:		
            pSensorInfo->SensorGrabStartX = HSM_SENSOR_GRAB_START_X; 
            pSensorInfo->SensorGrabStartY = HSM_SENSOR_GRAB_START_Y;  
            pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 14;  //85           
            break;
    }

    //memcpy(pSensorConfigData, &HsmSensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));

    return ERROR_NONE;
}	/* HsmGetInfo() */


static kal_uint32 HsmControl(enum MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *pImageWindow,
        MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{
    CAMERA_CONTROL_FLOW(ScenarioId,ScenarioId);
	hsm_dbg("HsmControl enter ScenarioId=%d\n",ScenarioId);
    switch (ScenarioId)
    {
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
        //case MSDK_SCENARIO_ID_VIDEO_CAPTURE_MPEG4:
			hsm_dbg("[hsm_ioctl] PREVIEW\n ");
            Hsm_Preview(pImageWindow, pSensorConfigData);
            break;
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
        case MSDK_SCENARIO_ID_CAMERA_ZSD:
            capture(pImageWindow, pSensorConfigData);
        break;
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
            normal_video(pImageWindow, pSensorConfigData);
        break;
        case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
            hs_video(pImageWindow, pSensorConfigData);
        break;
        case MSDK_SCENARIO_ID_SLIM_VIDEO:
            slim_video(pImageWindow, pSensorConfigData);
        break;

        default:
            return ERROR_INVALID_SCENARIO_ID;
    }
    return TRUE;
}	/* MT9P012Control() */


static kal_uint32 Hsm_SetVideoMode(UINT16 u2FrameRate)
{
    Hsm_Sensor_Driver.MPEG4_encode_mode = KAL_TRUE; 

    if (u2FrameRate == 60)
    {
    }
    else if (u2FrameRate == 30)       
    {
    }
    else 
    {
        hsm_dbg("Wrong frame rate setting \n");
    }   

    hsm_dbg("\n Hsm_SetVideoMode:u2FrameRate=%d\n\n",u2FrameRate);
    return TRUE;
}

UINT32 Hsm_SetSoftwarePWDNMode(kal_bool bEnable)
{
#if 0
    SENSORDB("[Hsm_SetSoftwarePWDNMode] Software Power down enable:%d\n", bEnable);

    if(bEnable) {   // enable software sleep mode   
        Hsm_write_cmos_sensor(0x09, 0x10);
    } else {
        Hsm_write_cmos_sensor(0x09, 0x03);  
    }
#endif
    return TRUE;
}

/*************************************************************************
 * FUNCTION
 *    Hsm_get_size
 *
 * DESCRIPTION
 *    This function return the image width and height of image sensor.
 *
 * PARAMETERS
 *    *sensor_width: address pointer of horizontal effect pixels of image sensor
 *    *sensor_height: address pointer of vertical effect pixels of image sensor
 *
 * RETURNS
 *    None
 *
 * LOCAL AFFECTED
 *
 *************************************************************************/
static void Hsm_get_size(kal_uint16 *sensor_width, kal_uint16 *sensor_height)
{
	switch(engine_id)
	{
		case HSM_ENGINE_N3600:
		case HSM_ENGINE_N3603:
			*sensor_width = N3600_SENSOR_FULL_WIDTH; /* must be 4:3 ??? */
			*sensor_height = N3600_SENSOR_FULL_HEIGHT;
			break;
		case HSM_ENGINE_N5600:
			*sensor_width = N5600_SENSOR_FULL_WIDTH; /* must be 4:3 ??? */
			*sensor_height = N5600_SENSOR_FULL_HEIGHT;
			break;
		case HSM_ENGINE_N6700:
			*sensor_width = N6700_SENSOR_FULL_WIDTH; /* must be 4:3 ??? */
			*sensor_height = N6700_SENSOR_FULL_HEIGHT;
			break;
		case HSM_ENGINE_EX30:
			*sensor_width = EX30_SENSOR_FULL_WIDTH; /* must be 4:3 ??? */
			*sensor_height = EX30_SENSOR_FULL_HEIGHT;
			break;
		case HSM_ENGINE_N4603:
			*sensor_width = N4603_SENSOR_FULL_WIDTH; /* must be 4:3 ??? */
			*sensor_height = N4603_SENSOR_FULL_HEIGHT;
			break;
		case HSM_ENGINE_N5703:
			*sensor_width = N5703_SENSOR_FULL_WIDTH; /* must be 4:3 ??? */
			*sensor_height = N5703_SENSOR_FULL_HEIGHT;
			break;            			
		default:
			*sensor_width = N6700_SENSOR_FULL_WIDTH; /* must be 4:3 ??? */
			*sensor_height = N6700_SENSOR_FULL_HEIGHT;
			break;
	}
	hsm_dbg("[Hsm_get_size]  engine_id = %d,sensor_width = %d,sensor_height = %d\n",engine_id,*sensor_width,*sensor_height);
}


/*************************************************************************
 * FUNCTION
 *    Hsm_feature_control
 *
 * DESCRIPTION
 *    This function control sensor mode
 *
 * PARAMETERS
 *    id: scenario id
 *    image_window: image grab window
 *    cfg_data: config data
 *
 * RETURNS
 *    error code
 *
 * LOCAL AFFECTED
 *
 *************************************************************************/
static kal_uint32 HsmFeatureControl(MSDK_SENSOR_FEATURE_ENUM id, kal_uint8 *para, kal_uint32 *len)
{
    UINT32 *pFeatureData32=(UINT32 *) para;

    UINT16 *feature_return_para_16=(UINT16 *) para;
    unsigned long long *pFeatureData64=(unsigned long long *) para;
	hsm_dbg("[hsm_mipi_raw] HsmFeatureControl id: %d\n",id);
    switch (id)
    {
        case SENSOR_FEATURE_GET_RESOLUTION: /* no use */
            Hsm_get_size((kal_uint16 *)para, (kal_uint16 *)(para + sizeof(kal_uint16)));
            *len = sizeof(kal_uint32);
            break;
        case SENSOR_FEATURE_GET_PERIOD:
            //Hsm_get_period((kal_uint16 *)para, (kal_uint16 *)(para + sizeof(kal_uint16)));
            *feature_return_para_16++ = 1178;//VGA_PERIOD_LINE_NUMS + 42;
            *feature_return_para_16 = 0;//VGA_PERIOD_PIXEL_NUMS;
            *len = sizeof(kal_uint32);
            break;
        case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
            *(kal_uint32 *)para = Hsm_Sensor_Driver.fPV_PCLK;
            *len = sizeof(kal_uint32);
            break;
        case SENSOR_FEATURE_SET_ESHUTTER:
            break;
        case SENSOR_FEATURE_SET_NIGHTMODE: 
            Hsm_night_mode((kal_bool)*(kal_uint16 *)para);
            break;
        case SENSOR_FEATURE_SET_GAIN:
        case SENSOR_FEATURE_SET_FLASHLIGHT:
        case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
            break;
        case SENSOR_FEATURE_SET_REGISTER:
            //Hsm_write_cmos_sensor(((MSDK_SENSOR_REG_INFO_STRUCT *)para)->RegAddr, ((MSDK_SENSOR_REG_INFO_STRUCT *)para)->RegData, sizeof);
            break;
        case SENSOR_FEATURE_GET_REGISTER: /* 10 */
            //((MSDK_SENSOR_REG_INFO_STRUCT *)para)->RegData = N6700_read_cmos_sensor(((MSDK_SENSOR_REG_INFO_STRUCT *)para)->RegAddr);
            break;
        case SENSOR_FEATURE_SET_CCT_REGISTER:
            memcpy(&Hsm_Sensor_Driver.eng.CCT, para, sizeof(Hsm_Sensor_Driver.eng.CCT));
            break;
        case SENSOR_FEATURE_GET_CCT_REGISTER:
        case SENSOR_FEATURE_SET_ENG_REGISTER:
        case SENSOR_FEATURE_GET_ENG_REGISTER:
        case SENSOR_FEATURE_GET_REGISTER_DEFAULT:
        case SENSOR_FEATURE_GET_CONFIG_PARA: /* no use */
            break;
        case SENSOR_FEATURE_CAMERA_PARA_TO_SENSOR:
            break;
        case SENSOR_FEATURE_SENSOR_TO_CAMERA_PARA:
            break;
        case SENSOR_FEATURE_GET_GROUP_COUNT:
        case SENSOR_FEATURE_GET_GROUP_INFO: /* 20 */
        case SENSOR_FEATURE_GET_ITEM_INFO:
        case SENSOR_FEATURE_SET_ITEM_INFO:
        case SENSOR_FEATURE_GET_ENG_INFO:
            break;
        case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
            /*
             * get the lens driver ID from EEPROM or just return LENS_DRIVER_ID_DO_NOT_CARE
             * if EEPROM does not exist in camera module.
             */
            *(kal_uint32 *)para = LENS_DRIVER_ID_DO_NOT_CARE;
            *len = sizeof(kal_uint32);
            break;
#if 0
        case SENSOR_FEATURE_SET_YUV_CMD:
            //HSM_YUVSensorSetting((enum FEATURE_ID)(UINT32 *)para, (UINT32 *)(para+1));

            HSM_YUVSensorSetting((FEATURE_ID)*pFeatureData32, *(pFeatureData32+1));
            break;		    		
        case SENSOR_FEATURE_QUERY:
            Hsm_Query(pSensorFeatureInfo);
            *pFeatureParaLen = sizeof(MSDK_FEATURE_INFO_STRUCT);
            break;		
        case SENSOR_FEATURE_SET_YUV_CAPTURE_RAW_SUPPORT:
            /* update yuv capture raw support flag by *pFeatureData16 */
            break;		
#endif 			
        case SENSOR_FEATURE_SET_VIDEO_MODE:
            Hsm_SetVideoMode(*para);
            break;
        case SENSOR_FEATURE_CHECK_SENSOR_ID:
            Hsm_GetSensorID(pFeatureData32); 
            break; 	
        case SENSOR_FEATURE_SET_SOFTWARE_PWDN:
            Hsm_SetSoftwarePWDNMode((BOOL)*pFeatureData32);        	        	
            break;
#if 0
        case SENSOR_FEATURE_SET_POWER_STATE:
            hsm_dbg("%s, SENSOR_FEATURE_SET_POWER_STATE, state: %d\n", __func__, (int) *para);
            mt_hsm_s_power((int)*para);
            break;
#endif
        case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
            HsmGetDefaultFramerateByScenario((enum MSDK_SCENARIO_ID_ENUM)*pFeatureData64, (MUINT32 *)(uintptr_t)(*(pFeatureData64+1)));
            break;
        case SENSOR_FEATURE_GET_CROP_INFO:
                /*
                printk("SENSOR_FEATURE_GET_CROP_INFO scenarioId:%d\n", (UINT32)*feature_data);
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
                }*/
                break;
        default:
			hsm_dbg("[hsm_ioctl] HsmFeatureControl id %d\n",id);
            hsm_ioctl(id, para, len);
            break;
    }
    return ERROR_NONE;
}


static struct SENSOR_FUNCTION_STRUCT SensorFuncHsm = {
    HsmOpen,
    HsmGetInfo,
    HsmGetResolution,
    HsmFeatureControl,
    HsmControl,
    HsmClose
};

#if 0

#include <linux/proc_fs.h>
#include <linux/uaccess.h>

#define PROC_NAME_OF_Hsm "hsm"
#define BUFFER_LENGTH (280)


#endif


 void hsm_parse_dt(void) {
     struct device_node *np;// = dev->of_node;
     u8 ret;
    
     np = of_find_compatible_node(NULL, NULL, "mediatek,hsm_gpio");

     if (np) {
         g_hsm_power_pinA[0] = of_get_named_gpio_flags(np, "hsm,gpio_vcc_io_host", 0, NULL);
        hsm_dbg("GPIO_VCC_IO_HOST=%d\n", g_hsm_power_pinA[0]);
     } else {
        hsm_dbg("hsm_control get gpio num err.\n");
         //return -1;
     }

     if (np) {
         g_hsm_power_pinA[1] = of_get_named_gpio_flags(np, "hsm,gpio_3v3_led_imgr", 0, NULL);
        hsm_dbg("GPIO_3V3_LED_IMGR=%d\n", g_hsm_power_pinA[1]);
     } else {
        hsm_dbg("hsm_control get gpio num err.\n");
         //return -1;
     }   
     if (np) {
         g_hsm_power_pinA[2] = of_get_named_gpio_flags(np, "hsm,gpio_3v3_led_laser", 0, NULL);
        hsm_dbg("GPIO_3V3_LED_LASER=%d\n", g_hsm_power_pinA[2]);
     } else {
        hsm_dbg("hsm_control get gpio num err.\n");
         //return -1;
     }   
     if (np) {
         g_hsm_power_pinA[3] = of_get_named_gpio_flags(np, "hsm,gpio_vcc_mipi_switch", 0, NULL);
        hsm_dbg("GPIO_VCC_MIPI_SWITCH=%d\n", g_hsm_power_pinA[3]);
     } else {
        hsm_dbg("hsm_control get gpio num err.\n");
         //return -1;
     }   
     if (np) {
         g_hsm_power_pinA[4] = of_get_named_gpio_flags(np, "hsm,gpio_mipi_switch_sel", 0, NULL);
        hsm_dbg("GPIO_MIPI_SWITCH_SEL=%d\n", g_hsm_power_pinA[4]);
     } else {
        hsm_dbg("hsm_control get gpio num err.\n");
         //return -1;
     }   
     
     if (np) {
         g_hsm_power_pinA[5] = of_get_named_gpio_flags(np, "hsm,gpio_aim_on", 0, NULL);
        hsm_dbg("GPIO_AIM_ON=%d\n", g_hsm_power_pinA[5]);
     } else {
        hsm_dbg("hsm_control get gpio num err.\n");
         //return -1;
     }
     if (np) {
         g_hsm_power_pinA[6] = of_get_named_gpio_flags(np, "hsm,gpio_ill_on", 0, NULL);
        hsm_dbg("GPIO_ILL_ON=%d\n", g_hsm_power_pinA[6]);
     } else {
        hsm_dbg("hsm_control get gpio num err.\n");
         //return -1;
     }   
     if (np) {
         g_hsm_power_pinA[7] = of_get_named_gpio_flags(np, "hsm,gpio_eng_reset", 0, NULL);
        hsm_dbg("GPIO_ENG_RESET=%d\n", g_hsm_power_pinA[7]);
     } else {
        hsm_dbg("hsm_control get gpio num err.\n");
         //return -1;
     }     
     if (np) {
         g_hsm_power_pinA[8] = of_get_named_gpio_flags(np, "hsm,gpio_power_ena", 0, NULL);
        hsm_dbg("GPIO_POWER_ENA=%d\n", g_hsm_power_pinA[8]);
     } else {
        hsm_dbg("hsm_control get gpio num err.\n");
         //return -1;
     }   
     if (np) {
         g_hsm_power_pinA[9] = of_get_named_gpio_flags(np, "hsm,gpio_3v3_pwr_en", 0, NULL);
        hsm_dbg("GPIO_3V3_PWR_EN=%d\n", g_hsm_power_pinA[9]);
     } else {
        hsm_dbg("hsm_control get gpio num err.\n");
        //return -1;
     }   
 //add for Hsm
 /*
     ret = gpio_request(g_hsm_power_pinA[0], "gpio_vcc_io_host");
     if(ret<0)
        hsm_dbg("gpio_request failed\n");
     ret = gpio_direction_output(g_hsm_power_pinA[0], 0);
     if(ret<0)
        hsm_dbg("gpio_direction_output failed\n");

     ret = gpio_request(g_hsm_power_pinA[1], "gpio_3v3_led_imgr");
     if(ret<0)
        hsm_dbg("gpio_request failed\n");
     ret = gpio_direction_output(g_hsm_power_pinA[1], 0);
     if(ret<0)
        hsm_dbg("gpio_direction_output failed\n");

     ret = gpio_request(g_hsm_power_pinA[2], "gpio_3v3_led_laser");
     if(ret<0)
        hsm_dbg("gpio_request failed\n");
     ret = gpio_direction_output(g_hsm_power_pinA[2], 0);
     if(ret<0)
        hsm_dbg("gpio_direction_output failed\n");

     ret = gpio_request(g_hsm_power_pinA[3], "gpio_vcc_mipi_switch");
     if(ret<0)
        hsm_dbg("gpio_request failed\n");
     ret = gpio_direction_output(g_hsm_power_pinA[3], 0);
     if(ret<0)
        hsm_dbg("gpio_direction_output failed\n");

     ret = gpio_request(g_hsm_power_pinA[4], "gpio_mipi_switch_sel");
     if(ret<0)
        hsm_dbg("gpio_request failed\n");
     ret = gpio_direction_output(g_hsm_power_pinA[4], 0);
     if(ret<0)
        hsm_dbg("gpio_direction_output failed\n");

     ret = gpio_request(g_hsm_power_pinA[5], "gpio_aim_on");
     if(ret<0)
        hsm_dbg("gpio_request failed\n");
     ret = gpio_direction_output(g_hsm_power_pinA[5], 0);
     if(ret<0)
        hsm_dbg("gpio_direction_output failed\n");

     ret = gpio_request(g_hsm_power_pinA[6], "gpio_ill_on");
     if(ret<0)
        hsm_dbg("gpio_request failed\n");
     ret = gpio_direction_output(g_hsm_power_pinA[6], 0);
     if(ret<0)
        hsm_dbg("gpio_direction_output failed\n");
        */
     ret = gpio_request(g_hsm_power_pinA[7], "gpio_eng_reset");
     if(ret<0)
        hsm_dbg("gpio_request failed\n");
     ret = gpio_direction_output(g_hsm_power_pinA[7], 0);
     if(ret<0)
        hsm_dbg("gpio_direction_output failed\n");

     ret = gpio_request(g_hsm_power_pinA[8], "gpio_power_ena");
     if(ret<0)
        hsm_dbg("gpio_request failed\n");
     ret = gpio_direction_output(g_hsm_power_pinA[8], 0);
     if(ret<0)
        hsm_dbg("gpio_direction_output failed\n");

     if(ret<0)
        hsm_dbg("gpio_direction_output failed\n");
     //return 0;

 }

UINT32 HSM_MIPI_RAW_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc)
{
    /* To Do : Check Sensor status here */
    hsm_dbg("%s\n", __func__);
    hsm_parse_dt();
    if (pfFunc != NULL)
        *pfFunc = &SensorFuncHsm;
        
    //create_hsm_debug_channel();
//    hsm_parse_dt();
    return ERROR_NONE;
}	/* SensorInit() */

