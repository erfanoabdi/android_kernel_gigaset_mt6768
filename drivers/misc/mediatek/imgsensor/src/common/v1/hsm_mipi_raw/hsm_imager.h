/*
 * Camera driver for barcode imagers from Honeywell.
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _HSM_IMAGER_H
#define _HSM_IMAGER_H

struct hsm_iic_data {
	u8	i2c_addr;
	u16	reg;
	u8	*buf;
	u8	len;
};

struct hsm_gpio_config {
	u8	pin;
	u8	dir;
	u8	def_value;
};

struct hsm_gpio_data {
	u8	pin;
	u8	value;
};

enum hsm_engine_id {
	HSM_ENGINE_UNKNOWN = 0,
	HSM_ENGINE_IT5000,
	HSM_ENGINE_N5600,
	HSM_ENGINE_N6700,
	HSM_ENGINE_N3600,
	HSM_ENGINE_N3603,
	HSM_ENGINE_EX30,
	HSM_ENGINE_N4603,
	HSM_ENGINE_N5703,
	HSM_ENGINE_NUM,
	HSM_ENGINE_UNSUPPORTED
};

enum hsm_gpio_id {
	HSM_GPIO_POWER_ENABLE = 0,
	HSM_GPIO_AIMER,
	HSM_GPIO_ILLUMINATOR,
	HSM_GPIO_ENGINE_RESET,
	HSM_GPIO_VSYNC_GPIO,
	HSM_GPIO_MIPI_RESET,
	HSM_GPIO_POWER_SUPPLY,
	HSM_GPIO_FLASH_OUT,
	HSM_NUM_GPIO,
};

enum hsm_mount_options {
	HSM_MOUNT_RIGHT_SIDE_UP = 0,
	HSM_MOUNT_RIGHT,
	HSM_MOUNT_UPSIDE_DOWN,      /* This is what most devices use. Ears
	                             * up, causing the image upside down. */
	HSM_MOUNT_LEFT,
};

#define HSM_PLATFORM_BUFFER_ALLIGN	0x0001
#define HSM_PLATFORM_PROBE_AFTER_S_INPUT	0x1000
#define HSM_PLATFORM_SUPPORTS_SUPPLY	0x4000
#define HSM_PLATFORM_HAS_STATUS	0x8000
#define HSM_PLATFORM_SET_SKIP_IMAGES(n)	(n<<8)
#define HSM_PLATFORM_GET_SKIP_IMAGES(n)	((n>>8)&0xF)

#define HSM_PROPERTIES_V1 1
struct hsm_engine_properties {
	u32	version;
	u32	imager;
	u32	width;
	u32	height;
	u32	mount;				//!< hsm_mount_options
	u8	i2c_addr_sensor;	//!< Address of camera sensor in 7bit format
	u8	i2c_addr_psoc;		//!< Address of PSOC in 7bit format
	u8	i2c_addr_clock;		//!< Address of clock chip in 7bit format
	u8	i2c_addr_mipi;		//!< Address of the mipi converter in 7bit format
	u32	spreadspectrum;		//!< Currently not used
	u32	platform_details;	//!< Workaround hints for Platforms
	u32	Reserved[3];
};

#define HSM_STATUS_NEED_REINIT 0x1
#define HSM_STATUS_V1 1
struct hsm_status_data {
	u32	version;
	u32	status;
	u32	Reserved;
};

#ifdef __KERNEL__
#define HSM_PLATFORM_VERSION 1
struct hsm_gpio {
	char * name;
	resource_size_t port;
	bool init;
	bool inverted;
};

struct hsm_platform_data {
	unsigned int pixelformat;
	enum hsm_mount_options mount;
	int spreadspectrum;
	int (*hsm_supply_en)(int en); /* This function should enable the VCC on sensor */
	struct hsm_gpio gpio[HSM_NUM_GPIO];
	u8 i2c_addr_mipi;
	int i2c_adapter_id_mipi;
};
#endif

/*
 * Note these are 7 bit I2C device addresses.
 * The R/W bit is not included and the actual address is in D6..D0.  D7 = 0.
 */
#define PSOC_I2C_ADDR       0x40	//!< System control in engine
#define MT9V022_I2C_ADDR    0x48	//!< Image sensor IT5000 (gen5)
#define EV76C454_I2C_ADDR   0x18	//!< Image sensor N5600 (gen6)
#define IDT6P50016_I2C_ADDR 0x69	//!< spread spectrum clock chip
#define TC358748_I2C_ADDR   0x0E	//!< Toshiba 748 parallel to mipi converter
#define TC358746_I2C_ADDR   0x07	//!< Toshiba 746 parallel to mipi converter
#define AR0144_I2C_ADDR     0x18	//!< Image sensor N6700
#define AR0144_I2C_ADDR2    0x10	//!< Image sensor EX30
#define MT9M114_I2C_ADDR	0x48	//!< Image sensor N3601
#define AIMER_I2C_ADDR		0x67
#define ILLUM_I2C_ADDR		0x63
#define N4603_I2C_ADDR		0x30	//!< Image sensor N4603
#define N5703_I2C_ADDR		0x30	//!< Image sensor N5703
#define N5703_I2C_ADDR2		0x31	//!< Image sensor N5703-2

#define N5600_WIDTH		832
#define N5600_HEIGHT	640

#define IT5000_WIDTH	752
#define IT5000_HEIGHT	480

#define N3601_WIDTH		1280
#define N3601_HEIGHT	800

#define N6700_WIDTH		1280
#define N6700_HEIGHT	800

#define EX30_WIDTH		1280
#define EX30_HEIGHT		800

#define N4603_WIDTH		640
#define N4603_HEIGHT	480

#define N5703_WIDTH		800
#define N5703_HEIGHT	1280

#ifndef __KERNEL__
/* This can make integration into user space a little easier. */
#ifndef BASE_VIDIOC_PRIVATE
#define BASE_VIDIOC_PRIVATE		192
#warning "BASE_VIDIOC_PRIVATE not defined. I assumed a value taken from V3.0 kernel."
#endif
#endif

#define HSM_GET_PROPERTIES _IOWR('V', BASE_VIDIOC_PRIVATE + 0, struct hsm_engine_properties)
#define HSM_IIC_WRITE      _IOWR('V', BASE_VIDIOC_PRIVATE + 2, struct hsm_iic_data)
#define HSM_IIC_READ       _IOWR('V', BASE_VIDIOC_PRIVATE + 3, struct hsm_iic_data)
/* #define HSM_GPIO_CONFIG    _IOWR('V', BASE_VIDIOC_PRIVATE + 6, struct hsm_gpio_config) */
#define HSM_GPIO_WRITE     _IOWR('V', BASE_VIDIOC_PRIVATE + 7, struct hsm_gpio_data)
#define HSM_GPIO_READ      _IOWR('V', BASE_VIDIOC_PRIVATE + 8, struct hsm_gpio_data)
#define HSM_IIC_TRANSFER   _IOWR('V', BASE_VIDIOC_PRIVATE + 9, struct i2c_rdwr_ioctl_data)
#define HSM_STATUS_READ    _IOWR('V', BASE_VIDIOC_PRIVATE + 10, struct hsm_status_data)
#define HSM_SUPPLY_WRITE   _IOWR('V', BASE_VIDIOC_PRIVATE + 11, struct hsm_gpio_data)
#define HSM_SUPPLY_READ    _IOWR('V', BASE_VIDIOC_PRIVATE + 12, struct hsm_gpio_data)


#endif /* _HSM_IMAGER_H */

