/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#ifndef __SAR_FACTORY_H__
#define __SAR_FACTORY_H__

#include <linux/atomic.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/kobject.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>

#include <hwmsen_helper.h>
#include <hwmsensor.h>
#include <sensors_io.h>

//add Bob start
struct SENSOR_DATA_DUL {
    int x;
    int y;
    int z;
    int x1; 
    int y1; 
    int z1; 
};

struct aw_i2c_data {
    unsigned char len;
    unsigned char flag;
    unsigned char *buf;
};

struct SAR_SENSOR_DATA {
    struct aw_i2c_data __user *data;
    unsigned char num;
};
//add Bob end

struct sar_factory_fops {
	int (*enable_sensor)(bool enabledisable,
			int64_t sample_periods_ms);
	int (*get_data)(int32_t sensor_data[3]);
	int (*enable_calibration)(void);
	int (*get_cali)(int32_t data[3]);
	int (*get_cfg_data)(unsigned char *, unsigned char); /* prize add for sar */
};

struct sar_factory_public {
	uint32_t gain;
	uint32_t sensitivity;
	struct sar_factory_fops *fops;
};
int sar_factory_device_register(struct sar_factory_public *dev);
int sar_factory_device_deregister(struct sar_factory_public *dev);
#endif
