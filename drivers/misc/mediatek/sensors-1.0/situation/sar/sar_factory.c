// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#define pr_fmt(fmt) "<SAR_FAC>" fmt

#include "sar_factory.h"

/* prize, aw add for sar start */
extern int aw_data_debug[3];
/* prize, aw add for sar end */

struct sar_factory_private {
	uint32_t gain;
	uint32_t sensitivity;
	struct sar_factory_fops *fops;
};

static struct sar_factory_private sar_factory;

static int sar_factory_open(struct inode *inode, struct file *file)
{
	return nonseekable_open(inode, file);
}

static int sar_factory_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	return 0;
}

/* prize, aw add for sar start */
void *aw_memdup_user(const void __user *src, size_t len)
{
	void *p = NULL;

	/*
	 * Always use GFP_KERNEL, since copy_from_user() can sleep and
	 * cause pagefault, which makes it pointless to use GFP_NOFS
	 * or GFP_ATOMIC.
	 */
	p = vzalloc(len);
	if (!p) {
		pr_err("sar vzalloc err!");
		return p;
	}

	if (copy_from_user(p, src, len)) {
		pr_err("sar copy_from_user src err!");
		vfree(p);
	}

	return p;
}
/* prize, aw add for sar end */

static long sar_factory_unlocked_ioctl(struct file *file, unsigned int cmd,
					unsigned long arg)
{
	long err = 0;
	void __user *ptr = (void __user *)arg;
	int32_t data_buf[3] = {0};
	struct SENSOR_DATA sensor_data = {0};
	uint32_t flag = 0;
	//add bob
	struct SENSOR_DATA_DUL sensor_data_dul = { 0 };
	struct SAR_SENSOR_DATA aw_sar_sensor_data;
	struct aw_i2c_data *i2c_data;
	unsigned char __user **data_ptrs;
	uint32_t i = 0;
	uint32_t j = 0;
	uint8_t buf[50] = { 0 };
/* prize, aw add for sar end */

	if (_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE, (void __user *)arg,
				 _IOC_SIZE(cmd));
	else if (_IOC_DIR(cmd) & _IOC_WRITE)
		err = !access_ok(VERIFY_READ, (void __user *)arg,
				 _IOC_SIZE(cmd));

	if (err) {
		pr_err("access error: %08X, (%2d, %2d)\n", cmd,
			    _IOC_DIR(cmd), _IOC_SIZE(cmd));
		return -EFAULT;
	}

	switch (cmd) {
	case SAR_IOCTL_INIT:
		if (copy_from_user(&flag, ptr, sizeof(flag)))
			return -EFAULT;
		if (sar_factory.fops != NULL &&
		    sar_factory.fops->enable_sensor != NULL) {
			err = sar_factory.fops->enable_sensor(flag, 200);
			if (err < 0) {
				pr_err("SAR_IOCTL_INIT fail!\n");
				return -EINVAL;
			}
			pr_debug(
				"SAR_IOCTL_INIT, enable: %d, sample_period:%dms\n",
				flag, 200);
		} else {
			pr_err("SAR_IOCTL_INIT NULL\n");
			return -EINVAL;
		}
		return 0;
	case SAR_IOCTL_READ_SENSORDATA:
		if (sar_factory.fops != NULL &&
		    sar_factory.fops->get_data != NULL) {
			err = sar_factory.fops->get_data(data_buf);
			if (err < 0) {
				pr_err(
					"SAR_IOCTL_READ_SENSORDATA read data fail!\n");
				return -EINVAL;
			}
			pr_debug("SAR_IOCTL_READ_SENSORDATA: (%d, %d, %d)!\n",
				data_buf[0], data_buf[1], data_buf[2]);
			sensor_data.x = data_buf[0];
			sensor_data.y = data_buf[1];
			sensor_data.z = data_buf[2];
/* prize, aw add for sar start */
		//	if (copy_to_user(ptr, &sensor_data,
		//					sizeof(sensor_data)))

			sensor_data_dul.x = data_buf[0];
			sensor_data_dul.y = data_buf[1];
			sensor_data_dul.z = data_buf[2];
			sensor_data_dul.x1 = aw_data_debug[0];
			sensor_data_dul.y1 = aw_data_debug[1];
			sensor_data_dul.z1 = aw_data_debug[2];
			
			if (copy_to_user(ptr, &sensor_data_dul, sizeof(sensor_data_dul)))
				return -EFAULT;
/* prize, aw add for sar end */
		} else {
			pr_err("SAR_IOCTL_READ_SENSORDATA NULL\n");
			return -EINVAL;
		}
		return 0;
	case SAR_IOCTL_ENABLE_CALI:
		if (sar_factory.fops != NULL &&
		    sar_factory.fops->enable_calibration != NULL) {
			err = sar_factory.fops->enable_calibration();
			if (err < 0) {
				pr_err(
					"SAR_IOCTL_ENABLE_CALI fail!\n");
				return -EINVAL;
			}
		} else {
			pr_err("SAR_IOCTL_ENABLE_CALI NULL\n");
			return -EINVAL;
		}
		return 0;
	case SAR_IOCTL_GET_CALI:
		if (sar_factory.fops != NULL &&
		    sar_factory.fops->get_cali != NULL) {
			err = sar_factory.fops->get_cali(data_buf);
			if (err < 0) {
				pr_err("SAR_IOCTL_GET_CALI FAIL!\n");
				return -EINVAL;
			}
		} else {
			pr_err("SAR_IOCTL_GET_CALI NULL\n");
			return -EINVAL;
		}

		pr_debug("SAR_IOCTL_GET_CALI: (%d, %d, %d)!\n",
			data_buf[0], data_buf[1], data_buf[2]);
		sensor_data.x = data_buf[0];
		sensor_data.y = data_buf[1];
		sensor_data.z = data_buf[2];
		if (copy_to_user(ptr, &sensor_data, sizeof(sensor_data)))
			return -EFAULT;
		return 0;
	//add bob strat
	case SAR_IOCTL_GET_CFG_DATA:
		pr_err("sar iotcl get cfg data\n");

		if (copy_from_user(&aw_sar_sensor_data,
				(struct SAR_SENSOR_DATA __user *)arg,
				sizeof(aw_sar_sensor_data))) {
			pr_err("sar copy_from_user err!\n");
			return -EFAULT;
		}
		pr_err("sar sensor_data->len = %d\n", aw_sar_sensor_data.num);

		i2c_data = (struct aw_i2c_data *)aw_memdup_user(aw_sar_sensor_data.data,
				aw_sar_sensor_data.num * sizeof(struct aw_i2c_data));
		if (i2c_data == NULL) {
			pr_err("sar aw_memdup_user err!\n");
			return -1;
		}

		data_ptrs = vzalloc(aw_sar_sensor_data.num * sizeof(u8 __user *));
		if (data_ptrs == NULL) {
			pr_err("sar vzalloc err\n");
			vfree(i2c_data);
			return -1;
		}

		for (i = 0; i < aw_sar_sensor_data.num; i++) {
			if (i2c_data[i].len > 256) {
				pr_err("sar i2c_data[i].len > 256 err!\n");
				goto free_cfg_data_hanld;
			}

			data_ptrs[i] = (unsigned char __user *)i2c_data[i].buf;
			i2c_data[i].buf = aw_memdup_user(data_ptrs[i], i2c_data[i].len);
			if (i2c_data[i].buf == NULL) {
				goto free_cfg_data_hanld;
				break;
			}
		}

		pr_err("sar sensor_data: %d", i2c_data[0].buf[0]);
		pr_err("sar sensor_data len: %d", i2c_data[0].len);

		if (sar_factory.fops != NULL && 
			sar_factory.fops->get_cfg_data != NULL) {
			err = sar_factory.fops->get_cfg_data(&i2c_data[0].buf[0], i2c_data[0].len);
			if (err < 0) {
				pr_err("get_cfg_data FAIL!\n");
				goto free_cfg_data_hanld;
			}
		} else {
			pr_err("sar_factory.fops->get_cfg_data NULL\n");
			goto free_cfg_data_hanld;
		}

		if (aw_sar_sensor_data.num == 2) {
		    mdelay(2);
			err = sar_factory.fops->get_cali(data_buf);
			if (err < 0) {
				pr_err("SAR_IOCTL_GET_CALI FAIL!\n");
		    mdelay(2);
					goto free_cfg_data_hanld;
			}
			snprintf(buf, 50, "0x%02x 0x%02x 0x%02x 0x%02x ",
									(data_buf[0] >> 24) & 0xff,
									(data_buf[0] >> 16) & 0xff,
									(data_buf[0] >> 8) & 0xff,
									(data_buf[0] >> 0) & 0xff);
			pr_err("sar %s", buf);
			if (copy_to_user(data_ptrs[1], buf, strlen(buf) + 1)) {
				pr_err("sar copy_to_user err");
			}
		}
free_cfg_data_hanld:
		for (j = 0; j < i; j++) {
			if (i2c_data[j].buf != NULL) {
				vfree(i2c_data[j].buf);
			}
		}
		vfree(data_ptrs);
		vfree(i2c_data);
		return 0;
	//add bob end
	default:
		pr_err("unknown IOCTL: 0x%08x\n", cmd);
		return -ENOIOCTLCMD;
	}
	return 0;
}

#if IS_ENABLED(CONFIG_COMPAT)
static long compat_sar_factory_unlocked_ioctl(struct file *filp,
					       unsigned int cmd,
					       unsigned long arg)
{
	if (!filp->f_op || !filp->f_op->unlocked_ioctl) {
		pr_err(
			"compat_ion_ioctl file has no f_op or no f_op->unlocked_ioctl.\n");
		return -ENOTTY;
	}

	switch (cmd) {
	case COMPAT_SAR_IOCTL_INIT:
	case COMPAT_SAR_IOCTL_READ_SENSORDATA:
	case COMPAT_SAR_IOCTL_ENABLE_CALI:
	case COMPAT_SAR_IOCTL_GET_CALI: {
		pr_debug(
			"compat_ion_ioctl : SAR_IOCTL_XXX command is 0x%x\n",
			cmd);
		return filp->f_op->unlocked_ioctl(
			filp, cmd, (unsigned long)compat_ptr(arg));
	}
	default:
		pr_err("compat_ion_ioctl : No such command!! 0x%x\n", cmd);
		return -ENOIOCTLCMD;
	}
}
#endif
/*----------------------------------------------------------------------------*/
static const struct file_operations _sar_factory_fops = {
	.open = sar_factory_open,
	.release = sar_factory_release,
	.unlocked_ioctl = sar_factory_unlocked_ioctl,
#if IS_ENABLED(CONFIG_COMPAT)
	.compat_ioctl = compat_sar_factory_unlocked_ioctl,
#endif
};

static struct miscdevice _sar_factory_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "sar",
	.fops = &_sar_factory_fops,
};

int sar_factory_device_register(struct sar_factory_public *dev)
{
	int err = 0;

	if (!dev || !dev->fops)
		return -1;
	sar_factory.gain = dev->gain;
	sar_factory.sensitivity = dev->sensitivity;
	sar_factory.fops = dev->fops;
	err = misc_register(&_sar_factory_device);
	if (err) {
		pr_err("sar_factory_device register failed\n");
		err = -1;
	}
	return err;
}

int sar_factory_device_deregister(struct sar_factory_public *dev)
{
	sar_factory.fops = NULL;
	misc_deregister(&_sar_factory_device);
	return 0;
}
