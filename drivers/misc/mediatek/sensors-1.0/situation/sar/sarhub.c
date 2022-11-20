// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#define pr_fmt(fmt) "[sarhub] " fmt

#include <hwmsensor.h>
#include "sarhub.h"
#include <situation.h>
#include <SCP_sensorHub.h>
#include <linux/notifier.h>
#include "include/scp.h"
//#include "scp_helper.h"   // prize,aw modify
#include "sar_factory.h"       // prize,aw modify
#if defined(CONFIG_PRIZE_HARDWARE_INFO)                                                                                                                                                                                                                                
#include "../../../hardware_info/hardware_info.h"
extern struct hardware_info current_sarsensor_info;
#endif
/* prize add by wuhui for sensorhub sar hardware info 2021.10.9 end*/

/* awinic bob add start */
//#define AW_USB_PLUG_CAIL

#ifdef AW_USB_PLUG_CAIL

#include <linux/notifier.h>
#include <linux/usb.h>
#include <linux/power_supply.h>
#include <linux/regulator/consumer.h>
#include <linux/version.h>

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,9,0)
#define USB_POWER_SUPPLY_NAME   "charger"
#else
#define USB_POWER_SUPPLY_NAME   "usb"
#endif

#define AW_SAR_CONFIG_MTK_CHARGER

#endif
/* awinic bob add end */
//add bob
int aw_data_debug[3];
// prize,aw modify end

static struct situation_init_info sarhub_init_info;
static DEFINE_SPINLOCK(calibration_lock);
struct sarhub_ipi_data {
	bool factory_enable;

	int32_t cali_data[3];
	int8_t cali_status;
	struct completion calibration_done;
};
static struct sarhub_ipi_data *obj_ipi_data;


static int sar_factory_enable_sensor(bool enabledisable,
					 int64_t sample_periods_ms)
{
	int err = 0;
	struct sarhub_ipi_data *obj = obj_ipi_data;

	if (enabledisable == true)
		WRITE_ONCE(obj->factory_enable, true);
	else
		WRITE_ONCE(obj->factory_enable, false);
	if (enabledisable == true) {
		err = sensor_set_delay_to_hub(ID_SAR,
					      sample_periods_ms);
		if (err) {
			pr_err("sensor_set_delay_to_hub failed!\n");
			return -1;
		}
	}
	err = sensor_enable_to_hub(ID_SAR, enabledisable);
	if (err) {
		pr_err("sensor_enable_to_hub failed!\n");
		return -1;
	}
	return 0;
}

static int sar_factory_get_data(int32_t sensor_data[3])
{
	int err = 0;
	struct data_unit_t data;

	err = sensor_get_data_from_hub(ID_SAR, &data);
	if (err < 0) {
		pr_err_ratelimited("sensor_get_data_from_hub fail!!\n");
		return -1;
	}
	sensor_data[0] = data.sar_event.data[0];
	sensor_data[1] = data.sar_event.data[1];
	sensor_data[2] = data.sar_event.data[2];

	return err;
}

static int sar_factory_enable_calibration(void)
{
	return sensor_calibration_to_hub(ID_SAR);
}

static int sar_factory_get_cali(int32_t data[3])
{
	int err = 0;
	struct sarhub_ipi_data *obj = obj_ipi_data;
	int8_t status = 0;

	err = wait_for_completion_timeout(&obj->calibration_done,
					  msecs_to_jiffies(3000));
	if (!err) {
		pr_err("sar factory get cali fail!\n");
		return -1;
	}
	spin_lock(&calibration_lock);
	data[0] = obj->cali_data[0];
	data[1] = obj->cali_data[1];
	data[2] = obj->cali_data[2];
	status = obj->cali_status;
	spin_unlock(&calibration_lock);
/* prize,aw modify for sar start */
//	if (status != 0) {
//		pr_err("sar cali fail!\n");
//		return -2;
//	}
/* prize,aw modify for sar end */
	return 0;
}

/* prize,aw modify for sar start */
static int sar_get_cfg_data(unsigned char *buf, unsigned char count)
{
	pr_err("sar sar_get_cfg_data\n");

	return sensor_cfg_to_hub(ID_SAR, buf, count);
}
/* prize,aw modify for sar end */

static struct sar_factory_fops sarhub_factory_fops = {
	.enable_sensor = sar_factory_enable_sensor,
	.get_data = sar_factory_get_data,
	.enable_calibration = sar_factory_enable_calibration,
	.get_cali = sar_factory_get_cali,
	.get_cfg_data = sar_get_cfg_data,   /* prize,aw modify for sar */
};

static struct sar_factory_public sarhub_factory_device = {
	.gain = 1,
	.sensitivity = 1,
	.fops = &sarhub_factory_fops,
};

static int sar_get_data(int *probability, int *status)
{
	int err = 0;
	struct data_unit_t data;
	uint64_t time_stamp = 0;

	err = sensor_get_data_from_hub(ID_SAR, &data);
	if (err < 0) {
		pr_err_ratelimited("sensor_get_data_from_hub fail!!\n");
		return -1;
	}
	time_stamp		= data.time_stamp;
	*probability	= data.sar_event.data[0];
	return 0;
}
static int sar_open_report_data(int open)
{
	int ret = 0;
#if defined CONFIG_MTK_SCP_SENSORHUB_V1
	if (open == 1)
		ret = sensor_set_delay_to_hub(ID_SAR, 120);
#elif defined CONFIG_NANOHUB

#else

#endif
	ret = sensor_enable_to_hub(ID_SAR, open);
	return ret;
}
static int sar_batch(int flag,
	int64_t samplingPeriodNs, int64_t maxBatchReportLatencyNs)
{
	return sensor_batch_to_hub(ID_SAR,
		flag, samplingPeriodNs, maxBatchReportLatencyNs);
}

static int sar_flush(void)
{
	return sensor_flush_to_hub(ID_SAR);
}

static int sar_recv_data(struct data_unit_t *event, void *reserved)
{
	struct sarhub_ipi_data *obj = obj_ipi_data;
	int32_t value[3] = {0};
	int err = 0;

	if (event->flush_action == FLUSH_ACTION)
		err = situation_flush_report(ID_SAR);
	else if (event->flush_action == DATA_ACTION) {
		value[0] = event->sar_event.data[0];
		value[1] = event->sar_event.data[1];
		value[2] = event->sar_event.data[2];
		//add bob start
		aw_data_debug[0] =  event->sar_event.data[0];
		aw_data_debug[1] =  event->sar_event.data[1];
		aw_data_debug[2] =  event->sar_event.data[2];
		//add bob end
		err = sar_data_report_t(value, (int64_t)event->time_stamp);
	} else if (event->flush_action == CALI_ACTION) {
		spin_lock(&calibration_lock);
		obj->cali_data[0] =
			event->sar_event.x_bias;
		obj->cali_data[1] =
			event->sar_event.y_bias;
		obj->cali_data[2] =
			event->sar_event.z_bias;
		obj->cali_status =
			(int8_t)event->sar_event.status;
/* prize,aw modify for sar start */
		pr_err("sar cali_data[0]: 0x%08x, cali_data[1]: 0x%08x, cali_data[2]: 0x%08x\n",
			obj->cali_data[0], obj->cali_data[1], obj->cali_data[2]);
/* prize,aw modify for sar end */
		spin_unlock(&calibration_lock);
		complete(&obj->calibration_done);
	}
	return err;
}

/* awinic bob add start */
#ifdef AW_USB_PLUG_CAIL

struct aw_sar_ps {
	bool ps_is_present;
	struct work_struct ps_notify_work;
	struct notifier_block ps_notif;
};

static void aw_sar_ps_notify_callback_work(struct work_struct *work)
{
	pr_info("sar Usb insert,going to force calibrate\n");
	sar_factory_enable_calibration();
}

static int aw_sar_ps_get_state(struct power_supply *psy, bool *present)
{
	union power_supply_propval pval = { 0 };
	int retval;

#ifdef AW_SAR_CONFIG_MTK_CHARGER
	retval = power_supply_get_property(psy, POWER_SUPPLY_PROP_ONLINE,
			&pval);
#else
	retval = power_supply_get_property(psy, POWER_SUPPLY_PROP_PRESENT,
			&pval);
#endif
	if (retval) {
		pr_err("sar %s psy get property failed\n", psy->desc->name);
		return retval;
	}
	if (strcmp(psy->desc->name, "usb") == 0) {
		*present = (pval.intval) ? true : false;
		pr_info("sar %s is %s\n", psy->desc->name,
				(*present) ? "present" : "not present");
	}

	return 0;
}

static int aw_sar_ps_notify_callback(struct notifier_block *self,
		unsigned long event, void *p)
{
	struct aw_sar_ps *aw_sar_ps_to_cail = container_of(self, struct aw_sar_ps, ps_notif);
	struct power_supply *psy = p;
	bool present;
	int retval;
	pr_info("sar %s\n", __func__);
	if ((event == PSY_EVENT_PROP_CHANGED)
		&& psy && psy->desc->get_property && psy->desc->name){
		pr_info("sar1 %s\n", __func__);
		retval = aw_sar_ps_get_state(psy, &present);
		if (retval) {
			return retval;
		}
		if (event == PSY_EVENT_PROP_CHANGED) {
			if (aw_sar_ps_to_cail->ps_is_present == present) {
				pr_err("sar ps present state not change\n");
				return 0;
			}
		}
		aw_sar_ps_to_cail->ps_is_present = present;
		schedule_work(&aw_sar_ps_to_cail->ps_notify_work);
	}

	return 0;
}

static int aw_sar_ps_notify_init(struct aw_sar_ps *aw_sar_ps_to_cail)
{
	struct power_supply *psy = NULL;
	int ret = 0;

	pr_info("%s enter\n", __func__);
	INIT_WORK(&aw_sar_ps_to_cail->ps_notify_work, aw_sar_ps_notify_callback_work);
	aw_sar_ps_to_cail->ps_notif.notifier_call = aw_sar_ps_notify_callback;
	ret = power_supply_reg_notifier(&aw_sar_ps_to_cail->ps_notif);
	if (ret) {
		pr_err("sar Unable to register ps_notifier: %d\n", ret);
		return -1;
	}
	psy = power_supply_get_by_name(USB_POWER_SUPPLY_NAME);
	if (psy) {
		ret = aw_sar_ps_get_state(psy, &aw_sar_ps_to_cail->ps_is_present);
		if (ret) {
			pr_err("sar psy get property failed rc=%d\n", ret);
			goto free_ps_notifier;
		}
	}
	return 0;

free_ps_notifier:
	power_supply_unreg_notifier(&aw_sar_ps_to_cail->ps_notif);

	return -1;
}
#endif
/* awinic bob add end*/


static int sarhub_local_init(void)
{
	struct situation_control_path ctl = {0};
	struct situation_data_path data = {0};
	int err = 0;

	struct sarhub_ipi_data *obj;
	/* prize add by wuhui for sensorhub sar hardware info 2021.10.9 start*/
#if defined(CONFIG_SENSORHUB_PRIZE_HARDWARE_INFO)
	struct sensor_hardware_info_t deviceinfo;
#endif
	/* prize add by wuhui for sensorhub sar hardware info 2021.10.9 end*/
	/* awinic bob add start */
#ifdef AW_USB_PLUG_CAIL
		int ret = 0;
		struct aw_sar_ps *aw_sar_ps_to_cail = NULL;
		aw_sar_ps_to_cail = kzalloc(sizeof(*aw_sar_ps_to_cail), GFP_KERNEL);
		if (!aw_sar_ps_to_cail) {
			err = -ENOMEM;
			goto exit_aw_kfree;
		}
#endif
	/* awinic bob add end */

	pr_debug("%s\n", __func__);
	obj = kzalloc(sizeof(*obj), GFP_KERNEL);
	if (!obj) {
		err = -ENOMEM;
		goto exit;
	}

	memset(obj, 0, sizeof(*obj));
	obj_ipi_data = obj;
	WRITE_ONCE(obj->factory_enable, false);
	init_completion(&obj->calibration_done);

	ctl.open_report_data = sar_open_report_data;
	ctl.batch = sar_batch;
	ctl.flush = sar_flush;
	ctl.is_support_wake_lock = true;
	ctl.is_support_batch = false;
	err = situation_register_control_path(&ctl, ID_SAR);
	if (err) {
		pr_err("register sar control path err\n");
		goto exit;
	}

	data.get_data = sar_get_data;
	err = situation_register_data_path(&data, ID_SAR);
	if (err) {
		pr_err("register sar data path err\n");
		goto exit;
	}

	err = sar_factory_device_register(&sarhub_factory_device);
	if (err) {
		pr_err("sar_factory_device register failed\n");
		goto exit;
	}

	/* prize add by wuhui for sensorhub sar hardware info 2021.10.9 start*/
#if defined(CONFIG_SENSORHUB_PRIZE_HARDWARE_INFO)
	err = sensorHub_get_hardware_info(ID_SAR, &deviceinfo);
	if (err < 0)
		pr_err("sensorHub_get_hardware_info ID_SAR fail\n");
	else
	{	
    #if defined(CONFIG_PRIZE_HARDWARE_INFO)
		strlcpy(current_sarsensor_info.chip, deviceinfo.chip, sizeof(current_sarsensor_info.chip));
		strlcpy(current_sarsensor_info.vendor, deviceinfo.vendor, sizeof(current_sarsensor_info.vendor));
		strlcpy(current_sarsensor_info.id, deviceinfo.id, sizeof(current_sarsensor_info.id));
		strlcpy(current_sarsensor_info.more, deviceinfo.more, sizeof(current_sarsensor_info.more));
    #endif
		pr_info("sensorHub_get_hardware_info ID_SAR ok\n");
	}	
#endif
	/* prize add by wuhui for sensorhub sar hardware info 2021.10.9 end*/

	err = scp_sensorHub_data_registration(ID_SAR,
		sar_recv_data);
	if (err) {
		pr_err("SCP_sensorHub_data_registration fail!!\n");
		goto exit;
	}

	/* awinic bob add start */
#ifdef AW_USB_PLUG_CAIL
		pr_err("sar usb_plug_cail\n");
		ret = aw_sar_ps_notify_init(aw_sar_ps_to_cail);
		if (ret < 0) {
			pr_err("sar error creating power supply notify\n");
			goto exit_ps_notify;
		}
#endif
	/* awinic bob add end */
	return 0;
/* awinic bob add start */
#ifdef AW_USB_PLUG_CAIL
exit_ps_notify:
	power_supply_unreg_notifier(&aw_sar_ps_to_cail->ps_notif);
exit_aw_kfree:
	kfree(aw_sar_ps_to_cail);
	aw_sar_ps_to_cail = NULL;
#endif
/* awinic bob add end */

exit:
	return -1;
}
static int sarhub_local_uninit(void)
{
	return 0;
}

static struct situation_init_info sarhub_init_info = {
	.name = "sar_hub",
	.init = sarhub_local_init,
	.uninit = sarhub_local_uninit,
};

static int __init sarhub_init(void)
{
	situation_driver_add(&sarhub_init_info, ID_SAR);
	return 0;
}

static void __exit sarhub_exit(void)
{
	pr_debug("%s\n", __func__);
}

module_init(sarhub_init);
module_exit(sarhub_exit);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("SAR_HUB driver");
MODULE_AUTHOR("Jashon.zhang@mediatek.com");
