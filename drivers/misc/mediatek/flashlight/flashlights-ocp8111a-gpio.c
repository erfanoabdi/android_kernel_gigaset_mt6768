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

#define pr_fmt(fmt) KBUILD_MODNAME ": %s: " fmt, __func__

#include <linux/types.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/workqueue.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/list.h>
#include <linux/delay.h>
#include <linux/pinctrl/consumer.h>

#include "flashlight-core.h"
#include "flashlight-dt.h"

/* define device tree */
/* TODO: modify temp device tree name */
#ifndef OCP8111A_GPIO_DTNAME
#define OCP8111A_GPIO_DTNAME "mediatek,flashlights_ocp8111a_gpio"
#endif

/* TODO: define driver name */
#define OCP8111A_NAME "flashlights-ocp8111a-gpio"

/* define registers */
/* TODO: define register */

/* define mutex and work queue */
static DEFINE_MUTEX(ocp8111a_mutex);
static struct work_struct ocp8111a_work;

/* define pinctrl */
/* TODO: define pinctrl */
#define OCP8111A_PINCTRL_PIN_HWEN 0
#define OCP8111A_PINCTRL_PIN_FLASH 1

#define OCP8111A_PINCTRL_PINSTATE_LOW 0
#define OCP8111A_PINCTRL_PINSTATE_HIGH 1
#define OCP8111A_PINCTRL_STATE_HWEN_HIGH "hwen_high"
#define OCP8111A_PINCTRL_STATE_HWEN_LOW  "hwen_low"

#define OCP8111A_PINCTRL_STATE_FLASH_HIGH "flash_high"
#define OCP8111A_PINCTRL_STATE_FLASH_LOW  "flash_low"

static struct pinctrl *ocp8111a_pinctrl;
static struct pinctrl_state *ocp8111a_hwen_high;
static struct pinctrl_state *ocp8111a_hwen_low;

static struct pinctrl_state *ocp8111a_flash_high;
static struct pinctrl_state *ocp8111a_flash_low;

/* define usage count */
static int use_count;
// prize add by zhuzhengjiang for current 20200814 start
//flight current:12.5*(17-N)  N is number of EN pulse rising edge. N is integer and changed from 1 to 16
#define ocp8111a_LEVEL_FLASH 2
#define AW3641e_LEVEL_TORCH 0
#if 0
static const unsigned char ocp8111a_strobe_level[ocp8111a_LEVEL_FLASH] = {
0,1
};
#endif
//static int duty = 6;
static int Flag_Flash = 0; //0:torch 1:flash
// prize add by zhuzhengjiang for current 20200814 end
/* platform data */
struct ocp8111a_platform_data {
	int channel_num;
	struct flashlight_device_id *dev_id;
};


/******************************************************************************
 * Pinctrl configuration
 *****************************************************************************/
static int ocp8111a_pinctrl_init(struct platform_device *pdev)
{
	int ret = 0;

	/* get pinctrl */
	ocp8111a_pinctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(ocp8111a_pinctrl)) {
		pr_err("Failed to get flashlight pinctrl.\n");
		ret = PTR_ERR(ocp8111a_pinctrl);
		return ret;
	}

	/* TODO: Flashlight HWEN pin initialization */
	ocp8111a_hwen_high = pinctrl_lookup_state(
			ocp8111a_pinctrl, OCP8111A_PINCTRL_STATE_HWEN_HIGH);
	if (IS_ERR(ocp8111a_hwen_high)) {
		pr_err("Failed to init (%s)\n", OCP8111A_PINCTRL_STATE_HWEN_HIGH);
		ret = PTR_ERR(ocp8111a_hwen_high);
	}
	ocp8111a_hwen_low = pinctrl_lookup_state(
			ocp8111a_pinctrl, OCP8111A_PINCTRL_STATE_HWEN_LOW);
	if (IS_ERR(ocp8111a_hwen_low)) {
		pr_err("Failed to init (%s)\n", OCP8111A_PINCTRL_STATE_HWEN_LOW);
		ret = PTR_ERR(ocp8111a_hwen_low);
	}

	/* TODO: Flashlight flash/torch pin initialization */
	ocp8111a_flash_high = pinctrl_lookup_state(
			ocp8111a_pinctrl, OCP8111A_PINCTRL_STATE_FLASH_HIGH);
	if (IS_ERR(ocp8111a_flash_high)) {
		pr_err("Failed to init (%s)\n", OCP8111A_PINCTRL_STATE_FLASH_HIGH);
		ret = PTR_ERR(ocp8111a_flash_high);
	}
	ocp8111a_flash_low = pinctrl_lookup_state(
			ocp8111a_pinctrl, OCP8111A_PINCTRL_STATE_FLASH_LOW);
	if (IS_ERR(ocp8111a_flash_low)) {
		pr_err("Failed to init (%s)\n", OCP8111A_PINCTRL_STATE_FLASH_LOW);
		ret = PTR_ERR(ocp8111a_flash_low);
	}

	return ret;
}

static int ocp8111a_pinctrl_set(int pin, int state)
{
	int ret = 0;

	if (IS_ERR(ocp8111a_pinctrl)) {
		pr_err("pinctrl is not available\n");
		return -1;
	}

	switch (pin) {
	case OCP8111A_PINCTRL_PIN_HWEN:
		if (state == OCP8111A_PINCTRL_PINSTATE_LOW &&
				!IS_ERR(ocp8111a_hwen_low))
			pinctrl_select_state(ocp8111a_pinctrl, ocp8111a_hwen_low);
		else if (state == OCP8111A_PINCTRL_PINSTATE_HIGH &&
				!IS_ERR(ocp8111a_hwen_high))
			pinctrl_select_state(ocp8111a_pinctrl, ocp8111a_hwen_high);
		else
			pr_err("set err, pin(%d) state(%d)\n", pin, state);
		break;

	case OCP8111A_PINCTRL_PIN_FLASH:
		if (state == OCP8111A_PINCTRL_PINSTATE_LOW &&
				!IS_ERR(ocp8111a_flash_low))
			pinctrl_select_state(ocp8111a_pinctrl, ocp8111a_flash_low);
		else if (state == OCP8111A_PINCTRL_PINSTATE_HIGH &&
				!IS_ERR(ocp8111a_flash_high))
			pinctrl_select_state(ocp8111a_pinctrl, ocp8111a_flash_high);
		else
			pr_err("set err, pin(%d) state(%d)\n", pin, state);
		break;

	default:
		pr_err("set err, pin(%d) state(%d)\n", pin, state);
		break;
	}
	pr_debug("pin(%d) state(%d)\n", pin, state);

	return ret;
}

static int ocp8111a_is_torch(int level)
{
	//int torch_level = 0;

	if (level > AW3641e_LEVEL_TORCH)
		return -1;

	return 0;
}

/******************************************************************************
 * ocp8111a operations
 *****************************************************************************/
/* flashlight enable function */
static int ocp8111a_enable(void)
{
	//int pin = 0, state = 1;

	/* TODO: wrap enable function */
	if(Flag_Flash) {
		ocp8111a_pinctrl_set(OCP8111A_PINCTRL_PIN_FLASH,OCP8111A_PINCTRL_PINSTATE_HIGH);
		ocp8111a_pinctrl_set(OCP8111A_PINCTRL_PIN_HWEN,OCP8111A_PINCTRL_PINSTATE_HIGH);
	}
	else {
		ocp8111a_pinctrl_set(OCP8111A_PINCTRL_PIN_FLASH,OCP8111A_PINCTRL_PINSTATE_LOW);
		ocp8111a_pinctrl_set(OCP8111A_PINCTRL_PIN_HWEN,OCP8111A_PINCTRL_PINSTATE_HIGH);
	}

	return 0;
}

/* flashlight disable function */
static int ocp8111a_disable(void)
{
	//int pin = 0, state = 0;

	/* TODO: wrap disable function */
	ocp8111a_pinctrl_set(OCP8111A_PINCTRL_PIN_FLASH,OCP8111A_PINCTRL_PINSTATE_LOW);
	ocp8111a_pinctrl_set(OCP8111A_PINCTRL_PIN_HWEN,OCP8111A_PINCTRL_PINSTATE_LOW);

	return 0;
}

/* set flashlight level */
static int ocp8111a_set_level(int level)
{
	//int pin = 0, state = 0;

	/* TODO: wrap set level function */
	if (!ocp8111a_is_torch(level)) {
		Flag_Flash = 0;
		printk("it is torch mode");
	} else {
		Flag_Flash = 1;
		printk("it is flash mode");
	}
	printk("Flag_Flash =%d",Flag_Flash);

	return 0;
}

/* flashlight init */
static int ocp8111a_init(void)
{
	//int pin = 0, state = 0;

	/* TODO: wrap init function */

	ocp8111a_pinctrl_set(OCP8111A_PINCTRL_PIN_FLASH,OCP8111A_PINCTRL_PINSTATE_LOW);
	ocp8111a_pinctrl_set(OCP8111A_PINCTRL_PIN_HWEN,OCP8111A_PINCTRL_PINSTATE_LOW);

	return 0;
}

/* flashlight uninit */
static int ocp8111a_uninit(void)
{
	//int pin = 0, state = 0;

	/* TODO: wrap uninit function */

	ocp8111a_pinctrl_set(OCP8111A_PINCTRL_PIN_FLASH,OCP8111A_PINCTRL_PINSTATE_LOW);
	ocp8111a_pinctrl_set(OCP8111A_PINCTRL_PIN_HWEN,OCP8111A_PINCTRL_PINSTATE_LOW);

	return 0;
}

/******************************************************************************
 * Timer and work queue
 *****************************************************************************/
static struct hrtimer ocp8111a_timer;
static unsigned int ocp8111a_timeout_ms;

static void ocp8111a_work_disable(struct work_struct *data)
{
	pr_debug("work queue callback\n");
	ocp8111a_disable();
}

static enum hrtimer_restart ocp8111a_timer_func(struct hrtimer *timer)
{
	schedule_work(&ocp8111a_work);
	return HRTIMER_NORESTART;
}


/******************************************************************************
 * Flashlight operations
 *****************************************************************************/
static int ocp8111a_ioctl(unsigned int cmd, unsigned long arg)
{
	struct flashlight_dev_arg *fl_arg;
	int channel;
	ktime_t ktime;
	unsigned int s;
	unsigned int ns;
	//unsigned int  i;// prize add by zhuzhengjiang for current 20200814 start

	fl_arg = (struct flashlight_dev_arg *)arg;
	channel = fl_arg->channel;

	switch (cmd) {
	case FLASH_IOC_SET_TIME_OUT_TIME_MS:
		pr_debug("FLASH_IOC_SET_TIME_OUT_TIME_MS(%d): %d\n",
				channel, (int)fl_arg->arg);
		ocp8111a_timeout_ms = fl_arg->arg;
		break;

	case FLASH_IOC_SET_DUTY:
		pr_debug("FLASH_IOC_SET_DUTY(%d): %d\n",
				channel, (int)fl_arg->arg);
		ocp8111a_set_level(fl_arg->arg);
		break;

	case FLASH_IOC_SET_ONOFF:
		pr_debug("FLASH_IOC_SET_ONOFF(%d): %d\n",
				channel, (int)fl_arg->arg);
		if (fl_arg->arg == 1) {
			if (ocp8111a_timeout_ms) {
				s = ocp8111a_timeout_ms / 1000;
				ns = ocp8111a_timeout_ms % 1000 * 1000000;
				ktime = ktime_set(s, ns);
				hrtimer_start(&ocp8111a_timer, ktime,
						HRTIMER_MODE_REL);
			}
			// prize add by zhuzhengjiang for current 20200814 start
			#if 0
			for(i=0;i < ocp8111a_strobe_level[duty]-1;i++){ //flight current:12.5*(17-N)  N is number of EN pulse rising edge. N is integer and changed from 1 to 16
				ocp8111a_enable();
				udelay(2);
				ocp8111a_disable();			
			}
			udelay(350);	// If high time of EN is larger than TON_DELAY (type 350us), then LED current will start to ramp up to setting value
			// prize add by zhuzhengjiang for current 20200814 start
			#endif
			ocp8111a_enable();
		} else {
			ocp8111a_disable();
			hrtimer_cancel(&ocp8111a_timer);
		}
		break;
	default:
		pr_info("No such command and arg(%d): (%d, %d)\n",
				channel, _IOC_NR(cmd), (int)fl_arg->arg);
		return -ENOTTY;
	}

	return 0;
}

static int ocp8111a_open(void)
{
	/* Move to set driver for saving power */
	return 0;
}

static int ocp8111a_release(void)
{
	/* Move to set driver for saving power */
	return 0;
}

static int ocp8111a_set_driver(int set)
{
	int ret = 0;

	/* set chip and usage count */
	mutex_lock(&ocp8111a_mutex);
	if (set) {
		if (!use_count)
			ret = ocp8111a_init();
		use_count++;
		pr_debug("Set driver: %d\n", use_count);
	} else {
		use_count--;
		if (!use_count)
			ret = ocp8111a_uninit();
		if (use_count < 0)
			use_count = 0;
		pr_debug("Unset driver: %d\n", use_count);
	}
	mutex_unlock(&ocp8111a_mutex);

	return ret;
}

static ssize_t ocp8111a_strobe_store(struct flashlight_arg arg)
{
	ocp8111a_set_driver(1);
	ocp8111a_set_level(arg.level);
	ocp8111a_timeout_ms = 0;
	ocp8111a_enable();
	msleep(arg.dur);
	ocp8111a_disable();
	ocp8111a_set_driver(0);

	return 0;
}

static struct flashlight_operations ocp8111a_ops = {
	ocp8111a_open,
	ocp8111a_release,
	ocp8111a_ioctl,
	ocp8111a_strobe_store,
	ocp8111a_set_driver
};


/******************************************************************************
 * Platform device and driver
 *****************************************************************************/
static int ocp8111a_chip_init(void)
{
	/* NOTE: Chip initialication move to "set driver" for power saving.
	 * ocp8111a_init();
	 */

	return 0;
}

static int ocp8111a_parse_dt(struct device *dev,
		struct ocp8111a_platform_data *pdata)
{
	struct device_node *np, *cnp;
	u32 decouple = 0;
	int i = 0;

	if (!dev || !dev->of_node || !pdata)
		return -ENODEV;

	np = dev->of_node;

	pdata->channel_num = of_get_child_count(np);
	if (!pdata->channel_num) {
		pr_info("Parse no dt, node.\n");
		return 0;
	}
	pr_info("Channel number(%d).\n", pdata->channel_num);

	if (of_property_read_u32(np, "decouple", &decouple))
		pr_info("Parse no dt, decouple.\n");

	pdata->dev_id = devm_kzalloc(dev,
			pdata->channel_num *
			sizeof(struct flashlight_device_id),
			GFP_KERNEL);
	if (!pdata->dev_id)
		return -ENOMEM;

	for_each_child_of_node(np, cnp) {
		if (of_property_read_u32(cnp, "type", &pdata->dev_id[i].type))
			goto err_node_put;
		if (of_property_read_u32(cnp, "ct", &pdata->dev_id[i].ct))
			goto err_node_put;
		if (of_property_read_u32(cnp, "part", &pdata->dev_id[i].part))
			goto err_node_put;
		snprintf(pdata->dev_id[i].name, FLASHLIGHT_NAME_SIZE,
				OCP8111A_NAME);
		pdata->dev_id[i].channel = i;
		pdata->dev_id[i].decouple = decouple;

		pr_info("Parse dt (type,ct,part,name,channel,decouple)=(%d,%d,%d,%s,%d,%d).\n",
				pdata->dev_id[i].type, pdata->dev_id[i].ct,
				pdata->dev_id[i].part, pdata->dev_id[i].name,
				pdata->dev_id[i].channel,
				pdata->dev_id[i].decouple);
		i++;
	}

	return 0;

err_node_put:
	of_node_put(cnp);
	return -EINVAL;
}

static int ocp8111a_probe(struct platform_device *pdev)
{
	struct ocp8111a_platform_data *pdata = dev_get_platdata(&pdev->dev);
	int err;
	int i;

	pr_debug("Probe start.\n");

	/* init pinctrl */
	if (ocp8111a_pinctrl_init(pdev)) {
		pr_debug("Failed to init pinctrl.\n");
		err = -EFAULT;
		goto err;
	}

	/* init platform data */
	if (!pdata) {
		pdata = devm_kzalloc(&pdev->dev, sizeof(*pdata), GFP_KERNEL);
		if (!pdata) {
			err = -ENOMEM;
			goto err;
		}
		pdev->dev.platform_data = pdata;
		err = ocp8111a_parse_dt(&pdev->dev, pdata);
		if (err)
			goto err;
	}

	/* init work queue */
	INIT_WORK(&ocp8111a_work, ocp8111a_work_disable);

	/* init timer */
	hrtimer_init(&ocp8111a_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	ocp8111a_timer.function = ocp8111a_timer_func;
	ocp8111a_timeout_ms = 100;

	/* init chip hw */
	ocp8111a_chip_init();

	/* clear usage count */
	use_count = 0;

	/* register flashlight device */
	if (pdata->channel_num) {
		for (i = 0; i < pdata->channel_num; i++)
			if (flashlight_dev_register_by_device_id(
						&pdata->dev_id[i],
						&ocp8111a_ops)) {
				err = -EFAULT;
				goto err;
			}
	} else {
		if (flashlight_dev_register(OCP8111A_NAME, &ocp8111a_ops)) {
			err = -EFAULT;
			goto err;
		}
	}

	pr_debug("Probe done.\n");

	return 0;
err:
	return err;
}

static int ocp8111a_remove(struct platform_device *pdev)
{
	struct ocp8111a_platform_data *pdata = dev_get_platdata(&pdev->dev);
	int i;

	pr_debug("Remove start.\n");

	pdev->dev.platform_data = NULL;

	/* unregister flashlight device */
	if (pdata && pdata->channel_num)
		for (i = 0; i < pdata->channel_num; i++)
			flashlight_dev_unregister_by_device_id(
					&pdata->dev_id[i]);
	else
		flashlight_dev_unregister(OCP8111A_NAME);

	/* flush work queue */
	flush_work(&ocp8111a_work);

	pr_debug("Remove done.\n");

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id ocp8111a_gpio_of_match[] = {
	{.compatible = OCP8111A_GPIO_DTNAME},
	{},
};
MODULE_DEVICE_TABLE(of, ocp8111a_gpio_of_match);
#else
static struct platform_device ocp8111a_gpio_platform_device[] = {
	{
		.name = OCP8111A_NAME,
		.id = 0,
		.dev = {}
	},
	{}
};
MODULE_DEVICE_TABLE(platform, ocp8111a_gpio_platform_device);
#endif

static struct platform_driver ocp8111a_platform_driver = {
	.probe = ocp8111a_probe,
	.remove = ocp8111a_remove,
	.driver = {
		.name = OCP8111A_NAME,
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = ocp8111a_gpio_of_match,
#endif
	},
};

static int __init flashlight_ocp8111a_init(void)
{
	int ret;

	pr_debug("Init start.\n");

#ifndef CONFIG_OF
	ret = platform_device_register(&ocp8111a_gpio_platform_device);
	if (ret) {
		pr_err("Failed to register platform device\n");
		return ret;
	}
#endif

	ret = platform_driver_register(&ocp8111a_platform_driver);
	if (ret) {
		pr_err("Failed to register platform driver\n");
		return ret;
	}

	pr_debug("Init done.\n");

	return 0;
}

static void __exit flashlight_ocp8111a_exit(void)
{
	pr_debug("Exit start.\n");

	platform_driver_unregister(&ocp8111a_platform_driver);

	pr_debug("Exit done.\n");
}

module_init(flashlight_ocp8111a_init);
module_exit(flashlight_ocp8111a_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Zhengjiang Zhu Prize ");
MODULE_DESCRIPTION("Prize Flashlight OCP8111A GPIO Driver");

