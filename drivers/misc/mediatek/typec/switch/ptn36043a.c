// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2020 MediaTek Inc.
 */
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of_platform.h>
#include <linux/usb/typec.h>
#include <linux/usb/typec_mux.h>

#include "mux_switch.h"

struct ptn36043a {
	struct device *dev;
	struct typec_switch *sw;
	struct pinctrl *pinctrl;
	struct pinctrl_state *sel_up;
	struct pinctrl_state *sel_down;
	//struct pinctrl_state *enable;
	//struct pinctrl_state *disable;
	struct mutex lock;
};
/*dws config 
脚的状态需要如下配置才可以识别usb3.0
<gpio185>
    <eint_mode>false</eint_mode>
    <def_mode>0</def_mode>
    <inpull_en>true</inpull_en>
    <inpull_selhigh>false</inpull_selhigh>
    <def_dir>OUT</def_dir>
    <out_high>false</out_high>
    <smt>false</smt>
    <ies>true</ies>
</gpio185>
<gpio182>
    <eint_mode>false</eint_mode>
    <def_mode>0</def_mode>
    <inpull_en>false</inpull_en>
    <inpull_selhigh>false</inpull_selhigh>
    <def_dir>IN</def_dir>
    <out_high>true</out_high>
    <smt>false</smt>
    <ies>true</ies>
</gpio182>
<gpio183>
    <eint_mode>false</eint_mode>
    <def_mode>0</def_mode>
    <inpull_en>false</inpull_en>
    <inpull_selhigh>false</inpull_selhigh>
    <def_dir>IN</def_dir>
    <out_high>true</out_high>
    <smt>false</smt>
    <ies>true</ies>
</gpio183>
 <gpio111>
	<eint_mode>false</eint_mode>
	<def_mode>0</def_mode>
	<inpull_en>false</inpull_en>
	<inpull_selhigh>false</inpull_selhigh>
	<def_dir>IN</def_dir>
	<out_high>false</out_high>
	<smt>false</smt>
	<ies>true</ies>
</gpio111>
最终效果			
182: 0010000100//CH1_SET2
183: 0010000100//CH2_SET1
185: 0100000110//sel 插入usb3.0的状态
111: 0000010100//CH1_SET1 

CH2_SET2硬件没有连接不用管
*/

static int ptn36043a_switch_set(struct typec_switch *sw,
			      enum typec_orientation orientation)
{
	struct ptn36043a *fusb = typec_switch_get_drvdata(sw);

	dev_info(fusb->dev, "%s %d\n", __func__, orientation);

	switch (orientation) {
	case TYPEC_ORIENTATION_NONE:
		/* switch off */
		//if (fusb->sel_up)
			//pinctrl_select_state(fusb->pinctrl, fusb->sel_down);
			//pinctrl_select_state(fusb->pinctrl, fusb->sel_up);
		//if (fusb->disable)
		//	pinctrl_select_state(fusb->pinctrl, fusb->disable);
		break;
	case TYPEC_ORIENTATION_NORMAL:
		/* switch cc1 side */
		//if (fusb->enable)
		//	pinctrl_select_state(fusb->pinctrl, fusb->enable);
		if (fusb->sel_up)
			//pinctrl_select_state(fusb->pinctrl, fusb->sel_down);
			pinctrl_select_state(fusb->pinctrl, fusb->sel_up);
		break;
	case TYPEC_ORIENTATION_REVERSE:
		/* switch cc2 side */
		//if (fusb->enable)
		//	pinctrl_select_state(fusb->pinctrl, fusb->enable);
		if (fusb->sel_down)
			//pinctrl_select_state(fusb->pinctrl, fusb->sel_up);
		pinctrl_select_state(fusb->pinctrl, fusb->sel_down);
			
		break;
	default:
		break;
	}

	return 0;
}

static int ptn36043a_pinctrl_init(struct ptn36043a *fusb)
{
	struct device *dev = fusb->dev;
	int ret = 0;

	fusb->pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR(fusb->pinctrl)) {
		ret = PTR_ERR(fusb->pinctrl);
		dev_info(dev, "failed to get pinctrl, ret=%d\n", ret);
		return ret;
	}

	fusb->sel_up =
		pinctrl_lookup_state(fusb->pinctrl, "sel_up");

	if (IS_ERR(fusb->sel_up)) {
		dev_info(dev, "Can *NOT* find sel_up\n");
		fusb->sel_up = NULL;
	} else
		dev_info(dev, "Find sel_up\n");

	fusb->sel_down =
		pinctrl_lookup_state(fusb->pinctrl, "sel_down");

	if (IS_ERR(fusb->sel_down)) {
		dev_info(dev, "Can *NOT* find sel_down\n");
		fusb->sel_down = NULL;
	} else
		dev_info(dev, "Find sel_down\n");

	

	ptn36043a_switch_set(fusb->sw, TYPEC_ORIENTATION_REVERSE);

	return ret;
}

static int ptn36043a_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct ptn36043a *fusb;
	struct typec_switch_desc sw_desc;
	int ret = 0;

	fusb = devm_kzalloc(&pdev->dev, sizeof(*fusb), GFP_KERNEL);
	if (!fusb)
		return -ENOMEM;

	fusb->dev = dev;

	sw_desc.drvdata = fusb;
	sw_desc.fwnode = dev->fwnode;
	sw_desc.set = ptn36043a_switch_set;

	fusb->sw = mtk_typec_switch_register(dev, &sw_desc);
	if (IS_ERR(fusb->sw)) {
		dev_info(dev, "error registering typec switch: %ld\n",
			PTR_ERR(fusb->sw));
		return PTR_ERR(fusb->sw);
	}

	platform_set_drvdata(pdev, fusb);

	ret = ptn36043a_pinctrl_init(fusb);
	if (ret < 0)
		mtk_typec_switch_unregister(fusb->sw);

	dev_info(dev, "%s done\n", __func__);
	return ret;
}

static int ptn36043a_remove(struct platform_device *pdev)
{
	struct ptn36043a *fusb = platform_get_drvdata(pdev);

	mtk_typec_switch_unregister(fusb->sw);
	return 0;
}

static const struct of_device_id ptn36043a_ids[] = {
	{.compatible = "mediatek,ptn36043a",},
	{},
};

static struct platform_driver ptn36043a_driver = {
	.driver = {
		.name = "ptn36043a",
		.of_match_table = ptn36043a_ids,
	},
	.probe = ptn36043a_probe,
	.remove = ptn36043a_remove,
};

module_platform_driver(ptn36043a_driver);

MODULE_DESCRIPTION("ptn36043a Type-C switch driver");
MODULE_LICENSE("GPL v2");

