/*
 * GalaxyCore touchscreen driver
 *
 * Copyright (C) 2021 GalaxyCore Incorporated
 *
 * Copyright (C) 2021 Neo Chen <neo_chen@gcoreinc.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include "gcore_drv_common.h"

static int tpd_local_init(void);
static void tpd_suspend(struct device *h);
static void tpd_resume(struct device *h);

static struct tpd_driver_t tpd_device_driver = {
	.tpd_device_name = "gcore",
	.tpd_local_init = tpd_local_init,
	.suspend = tpd_suspend,
	.resume = tpd_resume,
};

static int tpd_local_init(void)
{
	GTP_DEBUG("tpd_local_init start.");

	if (gcore_touch_bus_init()) {
		GTP_ERROR("bus init fail!");
		return -EPERM;
	}

	if (tpd_load_status == 0) {
		GTP_ERROR("add error touch panel driver.");
		gcore_touch_bus_exit();
		return -EPERM;
	}

	GTP_DEBUG("end %s, %d\n", __func__, __LINE__);
	tpd_type_cap = 1;

	return 0;
}

static void tpd_resume(struct device *h)
{
	GTP_DEBUG("TPD resume start...");

#ifdef CONFIG_ENABLE_GESTURE_WAKEUP
	disable_irq_wake(fn_data.gdev->touch_irq);
#endif

#ifdef CONFIG_GCORE_AUTO_UPDATE_FW_HOSTDOWNLOAD
	gcore_request_firmware_update_work(NULL);
#endif

	fn_data.gdev->ts_stat = TS_NORMAL;
	
	GTP_DEBUG("tpd resume end.");
}

static void tpd_suspend(struct device *h)
{
	GTP_DEBUG("TPD suspend start...");

#if GCORE_WDT_RECOVERY_ENABLE
	cancel_delayed_work_sync(&fn_data.gdev->wdt_work);
#endif

	cancel_delayed_work_sync(&fn_data.gdev->fwu_work);

#ifdef CONFIG_ENABLE_GESTURE_WAKEUP
	enable_irq_wake(fn_data.gdev->touch_irq);
#endif

	fn_data.gdev->ts_stat = TS_SUSPEND;
		
	GTP_DEBUG("TPD suspend end.");
}
//prize-add-pengzhipeng-20220708-start
struct tag_videolfb {
	u64 fb_base;
	u32 islcmfound;
	u32 fps;
	u32 vram;
	char lcmname[1];
};
//prize-add-pengzhipeng-20220708-end

static int __init tpd_driver_init(void)
{
	//prize-add-pengzhipeng-20220708-start
	struct tag_videolfb *videolfb_tag = NULL;
	struct device_node *lcm_name;
	unsigned long size = 0;
	GTP_DEBUG("pzp gx tpd_driver_init start.");


	lcm_name = of_find_node_by_path("/chosen");
	if (lcm_name) {
		videolfb_tag = (struct tag_videolfb *)of_get_property(lcm_name,"atag,videolfb",(int *)&size);
		if (!videolfb_tag)
			printk("pzp GCORE read lcm name : %s\n", videolfb_tag->lcmname);
	}
	printk("pzp GCORE Read lcm name : %s %d\n", videolfb_tag->lcmname, strcmp(videolfb_tag->lcmname, "gc7302_fhdp_dsi_vdo_auo_120hz_drv") );

	
	if (strcmp(videolfb_tag->lcmname, "gc7302_fhdp_dsi_vdo_auo_120hz_drv") != 0)
	{
		printk("pzp GCORE TP driver init for MTK111\n");

		return -1;
	}
	//prize-add-pengzhipeng-20220708-end
	
	tpd_get_dts_info();
	if (tpd_driver_add(&tpd_device_driver) < 0) {
		GTP_ERROR("add generic driver failed\n");
	}

	return 0;
}

/* should never be called */
static void __exit tpd_driver_exit(void)
{
	GTP_DEBUG("tpd_driver_exit exit\n");
	tpd_driver_remove(&tpd_device_driver);
}

module_init(tpd_driver_init);	//prize-modify-pengzhipeng-20220708-start
module_exit(tpd_driver_exit);

MODULE_AUTHOR("GalaxyCore, Inc.");
MODULE_DESCRIPTION("GalaxyCore Touch Main Mudule");
MODULE_LICENSE("GPL");
