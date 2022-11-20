#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/slab.h>
#include <linux/cdev.h>
#include <linux/err.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/leds.h>
#include <linux/delay.h>
#include <linux/timer.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>
#include <linux/alarmtimer.h>
#include <linux/kobject.h>
#include <linux/mtd/partitions.h>
#include <linux/blkdev.h>
#include <linux/mtd/blktrans.h>
#include <linux/mtd/mtd.h>
#include <linux/platform_device.h>
#include "../../drivers/gpio/gpiolib.h"
#include <linux/of_gpio.h>

unsigned int HAC_GPIO_PIN;

static int hac_gpio_probe(struct platform_device *pdev)
{
//	char flash_data[2] = {0};
//	int flash_flag = 0;
//	u8 index;
	int ret = 0;
	struct device_node *np = pdev->dev.of_node;

	printk("#### hac_gpio_probe name:%s \n", pdev->name);
	
    if(NULL == pdev)
    {
        printk( "hac_gpio_probe failed, pdev is NULL\n");
        return -1;
    }
 
    if(NULL == pdev->dev.of_node)
    {
        printk( "hac_gpio_probe failed, of_node is NULL\n");
        return -2;
    }

	//init gpio to out
	HAC_GPIO_PIN = of_get_named_gpio(np, "gpio_hac_en", 0);
	printk( "hac_gpio_probe pin:%d \n", HAC_GPIO_PIN);
	ret = gpio_request(HAC_GPIO_PIN, "HAC_GPIO_PIN");
	printk( "hac_gpio_probe gpio_request:%d \n", ret);
	gpio_direction_output(HAC_GPIO_PIN, 0);
	
	return 0;
}

static struct of_device_id hacgpio_match_table[] = {
    { .compatible = "prize,hac-gpio",},
    { },
};

static struct platform_driver hac_gpio_driver = {
    .driver = {
        .name  = "hac_gpio",
        .owner  = THIS_MODULE,
        //.groups = groups,
        .of_match_table = hacgpio_match_table,
    },
 
    .probe = hac_gpio_probe,
};

static int __init hac_gpio_init(void)
{
    return platform_driver_register(&hac_gpio_driver);
}
 
static void __exit hac_gpio_exit(void)
{
    platform_driver_unregister(&hac_gpio_driver);
}

late_initcall(hac_gpio_init);
module_exit(hac_gpio_exit);

MODULE_AUTHOR("yantt");
MODULE_DESCRIPTION("fota leds driver");
MODULE_LICENSE("GPL v2");

