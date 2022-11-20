#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/timer.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/input.h>
#include <linux/kthread.h>
#include <linux/wait.h>
#include <linux/spinlock.h>


static unsigned int g_charger_exist = 1; 

static struct class * base_charger_class;


static ssize_t charger_exist_show(struct class *class, struct class_attribute *attr,	char *buf)
{
	return sprintf(buf, "%d\n", g_charger_exist);
}

static struct class_attribute base_charger_attrs[] = {
	__ATTR(charger_exist, S_IRUGO | S_IWUSR, charger_exist_show, NULL),
	__ATTR_NULL,
};


static int base_charger_init(void)
{
	int i = 0,ret = 0;
	
	base_charger_class = class_create(THIS_MODULE, "base_charger");
	if (IS_ERR(base_charger_class))
		return PTR_ERR(base_charger_class);
	for (i = 0; base_charger_attrs[i].attr.name; i++) {
		ret = class_create_file(base_charger_class,&base_charger_attrs[i]);
		if (ret < 0)
		{
			pr_err("base_charger_class error !!\n");
			return ret;
		}
	}
	return ret;
}

static void base_charger_exit(void)
{
	class_destroy(base_charger_class);
}

late_initcall_sync(base_charger_init);
module_exit(base_charger_exit);

MODULE_DESCRIPTION("prize base charger driver");
MODULE_AUTHOR("zhaopengge <zhaopengge@szprize.com>");
MODULE_LICENSE("GPL");
