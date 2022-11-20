#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/interrupt.h>
#include <linux/vmalloc.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/wait.h>
#include <linux/spinlock.h>
#include <linux/ctype.h>
#include <linux/semaphore.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/kdev_t.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <asm/uaccess.h>
#include <linux/kthread.h>
#include <linux/input.h>
#if defined(CONFIG_PM_WAKELOCKS)
#include <linux/pm_wakeup.h>
#else
#include <linux/wakelock.h>
#endif
#include <linux/time.h>
#include <linux/string.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/alarmtimer.h>
/*----------------------------------------------------------------------
static variable defination

eoc ---> endoscope
----------------------------------------------------------------------*/
#define ENDOSCOPE_DEVNAME    "endoscope_dev"
#define EN_DEBUG
#if defined(EN_DEBUG)
#define TRACE_FUNC 	printk("[endoscope_dev] function: %s, line: %d \n", __func__, __LINE__);
#define EOC_DEBUG  printk
#else
#define TRACE_FUNC(x,...)
#define EOC_DEBUG(x,...)
#endif
/****************************************************************/
/*******static function defination                             **/
/****************************************************************/
static struct endoscope_device{
	struct alarm eoc_timer;
	struct platform_device* pla_dev;
	struct pinctrl *eoc_pinctrl;
	struct pinctrl_state *eoc_default;
	struct pinctrl_state *eoc_switch_high;
	struct pinctrl_state *eoc_switch_low;
	int  eoc_irq_gpio;
	bool eoc_irq_sta;
	int	 eoc_irq_num;
	struct timespec endtime;
};

static struct endoscope_device* eoc_dev;
int prize_get_endoscope_flag(void)
{
	return gpio_get_value(eoc_dev->eoc_irq_gpio);
}
EXPORT_SYMBOL(prize_get_endoscope_flag);

static enum alarmtimer_restart endoscope_alarm_timer_func(struct alarm *alarm, ktime_t now)
{

	struct endoscope_device *dev = container_of(alarm, struct endoscope_device,eoc_timer);
	
	dev->eoc_irq_sta = gpio_get_value(dev->eoc_irq_gpio);
	EOC_DEBUG("[endoscope dev] endoscope device  --- > %s\n", dev->eoc_irq_sta?"disconnect" :"connect" );

	if (dev->eoc_irq_sta)
	{ 
		pinctrl_select_state(dev->eoc_pinctrl, dev->eoc_switch_low);
	}else{
	
		pinctrl_select_state(dev->eoc_pinctrl, dev->eoc_switch_high);
	}


 	return ALARMTIMER_NORESTART;
}


static void endoscope_delay_detect_timer(struct endoscope_device* dev)
{
 	struct timespec time, time_now;
 	ktime_t ktime;
 	int ret = 0;

	ret = alarm_try_to_cancel(&dev->eoc_timer);
	if (ret < 0) {
		EOC_DEBUG("[endoscope dev]    callback was running, skip timer\n");
		return;
	}

	if(ret == 0)
		EOC_DEBUG("[endoscope dev]    the timer was not active\n");
	if(ret == 1)
		EOC_DEBUG("[endoscope dev]    the timer was active,cancel succ, restart new one\n");
	
 	get_monotonic_boottime(&time_now);
 	time.tv_sec =  2;
 	time.tv_nsec = 0;
 	dev->endtime = timespec_add(time_now, time);
 	ktime = ktime_set(dev->endtime.tv_sec, dev->endtime.tv_nsec);

 	EOC_DEBUG("[endoscope dev]    %s: alarm timer start:%d, %ld %ld\n", __func__, ret,dev->endtime.tv_sec, dev->endtime.tv_nsec);
 	alarm_start(&dev->eoc_timer, ktime);
	
}

static irqreturn_t endoscope_int_handler(int irq, void *dev_id)
{
	TRACE_FUNC;
	endoscope_delay_detect_timer(eoc_dev);
	// disable_irq_nosync(eoc_dev->eoc_irq_num);
	return IRQ_HANDLED;
}


static int endoscope_get_dts_fun(struct endoscope_device* dev)
{
	int ret = 0;
	struct platform_device* pdev = dev->pla_dev;
	
	dev->eoc_pinctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(dev->eoc_pinctrl)) {
		EOC_DEBUG("[endoscope dev]    Cannot find endoscope pinctrl!");
		ret = PTR_ERR(dev->eoc_pinctrl);
	}

	dev->eoc_default= pinctrl_lookup_state(dev->eoc_pinctrl, "default");
	if (IS_ERR(dev->eoc_default)) {
		ret = PTR_ERR(dev->eoc_default);
		EOC_DEBUG("[endoscope dev]    %s : init err, endoscope_default\n", __func__);
	}

	dev->eoc_switch_high = pinctrl_lookup_state(dev->eoc_pinctrl, "endoscope_switch_high");
	if (IS_ERR(dev->eoc_switch_high)) {
		ret = PTR_ERR(dev->eoc_switch_high);
		EOC_DEBUG("[endoscope dev]    %s : init err, endoscope_switch_high\n", __func__);
	}

	dev->eoc_switch_low = pinctrl_lookup_state(dev->eoc_pinctrl, "endoscope_switch_low");
	if (IS_ERR(dev->eoc_switch_low)) {
		ret = PTR_ERR(dev->eoc_switch_low);
		EOC_DEBUG("[endoscope dev]    %s : init err, endoscope_switch_low\n", __func__);
	}

	pinctrl_select_state(dev->eoc_pinctrl, dev->eoc_switch_low);

	return 0;
}

static int endoscope_irq_init(struct endoscope_device* dev)
{
	int irq_flags = 0;
	int ret = 0;
	struct platform_device* pdev = dev->pla_dev;
	
	dev->eoc_irq_gpio = of_get_named_gpio(pdev->dev.of_node, "irq-gpio", 0);
	if (dev->eoc_irq_gpio < 0) {
		EOC_DEBUG("[endoscope dev]    %s: no irq gpio provided.\n", __func__);
		return -1;
	} else {
		EOC_DEBUG("[endoscope dev]    %s: irq gpio provided ok.sy8801_dev->irq_gpio = %d\n", __func__, dev->eoc_irq_gpio);
	}
	dev->eoc_irq_num =	gpio_to_irq(dev->eoc_irq_gpio);

	if (gpio_is_valid(dev->eoc_irq_gpio)) {
		ret = devm_gpio_request_one(&pdev->dev,dev->eoc_irq_gpio,GPIOF_DIR_IN, "endoscope_int");
		if (ret) {
			EOC_DEBUG("[endoscope dev]    %s: irq_gpio request failed\n", __func__);
			return -1;
		}
		irq_flags = IRQF_TRIGGER_FALLING  | IRQF_ONESHOT | IRQF_TRIGGER_RISING;
		ret = devm_request_threaded_irq(&pdev->dev,dev->eoc_irq_num,NULL,endoscope_int_handler, irq_flags, "endoscope", dev);

		if (ret != 0) {
				EOC_DEBUG("[endoscope dev]    failed to request IRQ %d: %d\n", dev->eoc_irq_num, ret);
				return -1;
		}
		EOC_DEBUG("[endoscope dev]    sucess to request IRQ %d: %d\n", dev->eoc_irq_num, ret);

	}else{
		EOC_DEBUG("[endoscope dev]    %s skipping IRQ registration\n", __func__);
	}
	return 0;
}

static int endoscope_probe(struct platform_device *pdev)
{

	int ret;
	TRACE_FUNC;

	eoc_dev = devm_kzalloc(&pdev->dev, sizeof(struct endoscope_device), GFP_KERNEL);
	if(IS_ERR_OR_NULL(eoc_dev)) 
    { 
       ret = PTR_ERR(eoc_dev);
       EOC_DEBUG("[endoscope dev]    failed to devm_kzalloc endoscope_dev %d\n", ret);
	   return -1;
    }
	eoc_dev->pla_dev = pdev;
	
	ret = endoscope_irq_init(eoc_dev);
	if(ret < 0){
		EOC_DEBUG("[endoscope dev]    failed to endoscope_irq_init %d\n", ret);
		return ret;
	}

    ret = endoscope_get_dts_fun(eoc_dev);
	if(ret < 0){
		EOC_DEBUG("[endoscope dev]    failed to endoscope_get_dts_fun %d\n", ret);
		return ret;
	}

	alarm_init(&eoc_dev->eoc_timer, ALARM_BOOTTIME,endoscope_alarm_timer_func);
	
	return 0;
}

static int endoscope_remove(struct platform_device *dev)	
{
	EOC_DEBUG("[endoscope dev]    [endoscope_dev]:endoscope_remove begin!\n");
	EOC_DEBUG("[endoscope dev]    [endoscope_dev]:endoscope_remove Done!\n");
	return 0;
}
static const struct of_device_id endoscope_dt_match[] = {
	{.compatible = "prize,endoscope"},
	{},
};

static struct platform_driver endoscope_driver = {
	.probe	= endoscope_probe,
	.remove  = endoscope_remove,
	.driver    = {
		.name       = "endoscope_Driver",
		.of_match_table = of_match_ptr(endoscope_dt_match),
	},
};

static int __init endoscope_init(void)
{
    int retval = 0;
    TRACE_FUNC;

    EOC_DEBUG("[endoscope dev]    [%s]: endoscope_driver, retval=%d \n!", __func__, retval);
	if (retval != 0) {
		  return retval;
	}
    platform_driver_register(&endoscope_driver);
    return 0;
}

static void __exit endoscope_exit(void)
{
    TRACE_FUNC;
    platform_driver_unregister(&endoscope_driver);
}

module_init(endoscope_init);
module_exit(endoscope_exit);
MODULE_DESCRIPTION("ENDOSCOPE DEVICE driver");
MODULE_AUTHOR("liaojie <liaojie@cooseagroup.com>");
MODULE_LICENSE("GPL");
MODULE_SUPPORTED_DEVICE("ENDOSCOPE device");

