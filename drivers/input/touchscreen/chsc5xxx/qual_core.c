#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/cdev.h>
#include <linux/ide.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/irq.h>
#include <linux/of_irq.h>
#include <linux/i2c.h>

#include "head_def.h"
#include "semi_touch_device.h"
#include "semi_touch_custom.h"
#include "semi_touch_function.h"

#define semi_io_free(pin)                   do{ if(gpio_is_valid(pin)) gpio_free(pin); }while(0)
#if defined(CONFIG_PRIZE_HARDWARE_INFO)
#include "../../../misc/mediatek/hardware_info/hardware_info.h"
extern struct hardware_info current_tp_info;
#endif
static const struct of_device_id sm_of_match[] = 
{
    {.compatible = "chipsemi,chsc_cap_touch", },
    {}
};

static const struct i2c_device_id sm_ts_id[] = 
{
    {CHSC_DEVICE_NAME, 0},
    {}
};

int semi_touch_get_int(void)
{
    int int_gpio_no = 0;
    struct device_node* of_node = NULL;
    //of_node = of_find_node_by_name(NULL, "smtouch");
    //check_return_if_zero(of_node, NULL);
    of_node = of_find_matching_node(NULL, sm_of_match);
    check_return_if_zero(of_node, NULL);

    int_gpio_no = of_get_named_gpio(of_node, "chipsemi,int-gpio", 0);
    check_return_if_fail(int_gpio_no, NULL);

    gpio_request(int_gpio_no, "chsc_int_pin");

    return int_gpio_no;

    //return of_get_named_gpio(of_node, "chipsemi,int-gpio", 0);
}

int semi_touch_get_rst(void)
{
    int rst_gpio_no = 0;
    struct device_node* of_node = NULL;
    //of_node = of_find_node_by_name(NULL, "smtouch");
    //check_return_if_zero(of_node, NULL);
    of_node = of_find_matching_node(NULL, sm_of_match);
    check_return_if_zero(of_node, NULL);

    rst_gpio_no = of_get_named_gpio(of_node, "chipsemi,rst-gpio", 0);
    check_return_if_fail(rst_gpio_no, NULL);

    gpio_request(rst_gpio_no, "chsc_rst_pin");

    return rst_gpio_no;
}

int semi_touch_get_irq(int rst_pin)
{
    int irq_no = 0;

    gpio_set_debounce(rst_pin, 50);

    irq_no = gpio_to_irq(rst_pin);

    return irq_no;
}

int semi_touch_platform_variety(void)
{
    if(st_dev.int_pin) 
    {
        semi_io_free(st_dev.int_pin);
    }
        
    if(st_dev.rst_pin)
    {
        semi_io_free(st_dev.rst_pin);
    }
        

    return 0;
}

// int semi_touch_work_done(void)
// {
//     platform_variety();

//     return 0;
// }

/********************************************************************************************************************************/
/*proximity support*/
#if SEMI_TOUCH_PROXIMITY_OPEN
#include <linux/input.h>
#define PROXIMITY_CLASS_NAME            "chsc_tpd"
#define PROXIMITY_DEVICE_NAME           "device"

/* default cmd interface(refer to sensor HAL):"/sys/class/chsc-tpd/device/proximity" */

struct chsc_proximity
{
    struct class *proximity_cls;
    struct device *proximity_dev;
    struct input_dev *proximity_input;    
};

static struct chsc_proximity proximity_obj;

int semi_touch_proximity_init(void)
{
    int ret = 0;

    proximity_obj.proximity_cls = class_create(THIS_MODULE, PROXIMITY_CLASS_NAME);
    check_return_if_fail(proximity_obj.proximity_cls, NULL);

    proximity_obj.proximity_dev = device_create(proximity_obj.proximity_cls, NULL, 0, NULL, PROXIMITY_DEVICE_NAME);
    check_return_if_fail(proximity_obj.proximity_cls, NULL);

    proximity_obj.proximity_input = input_allocate_device();
    check_return_if_zero(proximity_obj.proximity_input, NULL);

    proximity_obj.proximity_input->name = "proximity_tp";
    set_bit(EV_ABS, proximity_obj.proximity_input->evbit);
    input_set_capability(proximity_obj.proximity_input, EV_ABS, ABS_DISTANCE);
    input_set_abs_params(proximity_obj.proximity_input, ABS_DISTANCE, 0, 1, 0, 0);
    ret = input_register_device(proximity_obj.proximity_input);
    check_return_if_fail(ret, NULL);

    open_proximity_function(st_dev.stc.custom_function_en);

    return ret;
}

irqreturn_t semi_touch_proximity_report(unsigned char proximity)
{
    if(is_proximity_function_en(st_dev.stc.custom_function_en))
    {
        input_report_abs(proximity_obj.proximity_input, ABS_DISTANCE, proximity);
        input_mt_sync(proximity_obj.proximity_input);
        input_sync(proximity_obj.proximity_input);
    }

    return IRQ_RETVAL(IRQ_HANDLED);
}

int semi_touch_proximity_stop(void)
{
    if(proximity_obj.proximity_input)
    {
        input_unregister_device(proximity_obj.proximity_input);
        input_free_device(proximity_obj.proximity_input);
    }
    if(proximity_obj.proximity_dev)
    {
        device_destroy(proximity_obj.proximity_cls, 0);
    }
    if(proximity_obj.proximity_cls)
    {
        class_destroy(proximity_obj.proximity_cls);
    }

    return 0;
}
#endif

/********************************************************************************************************************************/
#if(!defined(CONFIG_FB) && !defined(CONFIG_HAS_EARLYSUSPEND))
static const struct dev_pm_ops semi_touch_dev_pm_ops = 
{
    .suspend = semi_touch_suspend_entry,
    .resume = semi_touch_resume_entry,
};
#else
static const struct dev_pm_ops semi_touch_dev_pm_ops = 
{

};
#endif

#if defined(CONFIG_FB)
#include <linux/notifier.h>
#include <linux/fb.h>
struct notifier_block sm_fb_notify;
static int semi_touch_fb_notifier_callback(struct notifier_block *self, unsigned long event, void *data)
{
    int *blank;
    struct fb_event *evdata = data;
    if(evdata && evdata->data && FB_EVENT_BLANK == event && st_dev.client)
    {
        blank = evdata->data;
        if(FB_BLANK_UNBLANK == *blank)
            semi_touch_resume_entry(&st_dev.client->dev);
        else if(FB_BLANK_POWERDOWN == *blank)
            semi_touch_suspend_entry(&st_dev.client->dev);
    }

    return 0;
}
int semi_touch_work_done(void)
{
    int ret = 0;
    sm_fb_notify.notifier_call = semi_touch_fb_notifier_callback;
    ret = fb_register_client(&sm_fb_notify);
    check_return_if_fail(ret, NULL);

    return ret;
}
int semi_touch_resource_release(void)
{
    fb_unregister_client(&sm_fb_notify);
    return semi_touch_platform_variety();
}
#elif defined(CONFIG_DRM)
#include <linux/notifier.h>
#include <drm/drm_panel.h>
struct drm_panel *active_panel = NULL;
static int semi_touch_drm_notifier_callback(struct notifier_block *self, unsigned long event, void *data)
{
    int *blank;
    struct drm_panel_notifier *evdata = (struct drm_panel_notifier *)data;

    if(evdata && evdata->data && DRM_PANEL_EVENT_BLANK == event && st_dev.client)
    {
        blank = evdata->data;
        if(DRM_PANEL_BLANK_UNBLANK == *blank)
            semi_touch_resume_entry(&st_dev.client->dev);
        else if(DRM_PANEL_BLANK_POWERDOWN == *blank)
            semi_touch_suspend_entry(&st_dev.client->dev);
        
        //kernel_log_d("drm event = %lu, blank = %d\n", event, *blank);
    }

    return 0;
}
static int semi_touch_drm_get_panel(struct device_node * np)
{
    int index, count;
    struct device_node *node = NULL;
    struct drm_panel *panel = NULL;

    count = of_count_phandle_with_args(np, "panel", NULL);
    if(count <= 0) return -SEMI_DRV_INVALID_PARAM;

    for(index = 0; index < count; index++)
    {
        node = of_parse_phandle(np, "panel", index);
        panel = of_drm_find_panel(node);
        of_node_put(node);
        if (!IS_ERR(panel)) 
        {
            active_panel = panel;
            return SEMI_DRV_ERR_OK;
        }
    }

    return -SEMI_DRV_ERR_NOT_MATCH;
}
int semi_touch_work_done(void)
{
    int ret = 0;

    ret = semi_touch_drm_get_panel(st_dev.client->dev.of_node);
    check_return_if_fail(ret, NULL);

    kernel_log_d("register drm notify, active = %x\n", active_panel);

    sm_fb_notify.notifier_call = semi_touch_drm_notifier_callback;
    ret = drm_panel_notifier_register(active_panel, &sm_fb_notify);
    check_return_if_fail(ret, NULL);

    return ret;
}
int semi_touch_resource_release(void)
{
    if(NULL != active_panel)
    {
        drm_panel_notifier_unregister(active_panel, &sm_fb_notify);
    }
    return semi_touch_platform_variety();
}
#elif defined(CONFIG_HAS_EARLYSUSPEND)
#include <linux/earlysuspend.h>
struct early_suspend esp;
static void semi_touch_early_suspend(struct early_suspend* h)
{
    if(NULL == h) return;

    semi_touch_suspend_entry(&st_dev.client->dev);
}
static void semi_touch_late_resume(struct early_suspend* h)
{
    if(NULL == h) return;

    semi_touch_resume_entry(&st_dev.client->dev);
}
int semi_touch_work_done(void)
{
    esp.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
    esp.suspend = semi_touch_early_suspend;
    esp.resume = semi_touch_late_resume;
    register_early_suspend(&esp);

    return 0;
}
int semi_touch_resource_release(void)
{
    unregister_early_suspend(&esp);
    return semi_touch_platform_variety();
}
#else
int semi_touch_work_done(void)
{
    return 0;
}
int semi_touch_resource_release(void)
{
    return semi_touch_platform_variety();
}
#endif

static ssize_t chsc_version_hardinfo()
{
    int ret;
    unsigned char readBuffer[8] = {0};
  //  kernel_buffer_prepare(szCopy, szKernel, 128);

    ret = semi_touch_read_bytes(0x20000000 + 0x80, readBuffer, 8);
    check_return_if_fail(ret, NULL);
	
    #if defined(CONFIG_PRIZE_HARDWARE_INFO)
	    sprintf(current_tp_info.chip, "%s","CHSC5448");
		sprintf(current_tp_info.vendor, "0x%02x",readBuffer[1]);
		sprintf(current_tp_info.id,"0x%02x",(readBuffer[3] << 8) + readBuffer[2]);
	   	sprintf(current_tp_info.more,"touchpanel");
    #endif
	
 //   szCopy += sprintf(szCopy, "Ic type %s\n", mapping_ic_from_type(readBuffer[0]));

 //   szCopy += sprintf(szCopy, "config version is %02X\n", readBuffer[1]);
  //  szCopy += sprintf(szCopy, "vender id is %d, ", readBuffer[4]);
  //  szCopy += sprintf(szCopy, "product id is %d\n", (readBuffer[3] << 8) + readBuffer[2]);

  //  ret = semi_touch_read_bytes(0x20000000 + 0x10, readBuffer, 8);
  //  check_return_if_fail(ret, NULL);

//    szCopy += sprintf(szCopy, "boot version is %04X\n", ((readBuffer[5] << 8) + readBuffer[4]));
  //  szCopy += sprintf(szCopy, "driver version is %s\n", CHSC_DRIVER_VERSION);

    //count = szCopy - szKernel;

//kernel_buffer_to_entry(szKernel, count);

    return 0;
}

static int semi_touch_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int ret = 0;

    ret = semi_touch_init(client);
    if(SEMI_DRV_ERR_OK != ret)
    {
        semi_touch_deinit(client);
    }
    check_return_if_fail(ret, NULL);
    chsc_version_hardinfo();
    kernel_log_d("semitouch probe finished\r\n");

    return ret;
}

static int semi_touch_remove(struct i2c_client *client)
{
    int ret = 0;

    ret = semi_touch_deinit(client);

    return ret;
}

int semi_touch_suspend_entry(struct device* dev)
{
    //struct i2c_client *client = st_dev.client;

    if(is_proximity_function_en(st_dev.stc.custom_function_en))
    {
        if(is_proximity_activate(st_dev.stc.ctp_run_status))
        {
            kernel_log_d("proximity is active, so fake suspend...");
            return SEMI_DRV_ERR_OK;
        }
    }

    if(is_guesture_function_en(st_dev.stc.custom_function_en))
    {
        semi_touch_guesture_switch(1);
    }
    else
    {
        semi_touch_suspend_ctrl(1);
        semi_touch_clear_report();
        //disable_irq(client->irq);
        kernel_log_d("tpd real suspend...\n");
    }

    return SEMI_DRV_ERR_OK;
}

int semi_touch_resume_entry(struct device* dev)
{
    unsigned char bootCheckOk = 0;
    unsigned char glove_activity = is_glove_activate(st_dev.stc.ctp_run_status);

    if(is_proximity_function_en(st_dev.stc.custom_function_en))
    {
        if(is_proximity_activate(st_dev.stc.ctp_run_status))
        {
            kernel_log_d("proximity is active, so fake resume...");
            return SEMI_DRV_ERR_OK;
        }
    }

    //reset tp + iic detected
    semi_touch_reset_and_detect();
    //set_status_pointing(st_dev.stc.ctp_run_status);
    semi_touch_clear_report();
    //enable_irq(client->irq);

    if(glove_activity)
    {
        semi_touch_start_up_check(&bootCheckOk);
        if(bootCheckOk)
        {
            semi_touch_glove_switch(1);
        }
    }
    kernel_log_d("tpd_resume...\r\n");

    return SEMI_DRV_ERR_OK;
}

static struct i2c_driver sm_touch_driver = 
{
    .driver = 
    {
        .owner = THIS_MODULE,
        .name = "semi_touch",
        .of_match_table = of_match_ptr(sm_of_match),
#if CONFIG_PM
        .pm = &semi_touch_dev_pm_ops,
#endif
    },
    .id_table = sm_ts_id,
    .probe = semi_touch_probe,
    .remove = semi_touch_remove,
};

static int __init i2c_device_init(void)
{
    int ret = 0;

    ret = i2c_add_driver(&sm_touch_driver);
    check_return_if_fail(ret, NULL);

    return ret;
}

static void __exit i2c_device_exit(void)
{
    i2c_del_driver(&sm_touch_driver);
}

module_init(i2c_device_init);
module_exit(i2c_device_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("wasim");
