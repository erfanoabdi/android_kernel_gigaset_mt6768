#ifndef __SEMI_TOUCH_DEV_H__
#define __SEMI_TOUCH_DEV_H__
#include <linux/i2c.h>

int semi_touch_init(struct i2c_client *client);

int semi_touch_deinit(struct i2c_client *client);

irqreturn_t semi_touch_clear_report(void);

#endif //__SEMI_TOUCH_DEV_H__