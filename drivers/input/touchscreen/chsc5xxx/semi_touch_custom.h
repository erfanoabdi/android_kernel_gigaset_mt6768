#ifndef __SEMI_TOUCH_CUSTOM__
#define __SEMI_TOUCH_CUSTOM__
#include "head_def.h"

#define MAKE_NODE_UNDER_PROC                  0
#define MAKE_NDDE_UNDER_SYS                   1

#define SEMI_TOUCH_PROC_DIR                 "touchscreen"
#define SEMI_TOUCH_MAKE_NODES_DIR           MAKE_NODE_UNDER_PROC


int semi_touch_custom_work(struct sm_touch_dev *st_dev);
int semi_touch_custom_clean_up(void);

#if SEMI_TOUCH_ESD_CHECK_OPEN
int semi_touch_esd_check_prepare(void);
int semi_touch_esd_check_stop(void);
#else
#define semi_touch_esd_check_prepare()      0
#define semi_touch_esd_check_stop()         0
#endif

#if SEMI_TOUCH_GESTURE_OPEN
int semi_touch_gesture_prepare(void);
int semi_touch_gesture_stop(void);
irqreturn_t semi_touch_gesture_report(unsigned char gesture_id);
#else
#define semi_touch_gesture_prepare()        0
#define semi_touch_gesture_stop()           0
#define semi_touch_gesture_report(x)        IRQ_RETVAL(IRQ_HANDLED)
#endif

#if SEMI_TOUCH_GLOVE_OPEN
int semi_touch_glove_prepare(void);
#else
#define semi_touch_glove_prepare()          0
#endif



#endif