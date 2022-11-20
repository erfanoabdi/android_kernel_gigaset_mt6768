#ifndef __SEMI_TOUCH_APK__
#define __SEMI_TOUCH_APK__
#include "head_def.h"

#define SEMI_TOUCH_PROC_NAME                "semi_touch_debug"

#if SEMI_TOUCH_APK_NODE_EN
int semi_touch_create_apk_proc(struct sm_touch_dev* st_dev);
int semi_touch_remove_apk_proc(struct sm_touch_dev* st_dev);
#else
#define semi_touch_create_apk_proc(x)     0  
#define semi_touch_remove_apk_proc(x)     0                    
#endif



#endif //__SEMI_TOUCH_APK__