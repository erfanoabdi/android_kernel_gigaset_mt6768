#ifndef __SEMI_TOUCH_UPGRADE_H__
#define __SEMI_TOUCH_UPGRADE_H__
#include "semi_touch_function.h"
int semi_touch_memory_write(struct apk_complex_data* apk_comlex_addr);
int semi_touch_memory_read(struct apk_complex_data* apk_comlex_addr);
int semi_touch_run_ram_code(unsigned char code);

#if SEMI_TOUCH_BOOTUP_UPDATE_EN
int semi_touch_bootup_update_check(void);
#else
#define semi_touch_bootup_update_check()        0  
#endif //SEMI_TOUCH_BOOTUP_UPDATE_EN

#if SEMI_TOUCH_ONLINE_UPDATE_EN
int semi_touch_online_update_check(char* file_path);
#else
#define semi_touch_online_update_check(x)        0
#endif //SEMI_TOUCH_ONLINE_UPDATE_EN

#endif //__SEMI_TOUCH_UPGRADE_H__