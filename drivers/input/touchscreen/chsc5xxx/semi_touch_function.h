#ifndef __SEMI_TOUCH_FUNCTION__
#define __SEMI_TOUCH_FUNCTION__
#include "head_def.h"
#include "basic_util.h"
#include "i2c_communication.h"

#define semi_touch_watch_dog_feed(watch_dog_feed)            (watch_dog_feed = true)
#define semi_touch_check_watch_dog_feed(watch_dog_feed)      (watch_dog_feed == true)
#define semi_touch_reset_watch_dog(watch_dog_feed)           (watch_dog_feed = false)

int semi_touch_reset(enum reset_action action);
int semi_touch_device_prob(void);
int semi_touch_reset_and_detect(void);
int semi_touch_start_up_check(unsigned char* checkOK);
int semi_touch_heart_beat(void);
int semi_touch_write_bytes(unsigned int reg, const unsigned char* buffer, unsigned short len);
int semi_touch_read_bytes(unsigned int reg, unsigned char* buffer, unsigned short len);
int cmd_send_to_tp(struct m_ctp_cmd_std_t *ptr_cmd, struct m_ctp_rsp_std_t *ptr_rsp, int once_delay);
int cmd_send_to_tp_no_check(struct m_ctp_cmd_std_t *ptr_cmd);
int read_and_report_touch_points(unsigned char *readbuffer, unsigned short len);
int semi_touch_mode_init(struct sm_touch_dev *st_dev);

int semi_touch_suspend_ctrl(unsigned char en);
int semi_touch_glove_switch(unsigned char en);
int semi_touch_guesture_switch(unsigned char en);
int semi_touch_proximity_switch(unsigned char en);
int semi_touch_orientation_switch(unsigned char en);

//int semi_touch_create_queue(enum work_queue_t queue_type);
int semi_touch_queue_asyn_work(enum work_queue_t queue_type, work_func_t work_func);
int semi_touch_create_work_queue(enum work_queue_t queue_type, const char* queue_name); 
int semi_touch_destroy_work_queue(void);

#endif