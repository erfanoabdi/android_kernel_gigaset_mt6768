#ifndef __I2C_COMM_H__
#define __I2C_COMM_H__
#include "head_def.h"

int i2c_write_bytes(struct hal_io_packet* ppacket);
int i2c_read_bytes(struct hal_io_packet* ppacket);
int semi_touch_i2c_init(void);
int semi_touch_i2c_i2c_exit(void);
#endif //__I2C_COMM_H__