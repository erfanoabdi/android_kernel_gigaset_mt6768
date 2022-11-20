#ifndef __BASIC_UTIL__
#define __BASIC_UTIL__
unsigned short caculate_checksum_u16(unsigned short * buf, unsigned short length);
unsigned short caculate_checksum_u816(unsigned char * buf, unsigned short length);
unsigned int caculate_checksum_ex(unsigned char * buf, unsigned short length);
void delay_us(int usecs);
#endif