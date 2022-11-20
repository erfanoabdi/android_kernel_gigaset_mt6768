#include "basic_util.h"
#include <linux/delay.h>
unsigned short caculate_checksum_u16(unsigned short * buf, unsigned short length)
{
    unsigned short sum, len, i;
    sum =0;
    
    len = length >>1;  //length is in byte unit;
    
    for(i=0; i < len; i++)
        sum += buf[i];
    
    return sum;
}

unsigned short caculate_checksum_u816(unsigned char * buf, unsigned short length)
{
    unsigned int i;
    unsigned short sum = 0;
    for(i = 0; i < length; i++)
       sum += buf[i];
    
    return sum;
}

unsigned int caculate_checksum_ex(unsigned char * buf, unsigned short length)
{
    unsigned int combchk = 0;
    unsigned short k = 0, check = 0, checkex = 0;
    
    for (k = 0; k < length; k++) {
        check   += buf[k];
        checkex += (unsigned short)(k * buf[k]);
    }

    combchk = (checkex<<16) | check;

    return combchk;
}

void delay_us(int usecs)
{
    while(usecs > 1000)
    {
        udelay(1000);
        usecs -= 1000;
    }
    udelay(usecs);
}