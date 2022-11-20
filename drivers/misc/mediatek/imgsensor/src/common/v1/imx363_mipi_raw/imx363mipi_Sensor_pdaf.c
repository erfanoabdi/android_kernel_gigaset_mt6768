#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
#include <linux/slab.h>
#include <linux/types.h>
#include "kd_camera_typedef.h"

#define PFX "IMX363_pdafotp"
#define LOG_INF(format, args...)	pr_debug(PFX "[%s] " format, __FUNCTION__, ##args)

#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"
extern int iReadRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u8 * a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
extern int iWriteRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u16 i2cId);
//extern void kdSetI2CSpeed(u16 i2cSpeed);
//extern int iBurstWriteReg_multi(u8 *pData, u32 bytes, u16 i2cId, u16 transfer_length);
extern int iMultiReadReg(u16 a_u2Addr , u8 * a_puBuff , u16 i2cId, u8 number);


#define USHORT             unsigned short
#define BYTE               unsigned char
//#define kal_uint8			unsigned char
//#define kal_uint16			unsigned short
//#define kal_uint32			unsigned int

#define Sleep(ms) mdelay(ms)

#define IMX363_EEPROM_READ_ID  0xA0
#define IMX363_EEPROM_WRITE_ID   0xA1
#define IMX363_I2C_SPEED        100
#define IMX363_MAX_OFFSET		0xFFFF

#define DATA_SIZE 2048
static BYTE imx363_eeprom_data[DATA_SIZE]= {0};
static bool get_done = false;
static int last_size = 0;
static int last_offset = 0;

static kal_uint16 read_cmos_sensor(kal_uint32 addr)
{
	kal_uint16 get_byte=0;

	char pu_send_cmd[2] = {(char)(addr >> 8), (char)(addr & 0xFF) };
	iReadRegI2C(pu_send_cmd, 2, (u8*)&get_byte, 1, IMX363_EEPROM_READ_ID);

	return get_byte;
}



static bool selective_read_eeprom(kal_uint16 addr, BYTE* data)
{
	char pu_send_cmd[2] = {(char)(addr >> 8) , (char)(addr & 0xFF) };
    if(addr > IMX363_MAX_OFFSET)
        return false;
//	kdSetI2CSpeed(IMX363_I2C_SPEED);

	if(iReadRegI2C(pu_send_cmd, 2, (u8*)data, 1, IMX363_EEPROM_READ_ID)<0)
		return false;
    return true;
}

static bool _read_IMX363_eeprom(kal_uint16 addr, BYTE* data, kal_uint32 size ){
	int i = 0;
	int offset = addr;
	for(i = 0; i < size; i++) {
		if(!selective_read_eeprom(offset, &data[i])){
			return false;
		}
		LOG_INF("read_eeprom 0x%0x 0x%0x\n",offset, data[i]);
		offset++;
	}
	get_done = true;
	last_size = size;
	last_offset = addr;
    return true;
}

bool read_IMX363_eeprom_pdaf( kal_uint16 addr, BYTE* data, kal_uint32 size){
	int i;
	kal_uint32 checksum = 0;

	addr = 0x791;
	size = 496;
	LOG_INF("read IMX363 eeprom PDAF data1, size1 = %d\n", size);
	if(!get_done || last_size != size || last_offset != addr) {
		if(!_read_IMX363_eeprom(addr, imx363_eeprom_data, size)){
			get_done = 0;
            last_size = 0;
            last_offset = 0;
			return false;
		}
	}

	for(i = 0; i < 496; i++)
	{
		checksum += read_cmos_sensor(0x0791 + i);
	}
	if((checksum % 256) == read_cmos_sensor(0x0981))
		LOG_INF("PDAF DATA1 Checkusm OK\n");
	else
		LOG_INF("PDAF DATA1 Checkusm Failed!!!\n");

	addr = 0x983;
	size = 814;
	LOG_INF("read IMX363 eeprom PDAF data2, size2 = %d\n", size);
	if(!get_done || last_size != size || last_offset != addr) {
		if(!_read_IMX363_eeprom(addr, imx363_eeprom_data+496, size)){
			get_done = 0;
            last_size = 0;
            last_offset = 0;
			return false;
		}
	}

	checksum = 0;
	for(i = 0; i < 814; i++)
	{
		checksum += read_cmos_sensor(0x0983 + i);
	}
	if((checksum % 256) == read_cmos_sensor(0xCB1))
		LOG_INF("PDAF DATA2 Checkusm OK\n");
	else
		LOG_INF("PDAF DATA2 Checkusm Failed!!!\n");

	memcpy(data, imx363_eeprom_data, 1310);
    return true;
}

u8 IMX363SX_lsc_data_pdaf[2048]={0};

 int imx363_otp_read_pdaf(void)
{
	kal_uint16 addr;
	kal_uint16 size;
	u8 data_temp=0;
	//unsigned long  checksum_lsc = 0;

//0x010b00ff
	IMX363SX_lsc_data_pdaf[0] = 0xff;  //zy  layout check
	IMX363SX_lsc_data_pdaf[1] = 0x00;  //zy layout check
	IMX363SX_lsc_data_pdaf[2] = 0x0b;  //zy layout check
	IMX363SX_lsc_data_pdaf[3] = 0x1;  //zy layout check

	IMX363SX_lsc_data_pdaf[4] = 0xb;  //bit1:awb bit1:af bit4:lsc zy  choose mode
	IMX363SX_lsc_data_pdaf[5] = 0;  //reserved


	addr = 0x0789;  //af
	size = 4;
	LOG_INF("read IMX363 eeprom, size1 = %d\n", size);
	if(!get_done || last_size != size || last_offset != addr) {
		if(!_read_IMX363_eeprom(addr, IMX363SX_lsc_data_pdaf+6, size)){
			get_done = 0;
            last_size = 0;
            last_offset = 0;
			return false;
		}
	}
	
	data_temp=IMX363SX_lsc_data_pdaf[6];
	IMX363SX_lsc_data_pdaf[6] = IMX363SX_lsc_data_pdaf[7];//Infinity_L;
	IMX363SX_lsc_data_pdaf[7] = data_temp;//Infinity_H;
	data_temp=IMX363SX_lsc_data_pdaf[8];
	IMX363SX_lsc_data_pdaf[8] = IMX363SX_lsc_data_pdaf[9];//Near_L;
	IMX363SX_lsc_data_pdaf[9] = data_temp;//Near_H;

	data_temp=IMX363SX_lsc_data_pdaf[6];
	IMX363SX_lsc_data_pdaf[6] = IMX363SX_lsc_data_pdaf[8];
	IMX363SX_lsc_data_pdaf[8] = data_temp;

	data_temp=IMX363SX_lsc_data_pdaf[7];
	IMX363SX_lsc_data_pdaf[7] = IMX363SX_lsc_data_pdaf[9];
	IMX363SX_lsc_data_pdaf[9] = data_temp;


	addr = 0x0023;  //awb
	size = 8;
	LOG_INF("read IMX363 eeprom, size1 = %d\n", size);
	if(!get_done || last_size != size || last_offset != addr) {
		if(!_read_IMX363_eeprom(addr, IMX363SX_lsc_data_pdaf+10, size)){
			get_done = 0;
            last_size = 0;
            last_offset = 0;
			return false;
		}
	}
	
	LOG_INF("read_cmos_sensor(0x24)=%x\n", read_cmos_sensor(0x24));

	IMX363SX_lsc_data_pdaf[10]= read_cmos_sensor(0x24);
	IMX363SX_lsc_data_pdaf[11]= read_cmos_sensor(0x26);
	IMX363SX_lsc_data_pdaf[12]= read_cmos_sensor(0x28);
	IMX363SX_lsc_data_pdaf[13]= read_cmos_sensor(0x2a);
	
	

	IMX363SX_lsc_data_pdaf[14]= 0x48;
	IMX363SX_lsc_data_pdaf[15]= 0x9f;
	IMX363SX_lsc_data_pdaf[16]= 0xa0;
	IMX363SX_lsc_data_pdaf[17]= 0x5c;
	
	

	IMX363SX_lsc_data_pdaf[18] = 0;  //reserved
	IMX363SX_lsc_data_pdaf[19] = 0;  //reserved


	addr = 0x003b;  //lsc
	size = 1868;
	LOG_INF("read IMX363 eeprom, size1 = %d\n", size);
	if(!get_done || last_size != size || last_offset != addr) {
		if(!_read_IMX363_eeprom(addr, IMX363SX_lsc_data_pdaf+20, size)){
			get_done = 0;
            last_size = 0;
            last_offset = 0;
			return false;
		}
	}

	

    return true;
}

