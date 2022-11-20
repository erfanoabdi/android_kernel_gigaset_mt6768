
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/uaccess.h>
#include <linux/delay.h>

#include "Koobee_FW_v202_d0628_set_FW_Word_Format.h"
#include "Koobee_RegisterFun_201127.h"

#define kal_uint16 unsigned short
#define AF_DRVNAME "dw9781c-ois"
#define OIS_DEBUG 1
#ifdef OIS_DEBUG
#define LOG_INF(format, args...)                                               \
	pr_info(AF_DRVNAME " [%s] " format, __func__, ##args)
#else
#define LOG_INF(format, args...)
#endif

#define I2C_BUFFER_LEN 256
#define I2C_WRITE_SPEED 400
static DEFINE_SPINLOCK(g_pOIS_SpinLock);


#ifndef I2C_WR_FLAG
#define I2C_WR_FLAG		(0x1000)
#define I2C_MASK_FLAG	(0x00ff)
#endif

/*prize add by zhuzhengjiang end*/


typedef struct
{
	unsigned int driverIc;
	unsigned int size;
	unsigned short *fwContentPtr;
	unsigned short version;
}FirmwareContex;

FirmwareContex g_firmwareContext;
unsigned short g_downloadByForce;
/*prize add by zhuzhengjiang start*/
extern struct i2c_client *g_pstAF_I2Cclient;
static int write_reg_16bit_value_16bit(kal_uint16 addr, kal_uint16 para)
{
	int i4RetValue = 0;
	char puSendCmd[4] = {
		(char)(addr >> 8), (char)(addr & 0xFF),
		(char)(para >> 8), (char)(para & 0xFF) };
	//LOG_INF("I2C write addr=0x%x  para=0x%x\n",addr,para);
	if(g_pstAF_I2Cclient == NULL) {
		LOG_INF("I2C write ERR!! not af i2c client\n");
		return -1;
	}
	spin_lock(&g_pOIS_SpinLock);
	g_pstAF_I2Cclient->addr = OIS_I2C_SLAVE_ADDR;

	g_pstAF_I2Cclient->addr = g_pstAF_I2Cclient->addr >> 1;
	spin_unlock(&g_pOIS_SpinLock);

	i4RetValue = i2c_master_send(g_pstAF_I2Cclient, puSendCmd, 4);

	if (i4RetValue < 0) {
		LOG_INF("I2C write failed!!\n");
		return -1;
	}

	return 0;
}
static int read_reg_16bit_value_16bit(kal_uint16 addr,kal_uint16 *value)
{
	int i4RetValue = 0;
	char pBuff[2];

	char puSendCmd[2] = {(char)(addr >> 8), (char)(addr & 0xFF)};
	if(g_pstAF_I2Cclient == NULL)
		return -1;
	spin_lock(&g_pOIS_SpinLock);

	g_pstAF_I2Cclient->addr = OIS_I2C_SLAVE_ADDR;

	g_pstAF_I2Cclient->addr = g_pstAF_I2Cclient->addr >> 1;
	spin_unlock(&g_pOIS_SpinLock);

	i4RetValue = i2c_master_send(g_pstAF_I2Cclient, puSendCmd, 2);

	if (i4RetValue < 0) {
		LOG_INF("I2C read - send failed!!\n");
		return -1;
	}

	i4RetValue = i2c_master_recv(g_pstAF_I2Cclient, pBuff, 2);

	if (i4RetValue < 0) {
		LOG_INF("I2C read - recv failed!!\n");
		return -1;
	}
	*value = pBuff[0] << 8 | pBuff[1];

	return 0;
}
static int i2c_block_write_reg(kal_uint16 addr, kal_uint16 *para, kal_uint16 len)
{
	int i;
	for(i=0;i< len;i++){
		write_reg_16bit_value_16bit(addr+i,*para);
		para++;
	}
	return 0;
}

static int i2c_block_read_reg(kal_uint16 a_u2Addr, kal_uint16 *a_puBuff, kal_uint16 ui4_length )
{
	int i4RetValue = 0;
	char puReadCmd[2] = { (char)(a_u2Addr >> 8), (char)(a_u2Addr & 0xFF) };
	struct i2c_msg msg[2];
	int i =0,j=0;
	unsigned char temp_buff[ui4_length*2];

	//printk("i2c_block_read_reg add=0x%x",a_u2Addr);

	if (ui4_length > 256) {
		pr_debug("exceed one transition %d bytes limitation\n",
			 256);
		return -1;
	}
	spin_lock(&g_pOIS_SpinLock);
	g_pstAF_I2Cclient->addr = OIS_I2C_SLAVE_ADDR >> 1;
	g_pstAF_I2Cclient->addr =
		g_pstAF_I2Cclient->addr & (I2C_MASK_FLAG | I2C_WR_FLAG);
	spin_unlock(&g_pOIS_SpinLock);

	msg[0].addr = g_pstAF_I2Cclient->addr;
	msg[0].flags = g_pstAF_I2Cclient->flags & I2C_M_TEN;
	msg[0].len = 2;
	msg[0].buf = puReadCmd;

	msg[1].addr = g_pstAF_I2Cclient->addr;
	msg[1].flags = g_pstAF_I2Cclient->flags & I2C_M_TEN;
	msg[1].flags |= I2C_M_RD;
	msg[1].len = ui4_length*2;
	msg[1].buf = temp_buff;

	i4RetValue = i2c_transfer(g_pstAF_I2Cclient->adapter,
				msg,
				2);

	spin_lock(&g_pOIS_SpinLock);
	g_pstAF_I2Cclient->addr = g_pstAF_I2Cclient->addr & I2C_MASK_FLAG;
	spin_unlock(&g_pOIS_SpinLock);

	if (i4RetValue != 2) {
		pr_debug("I2C read data failed!!\n");
		return -1;
	}
	for(i=0;i<ui4_length;i++) {
		*a_puBuff= (temp_buff[j]<<8) + temp_buff[j+1];
		 a_puBuff++;
		 j=j+2;
	}

	return 0;
}

void set_OIS_Standby(void)
{
	LOG_INF("[dw9781c_standby]\n");
	return;
	/* release all protection */
	write_reg_16bit_value_16bit(0xFAFA, 0x98AC);
	mdelay(1);
	write_reg_16bit_value_16bit(0xF053, 0x70BD);
	mdelay(1);
}
void set_OIS_ENABLE(int enable)
{
	unsigned short ret=0;
	LOG_INF("[set_OIS_ENABLE] enable =%d \n",enable);
	if(enable == 0)
		write_reg_16bit_value_16bit(0x7015, 0x0000); /* Servo controller on,OIS on*/
	else if(enable == 1)
		write_reg_16bit_value_16bit(0x7015, 0x0001); /* Servo controller on,Ois off */
	else
		write_reg_16bit_value_16bit(0x7015, 0x0002); /* Servo controller off,OIS off*/
	mdelay(4);
	read_reg_16bit_value_16bit(0x7015,&ret);
	LOG_INF("[dw9781c] set_OIS_ENABLE reg_0x7015 value:0x%04x\r\n",  ret);
}
int set_OIS_Mode(int mode)
{
	int ret=0;
	LOG_INF("[set_OIS_MODE] mode =%d \n",mode);
	if(mode == 1)
		ret = write_reg_16bit_value_16bit(0x7029, 0x8000); /* disable Pantilt off for SR test */
	return ret;
}

void set_OIS_degbug(void)
{
	unsigned short ret=0;
	//int i =0;
	read_reg_16bit_value_16bit(0x7012,&ret);
	LOG_INF("[dw9781c] set_OIS_degbug reg_0x7012	gyro_type:0x%04x\r\n",	ret);

	read_reg_16bit_value_16bit(0x7000,&ret);
	LOG_INF("[dw9781c] set_OIS_degbug reg_0x7000	value:0x%04x\r\n",	ret);

	read_reg_16bit_value_16bit(0x7001,&ret);
	LOG_INF("[dw9781c] set_OIS_degbug reg_0x7001	value:0x%04x\r\n",	ret);

	read_reg_16bit_value_16bit(0x7002,&ret);
	LOG_INF("[dw9781c] set_OIS_degbug reg_0x7002	value:0x%04x\r\n",	ret);
	#if 0
	for(i==0; i< 200 ;i++) {
		read_reg_16bit_value_16bit(0x70f0,&ret);
		LOG_INF("[dw9781c] set_OIS_degbug reg_0x70f0	value:0x%04x\r\n",	ret);

		read_reg_16bit_value_16bit(0x70f1,&ret);
		LOG_INF("[dw9781c] set_OIS_degbug reg_0x70f1	value:0x%04x\r\n",	ret);

	}
	#endif

}

/*prize add by zhuzhengjiang end*/

void GenerateFirmwareContexts(void)
{
	g_firmwareContext.version = 0x0202;
	g_firmwareContext.size = 10240; /* size: word */
	g_firmwareContext.driverIc = 0x9781;
	g_firmwareContext.fwContentPtr = DW9781_Flash_Buf;
	g_downloadByForce = 0;
}

int dw9781c_download_ois_fw(void)
{
	unsigned char ret;
	unsigned short fwchecksum = 0;
	unsigned short first_chip_id = 0;
	unsigned short second_chip_id = 0;
	unsigned short fw_version_current = 0;
	unsigned short fw_version_latest = 0;
	unsigned short fw_type = 0;
	
	ois_ready_check();
	GenerateFirmwareContexts();
	read_reg_16bit_value_16bit(DW9781C_CHIP_ID_ADDRESS, &first_chip_id);
	printk("[dw9781c] first_chip_id : 0x%x\r\n", first_chip_id);
	if (first_chip_id != DW9781C_CHIP_ID) { /* first_chip_id verification failed */
		all_protection_release();
		read_reg_16bit_value_16bit(0xd060, &second_chip_id); /* second_chip_id: 0x0020 */
		if ( second_chip_id == 0x0020 ) /* Check the second_chip_id*/
		{
			printk("[dw9781c] start flash download:: size:%d, version:0x%x\r\n",
				g_firmwareContext.size, g_firmwareContext.version);
			ret = download_fw(); /* Need to forced update OIS firmware again. */
			printk("[dw9781c] flash download::vendor_dw9781c\r\n");
			if (ret != 0x0) {
				erase_mtp_rewritefw();
				write_reg_16bit_value_16bit(0xd000, 0x0000); /* Shut download mode */
				printk("[dw9781c] firmware download error, ret = 0x%x\r\n", ret);
				printk("[dw9781c] change dw9781c state to shutdown mode\r\n");
				return ERROR_FW_VERIFY;
			} else {
				printk("[dw9781c] firmware download success\r\n");
			}
		}else
		{
			write_reg_16bit_value_16bit(0xd000, 0x0000); /* Shut download mode */
			printk("[dw9781c] second_chip_id check fail\r\n");
			printk("[dw9781c] change dw9781c state to shutdown mode\r\n");
			return ERROR_SECOND_ID;
		}
	} else {
		fwchecksum = fw_checksum_verify();
		if(fwchecksum != *(g_firmwareContext.fwContentPtr+10234))
		{
			g_downloadByForce = 1;
			printk("[dw9781c] firmware checksum error 0x%04X, 0x%04X\r\n", *(g_firmwareContext.fwContentPtr+10234), fwchecksum);
		}
		
		read_reg_16bit_value_16bit(FW_TYPE_ADDR, &fw_type);
		if(fw_type != SET_FW)
		{
			g_downloadByForce = 1;
			printk("[dw9781c] Update to firmware for set\r\n");
		}
		read_reg_16bit_value_16bit(FW_VER_CURR_ADDR, &fw_version_current);
		fw_version_latest = g_firmwareContext.version; /*Enter the firmware version in VIVO*/
		printk("[dw9781c] fw_version_current = 0x%x, fw_version_latest = 0x%x\r\n",
			fw_version_current, fw_version_latest);
		//test ok
		/* download firmware, check if need update, download firmware to flash */
		if (g_downloadByForce || ((fw_version_current & 0xFF) != (fw_version_latest & 0xFF))) {
			printk("[dw9781c] start flash download:: size:%d, version:0x%x g_downloadByForce %d\r\n",
				g_firmwareContext.size, g_firmwareContext.version, g_downloadByForce);
			ret = download_fw();
			printk("[dw9781c] flash download::vendor_dw9781c\r\n");
			if (ret != EOK) {
				erase_mtp_rewritefw();
				write_reg_16bit_value_16bit(0xd000, 0x0000); /* Shut download mode */
				printk("[dw9781c] firmware download error, ret = 0x%x\r\n", ret);
				printk("[dw9781c] change dw9781c state to shutdown mode\r\n");
				return ERROR_FW_VERIFY;
			} else {
				printk("[dw9781c] firmware download success\r\n");
			}
		} else {
			printk("[dw9781c] ois firmware version is updated, skip download\r\n");
		}
	}
	ois_reset(); /* reset after download fw */
	/* If INI needs to be updated, use fw_update_item() */
	//fw_update_item();
	return ret;
}

unsigned short buf_temp[10240];
unsigned char buf_temp2[10240*2];

int download_fw(void)
{
	unsigned char ret = ERROR_FW_VERIFY;
	unsigned short i;
	unsigned short addr;
	//unsigned short buf[g_firmwareContext.size];
	memset(buf_temp, 0, g_firmwareContext.size * sizeof(unsigned short));
	/* step 1: MTP Erase and DSP Disable for firmware 0x8000 write */
	write_reg_16bit_value_16bit(0xd001, 0x0000);
	/* step 2: MTP setup */
	all_protection_release();
	erase_mtp();
	printk("[dw9781c_download_fw] start firmware download\r\n");
	/* step 3: firmware sequential write to flash */
	// prize dd test return ERROR_FW_VERIFY;
	for (i = 0; i < g_firmwareContext.size; i += DATPKT_SIZE)
	{
		addr = MTP_START_ADDRESS + i;
		i2c_block_write_reg( addr, g_firmwareContext.fwContentPtr + i, DATPKT_SIZE );
	}
	printk("[dw9781c_download_fw] write firmware to flash\r\n");
	/* step 4: firmware sequential read from flash */
	for (i = 0; i <  g_firmwareContext.size; i += DATPKT_SIZE)
	{
		addr = MTP_START_ADDRESS + i;
		i2c_block_read_reg(addr, buf_temp + i, DATPKT_SIZE);
	}
	printk("[dw9781c_download_fw] read firmware from flash\r\n");
	/* step 5: firmware verify */
	//return 0;
	/*
	for (i = 0; i < g_firmwareContext.size; i++){
		printk("0x%x, ",buf_temp[i]);
		if(i%9 == 0)
			printk("\n");
	}
	*/
	for (i = 0; i < g_firmwareContext.size; i++)
	{
		
		if (g_firmwareContext.fwContentPtr[i] != buf_temp[i])
		{
			printk("[dw9781c_download_fw] firmware verify NG!!! ADDR:%04X(0x%x 0x%x) -- firmware:%04x -- READ:%04x \r\n", MTP_START_ADDRESS+i,MTP_START_ADDRESS,i, g_firmwareContext.fwContentPtr[i], buf_temp[i]);
			return ERROR_FW_VERIFY;
		}else
			ret = EOK;
	}
	printk("[dw9781c_download_fw] firmware verification pass\r\n");
	printk("[dw9781c_download_fw] firmware download success\r\n");
	ois_reset();
	return ret;
}

void ois_ready_check(void)
{
	unsigned short fw_flag;
	write_reg_16bit_value_16bit(0xD001, 0x0000); /* dsp off mode */
	write_reg_16bit_value_16bit(0xFAFA, 0x98AC); /* All protection(1) */
	write_reg_16bit_value_16bit(0xF053, 0x70BD); /* All protection(2) */
	read_reg_16bit_value_16bit(0xA7F9, &fw_flag); /* check checksum flag */
	printk("[dw9781c_ois_ready_check] checksum flag : 0x%04x\r\n", fw_flag);
	if(fw_flag == 0xCC33)
	{
		printk("[dw9781c_ois_ready_check] checksum flag is ok\r\n");
		ois_reset(); /* ois reset */
	} else {
		write_reg_16bit_value_16bit(0xD002, 0x0001); /* dw9781c reset */
		mdelay(4);
		printk("[dw9781c_ois_ready_check] previous firmware download fail\r\n");
	}
}

void ois_reset(void)
{
	printk("[dw9781c_ois_reset] ois reset\r\n");
	write_reg_16bit_value_16bit(0xD002, 0x0001); /* printkc reset */
	mdelay(4);
	write_reg_16bit_value_16bit(0xD001, 0x0001); /* Active mode (DSP ON) */
	mdelay(25); /* ST gyro - over wait 25ms, default Servo On */
	write_reg_16bit_value_16bit(0xEBF1, 0x56FA); /* User protection release */
}

void all_protection_release(void)
{
	printk("[dw9781c_all_protection_release] execution\r\n");
	/* release all protection */
	write_reg_16bit_value_16bit(0xFAFA, 0x98AC);
	mdelay(1);
	write_reg_16bit_value_16bit(0xF053, 0x70BD);
	mdelay(1);
}

unsigned short fw_checksum_verify(void)
{
	unsigned short data;

	/* FW checksum command */
	write_reg_16bit_value_16bit(0x7011, 0x2000);
	/* command  start */
	write_reg_16bit_value_16bit(0x7010, 0x8000);
	mdelay(10);
	/* calc the checksum to write the 0x7005 */
	read_reg_16bit_value_16bit(0x7005, &data);
	printk("F/W Checksum calculated value : 0x%04X\r\n", data);

	return data;
}

int erase_mtp_rewritefw(void)
{
	printk("dw9781c erase for rewritefw starting..");
	
	/* 512 byte page */
	write_reg_16bit_value_16bit(0xde03, 0x0027);
	/* page erase */
	write_reg_16bit_value_16bit(0xde04, 0x0008);
	mdelay(10);

	printk("dw9781c checksum flag erase : 0xCC33\r\n");
	return 0;
}

void erase_mtp(void)
{
	printk("[dw9781c_erase_mtp] start erasing firmware flash\r\n");
	/* 12c level adjust */
	write_reg_16bit_value_16bit(0xd005, 0x0001);
	write_reg_16bit_value_16bit(0xdd03, 0x0002);
	write_reg_16bit_value_16bit(0xdd04, 0x0002);
	
	/* 4k Sector_0 */
	write_reg_16bit_value_16bit(0xde03, 0x0000);
	/* 4k Sector Erase */
	write_reg_16bit_value_16bit(0xde04, 0x0002);
	mdelay(10);
	/* 4k Sector_1 */
	write_reg_16bit_value_16bit(0xde03, 0x0008);
	/* 4k Sector Erase */
	write_reg_16bit_value_16bit(0xde04, 0x0002);
	mdelay(10);
	/* 4k Sector_2 */
	write_reg_16bit_value_16bit(0xde03, 0x0010);
	/* 4k Sector Erase */
	write_reg_16bit_value_16bit(0xde04, 0x0002);
	mdelay(10);
	/* 4k Sector_3 */
	write_reg_16bit_value_16bit(0xde03, 0x0018);
	/* 4k Sector Erase */
	write_reg_16bit_value_16bit(0xde04, 0x0002);
	mdelay(10);
	/* 4k Sector_4 */
	write_reg_16bit_value_16bit(0xde03, 0x0020);
	/* 4k Sector Erase */
	write_reg_16bit_value_16bit(0xde04, 0x0002);
	mdelay(10);
	printk("[dw9781c_erase_mtp] complete erasing firmware flash\r\n");
}

int gyro_offset_calibrtion(void)
{
	unsigned short Addr, status;
	unsigned short xOffset, yOffset;
	unsigned int OverCnt;
	int msg;
	int i = 1;
	//unsigned short  value =0;
	printk("[dw9781c_gyro_offset_calibrtion] gyro_offset_calibrtion starting\r\n");
	for(i = 1; i < 3; i++)
	{
		/* Gyro offset */
		write_reg_16bit_value_16bit(0x7011, 0x4015);
		write_reg_16bit_value_16bit(0x7010, 0x8000);

		//read_reg_16bit_value_16bit(0x70f0,&value);
		//printk("[dw9781c_gyro_offset_calibrtion] reg_0x70f0 value=0x%04X \r\n",value);
		//read_reg_16bit_value_16bit(0x70f4,&value);
		//printk("[dw9781c_gyro_offset_calibrtion] reg_0x70f4 value=0x%04X \r\n",value);

		mdelay(100);
		msg = 0;
		OverCnt = 0;
		while (1)
		{
			Addr = 0x7036; status = 0;
			read_reg_16bit_value_16bit(Addr, &status);
			if (status & 0x8000)
			{
				break;
			} /* it done! */
			else
			{
				mdelay(50); /* 100msec waiting */
			}
			if (OverCnt++ > GYRO_OFST_CAL_OVERCNT)
			{ /* when it take over 10sec .. break */
				msg = GYRO_CAL_TIME_OVER;
				break;
			}
		}
		if (msg == 0)
		{
			if (status & 0x8000)
			{
				if ( status == 0x8000 )
				{
					msg = GYRO_OFFSET_CAL_OK;
					printk("[dw9781c_gyro_offset_calibrtion] GYRO_OFFSET_CAL_OK\r\n");
					break;
				}
				else
				{
					if (status & 0x1)
					{
						msg += X_GYRO_OFFSET_SPEC_OVER_NG;
						printk("[dw9781c_gyro_offset_calibrtion] X_GYRO_OFFSET_SPEC_OVER_NG\r\n");
					}
					if (status & 0x2)
					{
						msg += Y_GYRO_OFFSET_SPEC_OVER_NG;
						printk("[dw9781c_gyro_offset_calibrtion] Y_GYRO_OFFSET_SPEC_OVER_NG\r\n");
					}
					if (status & 0x10)
					{
						msg += X_GYRO_RAW_DATA_CHECK;
						printk("[dw9781c_gyro_offset_calibrtion] X_GYRO_RAW_DATA_CHECK\r\n");
					}
					if (status & 0x20)
					{
						msg += Y_GYRO_RAW_DATA_CHECK;
						printk("[dw9781c_gyro_offset_calibrtion] Y_GYRO_RAW_DATA_CHECK\r\n");
					}
					if (i >= 2)
					{
						printk("[dw9781c_gyro_offset_calibrtion] gyro offset calibration-retry NG (%d times)\r\n", i);
					} else {
						printk("[dw9781c_gyro_offset_calibrtion] gyro offset calibration-retry NG (%d times)\r\n", i);
						msg = 0;
					}
				}
			}
		}
	}
	read_reg_16bit_value_16bit(0x70F8, &xOffset);
	read_reg_16bit_value_16bit(0x70F9, &yOffset);
	printk("[dw9781c_gyro_offset_calibrtion] msg : %d\r\n", msg);
	printk("[dw9781c_gyro_offset_calibrtion] x_gyro_offset: 0x%04X, y_gyro_offset : 0x%04X\r\n", xOffset, yOffset);
	printk("[dw9781c_gyro_offset_calibrtion] gyro_offset_calibrtion finished...Status = 0x%04X\r\n", status);

	if(msg == EOK)
		calibration_save();

	return msg;
}
#if 0
void fw_update_item(void)
{
	unsigned short r_reg = 0;
	unsigned char fw_projectinfo = 0;
	unsigned char fw_projectver = 0;
	unsigned char actuator_id = 0;
	unsigned char actuator_version = 0;
	
	unsigned short item_cnt;
	unsigned short update_item_addr[100];
	unsigned short update_item_data[100];
	int i;

	item_cnt = update_item[0];

	for (i = 0; i < item_cnt; i++)
	{
		update_item_addr[i] = update_item[i * 2 + 1];
		update_item_data[i] = update_item[i * 2 + 2];
	}
	
	printk("[dw9781c_fw_update_item] fw update item starting\r\n");
	write_reg_16bit_value_16bit(0x7015, 0x0002); // servo off
	read_reg_16bit_value_16bit(0x7001, &r_reg); /* Project Info */
	fw_projectinfo = (unsigned char) ((r_reg >> 8) & 0xFF);
	fw_projectver = (unsigned char) (r_reg & 0xFF);
	printk("[dw9781c_fw_update_item] project info : 0x%02X, version : 0x%02X\r\n", fw_projectinfo, fw_projectver);
	read_reg_16bit_value_16bit(0x700A, &r_reg); /* Set & modula maker info */
	printk("[dw9781c_fw_update_item] Set & modula maker info : 0x%04X\r\n", r_reg);
	read_reg_16bit_value_16bit(0x7004, &r_reg);
	actuator_id = (unsigned char) (r_reg & 0xFF); /* Actuator ID */
	actuator_version = (unsigned char) ((r_reg >> 8) & 0xFF); /* Actuator Version */
	printk("[dw9781c_fw_update_item] actuator_vendor : 0x%02X, actuator_version : 0x%02X\r\n", actuator_id, actuator_version);
	
	if( item_cnt > 0)
	{
		for (int i=0;i< item_cnt;i++) {
			write_reg_16bit_value_16bit(update_item_addr[i], update_item_data[i]);
			printk("addr: %04x, data: %04x\r\n", update_item_addr[i],update_item_data[i]);
		};
		printk("[dw9781c_fw_update_item] fw update item finish\r\n");
		calibration_save();
	} else
	{
		printk("[dw9781c_fw_update_item] fw update item has been skipped.\r\n");
	}
}
#endif
void calibration_save(void)
{
	printk("[dw9781c_calibration_save] calibration save starting\r\n");
	write_reg_16bit_value_16bit(0x7011, 0x00AA); /* select mode */
	mdelay(10);
	write_reg_16bit_value_16bit(0x7010, 0x8000); /* start mode */
	mdelay(100);
	ois_reset();
	printk("[dw9781c_calibration_save] calibration save finish\r\n");
}

void dw9781c_fw_read(void)
{	
	/* Read the data of fw memory using register */
	unsigned short buf_R[10240];
	int i = 0;
	printk("dw9781c_fw_read\r\n");

	write_reg_16bit_value_16bit(0xD001, 0x0000); /* dsp mode */
	write_reg_16bit_value_16bit(0xFAFA, 0x98AC); /* PTA0 Off */
	write_reg_16bit_value_16bit(0xF053, 0x70BD); /* PTA1 Off */
	mdelay(5);
	/* FW Register Read */
	for (i = 0; i < 10240; i++)
	{
		read_reg_16bit_value_16bit(0x2000+i, buf_R+i);
	}
	
	for (i = 0; i < 10240; i+= 0x10)
	{
		/* log for debug */
		printk("[dw9781c_fw_read] %04X = %04X %04X %04X %04X %04X %04X %04X %04X %04X %04X %04X %04X %04X %04X %04X %04X\r\n", 
		0x2000 + i, buf_R[i + 0], buf_R[i + 1], buf_R[i + 2],buf_R[i + 3],buf_R[i + 4],buf_R[i + 5], buf_R[i + 6], buf_R[i + 7], 
		buf_R[i + 8], buf_R[i + 9], buf_R[i + 10], buf_R[i + 11], buf_R[i + 12], buf_R[i + 13], buf_R[i + 14], buf_R[i + 15] ); 
	}
	ois_reset();
}

/* 2020. 06. 10 add function */
void flash_if_ram_read(void)
{
	/* Read the data of IF memory using RAM register */
	unsigned short buf_R[1024];
	int i = 0;
	memset(buf_R, 0, 1024 * sizeof(unsigned short));
	for (i = 0; i < 1024; i++)
	{
		read_reg_16bit_value_16bit(0x7000+i, buf_R+i);
	}
	printk("[flash_if_ram_read] IF_Memory Data Log\r\n");
	for (i = 0; i < 1024; i+= 0x10)
	{
		/* log for debug */
		printk("[flash_if_ram_read] %04X = %04X %04X %04X %04X %04X %04X %04X %04X %04X %04X %04X %04X %04X %04X %04X %04X\r\n", 
		0x7000 + i, buf_R[i + 0], buf_R[i + 1], buf_R[i + 2],buf_R[i + 3],buf_R[i + 4],buf_R[i + 5], buf_R[i + 6], buf_R[i + 7], 
		buf_R[i + 8], buf_R[i + 9], buf_R[i + 10], buf_R[i + 11], buf_R[i + 12], buf_R[i + 13], buf_R[i + 14], buf_R[i + 15] ); 
	}
}

/* 2020. 06. 11 add function */
void flash_ldt_register_read(void)
{
	/* Read the data of LDT memory using register */
	unsigned short buf_R[20];
	int i = 0;
	memset(buf_R, 0, 20 * sizeof(unsigned short));
	write_reg_16bit_value_16bit(0xD001, 0x0000); /* dsp mode */
	write_reg_16bit_value_16bit(0xFAFA, 0x98AC); /* PTA0 Off */
	write_reg_16bit_value_16bit(0xF053, 0x70BD); /* PTA1 Off */
	mdelay(5);
	/* LDT Register Read */
	for (i = 0; i < 20; i++)
	{
		read_reg_16bit_value_16bit(0xD060+i, (unsigned short*)buf_R+i);
		printk("LDT : buf_R[%04X] = %04X\r\n", 0xD060 + i, buf_R[i]); /* log for debug */
	}
	ois_reset();
}

void check_calibration_data(void)
{
	//int msg = 0;
	unsigned short tmp, tmp1;
	unsigned short XAmpGain = 0;
	unsigned short YAmpGain = 0;
	unsigned short XADCMax = 0;
	unsigned short XADCMin = 0;
	unsigned short YADCMax = 0;
	unsigned short YADCMin = 0;
	unsigned char X_HALL_BIAS = 0;
	unsigned char Y_HALL_BIAS = 0;
	int X_ADC_Range = 0;
	int Y_ADC_Range = 0;
	/* SAC_PARA] */
	printk("[flash_calibration_data] SAC_PARA\r\n");
	read_reg_16bit_value_16bit(0x701D, &tmp); printk("SAC_CFG: %04X\r\n", tmp);
	read_reg_16bit_value_16bit(0x701E, &tmp); printk("SAC_TVIB: %04X\r\n", tmp);
	read_reg_16bit_value_16bit(0x701F, &tmp); printk("SAC_CC: %04X\r\n", tmp);
	read_reg_16bit_value_16bit(0x7012, &tmp); printk("IMU_SENSOR_SEL: %04X\r\n", tmp);
	/* [CALIB_PARA] */
	printk("[flash_calibration_data] HALL DATA\r\n");
	read_reg_16bit_value_16bit(0x7003, &tmp); printk("MODULE_ID: %04X\r\n", tmp);
	read_reg_16bit_value_16bit(0x7004, &tmp); printk("ACTUATOR_ID: %04X\r\n", tmp);
	read_reg_16bit_value_16bit(0x707A, &tmp); printk("X_HALL_POL: %04X\r\n", tmp & 0x0001);
	read_reg_16bit_value_16bit(0x707A, &tmp); printk("Y_HALL_POL: %04X\r\n", tmp>>4 & 0x0001);
	read_reg_16bit_value_16bit(0x707B, &tmp); printk("X_DAC_POL: %04X\r\n", tmp & 0x0001);
	read_reg_16bit_value_16bit(0x707B, &tmp); printk("Y_DAC_POL: %04X\r\n", tmp>>4 & 0x0001);
	read_reg_16bit_value_16bit(0x707B, &tmp); printk("Z_DAC_POL: %04X\r\n", tmp>>8 & 0x0001);
	/* Hall Bias Range Check */
	read_reg_16bit_value_16bit(0x7070, &tmp);
	X_HALL_BIAS = tmp & 0x00FF;
	Y_HALL_BIAS = tmp >> 8;
	printk("Hall Bias X = 0x%02X\r\n",X_HALL_BIAS);
	printk("Hall Bias Y = 0x%02X\r\n",Y_HALL_BIAS);
	/* Hall ADC Range Check */
	read_reg_16bit_value_16bit(0x708C, &XADCMax); // X axis ADC MAX
	read_reg_16bit_value_16bit(0x708D, &XADCMin); // X axis ADC MIN
	read_reg_16bit_value_16bit(0x709E, &YADCMax); // Y axis ADC MAX
	read_reg_16bit_value_16bit(0x709F, &YADCMin); // Y axis ADC MIN
	X_ADC_Range = abs(XADCMax) + abs(XADCMin);
	Y_ADC_Range = abs(YADCMax) + abs(YADCMin);
	printk("X ADC Max = 0x%04X (%d)\r\n",XADCMax, XADCMax);
	printk("X ADC Min = 0x%04X (%d)\r\n",XADCMin, XADCMin);
	printk("Y ADC Max = 0x%04X (%d)\r\n",YADCMax, YADCMax);
	printk("Y ADC Max = 0x%04X (%d)\r\n",YADCMin, YADCMin);
	printk("X_ADC_Range = 0x%04X (%d)\r\n",X_ADC_Range, X_ADC_Range);
	printk("Y_ADC_Range = 0x%04X (%d)\r\n",Y_ADC_Range, Y_ADC_Range);
	/* Hall Amp gain Check */
	read_reg_16bit_value_16bit(0x7071, &XAmpGain);
	read_reg_16bit_value_16bit(0x7072, &YAmpGain);
	printk("Amp gain X = 0x%04X\r\n",XAmpGain);
	printk("Amp gain Y = 0x%04X\r\n",YAmpGain);
	/* [PID_PARA] */
	printk("[flash_calibration_data] PID_PARA\r\n");
	read_reg_16bit_value_16bit(0x7260, &tmp); printk("X_SERVO_P_GAIN: %04X\r\n", tmp);
	read_reg_16bit_value_16bit(0x7261, &tmp); printk("X_SERVO_I_GAIN: %04X\r\n", tmp);
	read_reg_16bit_value_16bit(0x7262, &tmp); printk("X_SERVO_D_GAIN: %04X\r\n", tmp);
	read_reg_16bit_value_16bit(0x7263, &tmp); printk("X_SERVO_D_LPF:  %04X\r\n", tmp);
	read_reg_16bit_value_16bit(0x7142, &tmp); read_reg_16bit_value_16bit(0x7143, &tmp1); printk("X_SERVO_BIQAUD_0 : %08X\r\n",tmp<<16 | tmp1);
	read_reg_16bit_value_16bit(0x7144, &tmp); read_reg_16bit_value_16bit(0x7145, &tmp1); printk("X_SERVO_BIQAUD_1 : %08X\r\n",tmp<<16 | tmp1);
	read_reg_16bit_value_16bit(0x7146, &tmp); read_reg_16bit_value_16bit(0x7147, &tmp1); printk("X_SERVO_BIQAUD_2 : %08X\r\n",tmp<<16 | tmp1);
	read_reg_16bit_value_16bit(0x7148, &tmp); read_reg_16bit_value_16bit(0x7149, &tmp1); printk("X_SERVO_BIQAUD_3 : %08X\r\n",tmp<<16 | tmp1);
	read_reg_16bit_value_16bit(0x714A, &tmp); read_reg_16bit_value_16bit(0x714B, &tmp1); printk("X_SERVO_BIQAUD_4 : %08X\r\n",tmp<<16 | tmp1);
	read_reg_16bit_value_16bit(0x714C, &tmp); read_reg_16bit_value_16bit(0x714D, &tmp1); printk("X_SERVO_BIQAUD_5 : %08X\r\n",tmp<<16 | tmp1);
	read_reg_16bit_value_16bit(0x714E, &tmp); read_reg_16bit_value_16bit(0x714F, &tmp1); printk("X_SERVO_BIQAUD_6 : %08X\r\n",tmp<<16 | tmp1);
	read_reg_16bit_value_16bit(0x7150, &tmp); read_reg_16bit_value_16bit(0x7151, &tmp1); printk("X_SERVO_BIQAUD_7 : %08X\r\n",tmp<<16 | tmp1);
	read_reg_16bit_value_16bit(0x7152, &tmp); read_reg_16bit_value_16bit(0x7153, &tmp1); printk("X_SERVO_BIQAUD_8 : %08X\r\n",tmp<<16 | tmp1);
	read_reg_16bit_value_16bit(0x7154, &tmp); read_reg_16bit_value_16bit(0x7155, &tmp1); printk("X_SERVO_BIQAUD_9 : %08X\r\n",tmp<<16 | tmp1);
	read_reg_16bit_value_16bit(0x7156, &tmp); read_reg_16bit_value_16bit(0x7157, &tmp1); printk("X_SERVO_BIQAUD_10: %08X\r\n",tmp<<16 | tmp1);
	read_reg_16bit_value_16bit(0x7158, &tmp); read_reg_16bit_value_16bit(0x7159, &tmp1); printk("X_SERVO_BIQAUD_11: %08X\r\n",tmp<<16 | tmp1);
	read_reg_16bit_value_16bit(0x715A, &tmp); read_reg_16bit_value_16bit(0x715B, &tmp1); printk("X_SERVO_BIQAUD_12: %08X\r\n",tmp<<16 | tmp1);
	read_reg_16bit_value_16bit(0x715C, &tmp); read_reg_16bit_value_16bit(0x715D, &tmp1); printk("X_SERVO_BIQAUD_13: %08X\r\n",tmp<<16 | tmp1);
	read_reg_16bit_value_16bit(0x715E, &tmp); read_reg_16bit_value_16bit(0x715F, &tmp1); printk("X_SERVO_BIQAUD_14: %08X\r\n",tmp<<16 | tmp1);
	read_reg_16bit_value_16bit(0x7160, &tmp); read_reg_16bit_value_16bit(0x7161, &tmp1); printk("X_SERVO_BIQAUD_15: %08X\r\n",tmp<<16 | tmp1);
	read_reg_16bit_value_16bit(0x7162, &tmp); read_reg_16bit_value_16bit(0x7163, &tmp1); printk("X_SERVO_BIQAUD_16: %08X\r\n",tmp<<16 | tmp1);
	read_reg_16bit_value_16bit(0x7164, &tmp); read_reg_16bit_value_16bit(0x7165, &tmp1); printk("X_SERVO_BIQAUD_17: %08X\r\n",tmp<<16 | tmp1);
	read_reg_16bit_value_16bit(0x7166, &tmp); read_reg_16bit_value_16bit(0x7167, &tmp1); printk("X_SERVO_BIQAUD_18: %08X\r\n",tmp<<16 | tmp1);
	read_reg_16bit_value_16bit(0x7168, &tmp); read_reg_16bit_value_16bit(0x7169, &tmp1); printk("X_SERVO_BIQAUD_19: %08X\r\n",tmp<<16 | tmp1);
	read_reg_16bit_value_16bit(0x7264, &tmp); printk("Y_SERVO_P_GAIN: %04X\r\n", tmp);
	read_reg_16bit_value_16bit(0x7265, &tmp); printk("Y_SERVO_I_GAIN: %04X\r\n", tmp);
	read_reg_16bit_value_16bit(0x7266, &tmp); printk("Y_SERVO_D_GAIN: %04X\r\n", tmp);
	read_reg_16bit_value_16bit(0x7267, &tmp); printk("Y_SERVO_D_LPF : %04X\r\n", tmp);
	read_reg_16bit_value_16bit(0x719C, &tmp); read_reg_16bit_value_16bit(0x719D, &tmp1); printk("Y_SERVO_BIQAUD_0 : %08X\r\n",tmp<<16 | tmp1);  
	read_reg_16bit_value_16bit(0x719E, &tmp); read_reg_16bit_value_16bit(0x719F, &tmp1); printk("Y_SERVO_BIQAUD_1 : %08X\r\n",tmp<<16 | tmp1);  
	read_reg_16bit_value_16bit(0x71A0, &tmp); read_reg_16bit_value_16bit(0x71A1, &tmp1); printk("Y_SERVO_BIQAUD_2 : %08X\r\n",tmp<<16 | tmp1);  
	read_reg_16bit_value_16bit(0x71A2, &tmp); read_reg_16bit_value_16bit(0x71A3, &tmp1); printk("Y_SERVO_BIQAUD_3 : %08X\r\n",tmp<<16 | tmp1);  
	read_reg_16bit_value_16bit(0x71A4, &tmp); read_reg_16bit_value_16bit(0x71A5, &tmp1); printk("Y_SERVO_BIQAUD_4 : %08X\r\n",tmp<<16 | tmp1);  
	read_reg_16bit_value_16bit(0x71A6, &tmp); read_reg_16bit_value_16bit(0x71A7, &tmp1); printk("Y_SERVO_BIQAUD_5 : %08X\r\n",tmp<<16 | tmp1);  
	read_reg_16bit_value_16bit(0x71A8, &tmp); read_reg_16bit_value_16bit(0x71A9, &tmp1); printk("Y_SERVO_BIQAUD_6 : %08X\r\n",tmp<<16 | tmp1);  
	read_reg_16bit_value_16bit(0x71AA, &tmp); read_reg_16bit_value_16bit(0x71AB, &tmp1); printk("Y_SERVO_BIQAUD_7 : %08X\r\n",tmp<<16 | tmp1);  
	read_reg_16bit_value_16bit(0x71AC, &tmp); read_reg_16bit_value_16bit(0x71AD, &tmp1); printk("Y_SERVO_BIQAUD_8 : %08X\r\n",tmp<<16 | tmp1);  
	read_reg_16bit_value_16bit(0x71AE, &tmp); read_reg_16bit_value_16bit(0x71AF, &tmp1); printk("Y_SERVO_BIQAUD_9 : %08X\r\n",tmp<<16 | tmp1);  
	read_reg_16bit_value_16bit(0x71B0, &tmp); read_reg_16bit_value_16bit(0x71B1, &tmp1); printk("Y_SERVO_BIQAUD_10: %08X\r\n",tmp<<16 | tmp1);
	read_reg_16bit_value_16bit(0x71B2, &tmp); read_reg_16bit_value_16bit(0x71B3, &tmp1); printk("Y_SERVO_BIQAUD_11: %08X\r\n",tmp<<16 | tmp1);
	read_reg_16bit_value_16bit(0x71B4, &tmp); read_reg_16bit_value_16bit(0x71B5, &tmp1); printk("Y_SERVO_BIQAUD_12: %08X\r\n",tmp<<16 | tmp1);
	read_reg_16bit_value_16bit(0x71B6, &tmp); read_reg_16bit_value_16bit(0x71B7, &tmp1); printk("Y_SERVO_BIQAUD_13: %08X\r\n",tmp<<16 | tmp1);
	read_reg_16bit_value_16bit(0x71B8, &tmp); read_reg_16bit_value_16bit(0x71B9, &tmp1); printk("Y_SERVO_BIQAUD_14: %08X\r\n",tmp<<16 | tmp1);
	read_reg_16bit_value_16bit(0x71BA, &tmp); read_reg_16bit_value_16bit(0x71BB, &tmp1); printk("Y_SERVO_BIQAUD_15: %08X\r\n",tmp<<16 | tmp1);
	read_reg_16bit_value_16bit(0x71BC, &tmp); read_reg_16bit_value_16bit(0x71BD, &tmp1); printk("Y_SERVO_BIQAUD_16: %08X\r\n",tmp<<16 | tmp1);
	read_reg_16bit_value_16bit(0x71BE, &tmp); read_reg_16bit_value_16bit(0x71BF, &tmp1); printk("Y_SERVO_BIQAUD_17: %08X\r\n",tmp<<16 | tmp1);
	read_reg_16bit_value_16bit(0x71C0, &tmp); read_reg_16bit_value_16bit(0x71C1, &tmp1); printk("Y_SERVO_BIQAUD_18: %08X\r\n",tmp<<16 | tmp1);
	read_reg_16bit_value_16bit(0x71C2, &tmp); read_reg_16bit_value_16bit(0x71C3, &tmp1); printk("Y_SERVO_BIQAUD_19: %08X\r\n",tmp<<16 | tmp1);
	//[GYRO_PARA]
	printk("[flash_calibration_data] GYRO_PARA\r\n");
	read_reg_16bit_value_16bit(0x7084, &tmp); printk("ACT_GYRO_MAT_0 : %04X\r\n", tmp);
	read_reg_16bit_value_16bit(0x7085, &tmp); printk("ACT_GYRO_MAT_1 : %04X\r\n", tmp);
	read_reg_16bit_value_16bit(0x7086, &tmp); printk("ACT_GYRO_MAT_2 : %04X\r\n", tmp);
	read_reg_16bit_value_16bit(0x7087, &tmp); printk("ACT_GYRO_MAT_3 : %04X\r\n", tmp);
	read_reg_16bit_value_16bit(0x71DA, &tmp); printk("X_GYRO_GAIN_POL: %04X\r\n", tmp);
	read_reg_16bit_value_16bit(0x71DB, &tmp); printk("Y_GYRO_GAIN_POL: %04X\r\n", tmp);
	read_reg_16bit_value_16bit(0x7138, &tmp); read_reg_16bit_value_16bit(0x7139, &tmp1); printk("X_GYRO_LPF_0: %08X\r\n", tmp<<16 | tmp1);
	read_reg_16bit_value_16bit(0x713A, &tmp); read_reg_16bit_value_16bit(0x713B, &tmp1); printk("X_GYRO_LPF_1: %08X\r\n", tmp<<16 | tmp1);
	read_reg_16bit_value_16bit(0x713C, &tmp); read_reg_16bit_value_16bit(0x713D, &tmp1); printk("X_GYRO_LPF_2: %08X\r\n", tmp<<16 | tmp1);
	read_reg_16bit_value_16bit(0x713E, &tmp); read_reg_16bit_value_16bit(0x713F, &tmp1); printk("X_GYRO_LPF_3: %08X\r\n", tmp<<16 | tmp1);
	read_reg_16bit_value_16bit(0x7140, &tmp); read_reg_16bit_value_16bit(0x7141, &tmp1); printk("X_GYRO_LPF_4: %08X\r\n", tmp<<16 | tmp1);
	read_reg_16bit_value_16bit(0x7110, &tmp); read_reg_16bit_value_16bit(0x7111, &tmp1); printk("X_GYRO_BIQAUD_0 : %08X\r\n", tmp<<16 | tmp1);
	read_reg_16bit_value_16bit(0x7112, &tmp); read_reg_16bit_value_16bit(0x7113, &tmp1); printk("X_GYRO_BIQAUD_1 : %08X\r\n", tmp<<16 | tmp1);
	read_reg_16bit_value_16bit(0x7114, &tmp); read_reg_16bit_value_16bit(0x7115, &tmp1); printk("X_GYRO_BIQAUD_2 : %08X\r\n", tmp<<16 | tmp1);
	read_reg_16bit_value_16bit(0x7116, &tmp); read_reg_16bit_value_16bit(0x7117, &tmp1); printk("X_GYRO_BIQAUD_3 : %08X\r\n", tmp<<16 | tmp1);
	read_reg_16bit_value_16bit(0x7118, &tmp); read_reg_16bit_value_16bit(0x7119, &tmp1); printk("X_GYRO_BIQAUD_4 : %08X\r\n", tmp<<16 | tmp1);
	read_reg_16bit_value_16bit(0x711A, &tmp); read_reg_16bit_value_16bit(0x711B, &tmp1); printk("X_GYRO_BIQAUD_5 : %08X\r\n", tmp<<16 | tmp1);
	read_reg_16bit_value_16bit(0x711C, &tmp); read_reg_16bit_value_16bit(0x711D, &tmp1); printk("X_GYRO_BIQAUD_6 : %08X\r\n", tmp<<16 | tmp1);
	read_reg_16bit_value_16bit(0x711E, &tmp); read_reg_16bit_value_16bit(0x711F, &tmp1); printk("X_GYRO_BIQAUD_7 : %08X\r\n", tmp<<16 | tmp1);
	read_reg_16bit_value_16bit(0x7120, &tmp); read_reg_16bit_value_16bit(0x7121, &tmp1); printk("X_GYRO_BIQAUD_8 : %08X\r\n", tmp<<16 | tmp1);
	read_reg_16bit_value_16bit(0x7122, &tmp); read_reg_16bit_value_16bit(0x7123, &tmp1); printk("X_GYRO_BIQAUD_9 : %08X\r\n", tmp<<16 | tmp1);
	read_reg_16bit_value_16bit(0x7124, &tmp); read_reg_16bit_value_16bit(0x7125, &tmp1); printk("X_GYRO_BIQAUD_10: %08X\r\n", tmp<<16 | tmp1);
	read_reg_16bit_value_16bit(0x7126, &tmp); read_reg_16bit_value_16bit(0x7127, &tmp1); printk("X_GYRO_BIQAUD_11: %08X\r\n", tmp<<16 | tmp1);
	read_reg_16bit_value_16bit(0x7128, &tmp); read_reg_16bit_value_16bit(0x7129, &tmp1); printk("X_GYRO_BIQAUD_12: %08X\r\n", tmp<<16 | tmp1);
	read_reg_16bit_value_16bit(0x712A, &tmp); read_reg_16bit_value_16bit(0x712B, &tmp1); printk("X_GYRO_BIQAUD_13: %08X\r\n", tmp<<16 | tmp1);
	read_reg_16bit_value_16bit(0x712C, &tmp); read_reg_16bit_value_16bit(0x712D, &tmp1); printk("X_GYRO_BIQAUD_14: %08X\r\n", tmp<<16 | tmp1);
	read_reg_16bit_value_16bit(0x712E, &tmp); read_reg_16bit_value_16bit(0x712F, &tmp1); printk("X_GYRO_BIQAUD_15: %08X\r\n", tmp<<16 | tmp1);
	read_reg_16bit_value_16bit(0x7130, &tmp); read_reg_16bit_value_16bit(0x7131, &tmp1); printk("X_GYRO_BIQAUD_16: %08X\r\n", tmp<<16 | tmp1);
	read_reg_16bit_value_16bit(0x7132, &tmp); read_reg_16bit_value_16bit(0x7133, &tmp1); printk("X_GYRO_BIQAUD_17: %08X\r\n", tmp<<16 | tmp1);
	read_reg_16bit_value_16bit(0x7134, &tmp); read_reg_16bit_value_16bit(0x7135, &tmp1); printk("X_GYRO_BIQAUD_18: %08X\r\n", tmp<<16 | tmp1);
	read_reg_16bit_value_16bit(0x7136, &tmp); read_reg_16bit_value_16bit(0x7137, &tmp1); printk("X_GYRO_BIQAUD_19: %08X\r\n", tmp<<16 | tmp1);
	read_reg_16bit_value_16bit(0x7192, &tmp); read_reg_16bit_value_16bit(0x7193, &tmp1); printk("Y_GYRO_LPF_0: %08X\r\n", tmp<<16 | tmp1);
	read_reg_16bit_value_16bit(0x7194, &tmp); read_reg_16bit_value_16bit(0x7195, &tmp1); printk("Y_GYRO_LPF_1: %08X\r\n", tmp<<16 | tmp1);
	read_reg_16bit_value_16bit(0x7196, &tmp); read_reg_16bit_value_16bit(0x7197, &tmp1); printk("Y_GYRO_LPF_2: %08X\r\n", tmp<<16 | tmp1);
	read_reg_16bit_value_16bit(0x7198, &tmp); read_reg_16bit_value_16bit(0x7199, &tmp1); printk("Y_GYRO_LPF_3: %08X\r\n", tmp<<16 | tmp1);
	read_reg_16bit_value_16bit(0x719A, &tmp); read_reg_16bit_value_16bit(0x719B, &tmp1); printk("Y_GYRO_LPF_4: %08X\r\n", tmp<<16 | tmp1);
	read_reg_16bit_value_16bit(0x716A, &tmp); read_reg_16bit_value_16bit(0x716B, &tmp1); printk("Y_GYRO_BIQAUD_0 : %08X\r\n", tmp<<16 | tmp1);
	read_reg_16bit_value_16bit(0x716C, &tmp); read_reg_16bit_value_16bit(0x716D, &tmp1); printk("Y_GYRO_BIQAUD_1 : %08X\r\n", tmp<<16 | tmp1);
	read_reg_16bit_value_16bit(0x716E, &tmp); read_reg_16bit_value_16bit(0x716F, &tmp1); printk("Y_GYRO_BIQAUD_2 : %08X\r\n", tmp<<16 | tmp1);
	read_reg_16bit_value_16bit(0x7170, &tmp); read_reg_16bit_value_16bit(0x7171, &tmp1); printk("Y_GYRO_BIQAUD_3 : %08X\r\n", tmp<<16 | tmp1);
	read_reg_16bit_value_16bit(0x7172, &tmp); read_reg_16bit_value_16bit(0x7173, &tmp1); printk("Y_GYRO_BIQAUD_4 : %08X\r\n", tmp<<16 | tmp1);
	read_reg_16bit_value_16bit(0x7174, &tmp); read_reg_16bit_value_16bit(0x7175, &tmp1); printk("Y_GYRO_BIQAUD_5 : %08X\r\n", tmp<<16 | tmp1);
	read_reg_16bit_value_16bit(0x7176, &tmp); read_reg_16bit_value_16bit(0x7177, &tmp1); printk("Y_GYRO_BIQAUD_6 : %08X\r\n", tmp<<16 | tmp1);
	read_reg_16bit_value_16bit(0x7178, &tmp); read_reg_16bit_value_16bit(0x7179, &tmp1); printk("Y_GYRO_BIQAUD_7 : %08X\r\n", tmp<<16 | tmp1);
	read_reg_16bit_value_16bit(0x717A, &tmp); read_reg_16bit_value_16bit(0x717B, &tmp1); printk("Y_GYRO_BIQAUD_8 : %08X\r\n", tmp<<16 | tmp1);
	read_reg_16bit_value_16bit(0x717C, &tmp); read_reg_16bit_value_16bit(0x717D, &tmp1); printk("Y_GYRO_BIQAUD_9 : %08X\r\n", tmp<<16 | tmp1);
	read_reg_16bit_value_16bit(0x717E, &tmp); read_reg_16bit_value_16bit(0x717F, &tmp1); printk("Y_GYRO_BIQAUD_10: %08X\r\n", tmp<<16 | tmp1);
	read_reg_16bit_value_16bit(0x7180, &tmp); read_reg_16bit_value_16bit(0x7181, &tmp1); printk("Y_GYRO_BIQAUD_11: %08X\r\n", tmp<<16 | tmp1);
	read_reg_16bit_value_16bit(0x7182, &tmp); read_reg_16bit_value_16bit(0x7183, &tmp1); printk("Y_GYRO_BIQAUD_12: %08X\r\n", tmp<<16 | tmp1);
	read_reg_16bit_value_16bit(0x7184, &tmp); read_reg_16bit_value_16bit(0x7185, &tmp1); printk("Y_GYRO_BIQAUD_13: %08X\r\n", tmp<<16 | tmp1);
	read_reg_16bit_value_16bit(0x7186, &tmp); read_reg_16bit_value_16bit(0x7187, &tmp1); printk("Y_GYRO_BIQAUD_14: %08X\r\n", tmp<<16 | tmp1);
	read_reg_16bit_value_16bit(0x7188, &tmp); read_reg_16bit_value_16bit(0x7189, &tmp1); printk("Y_GYRO_BIQAUD_15: %08X\r\n", tmp<<16 | tmp1);
	read_reg_16bit_value_16bit(0x718A, &tmp); read_reg_16bit_value_16bit(0x718B, &tmp1); printk("Y_GYRO_BIQAUD_16: %08X\r\n", tmp<<16 | tmp1);
	read_reg_16bit_value_16bit(0x718C, &tmp); read_reg_16bit_value_16bit(0x718D, &tmp1); printk("Y_GYRO_BIQAUD_17: %08X\r\n", tmp<<16 | tmp1);
	read_reg_16bit_value_16bit(0x718E, &tmp); read_reg_16bit_value_16bit(0x718F, &tmp1); printk("Y_GYRO_BIQAUD_18: %08X\r\n", tmp<<16 | tmp1); 
	read_reg_16bit_value_16bit(0x7190, &tmp); read_reg_16bit_value_16bit(0x7191, &tmp1); printk("Y_GYRO_BIQAUD_19: %08X\r\n", tmp<<16 | tmp1);
	//[ACC_PARA]
	printk("[flash_calibration_data] ACC_PARA\r\n");
	read_reg_16bit_value_16bit(0x70E8, &tmp); printk("ACT_ACC_MAT_0 : %04X\r\n", tmp);
	read_reg_16bit_value_16bit(0x70E9, &tmp); printk("ACT_ACC_MAT_1 : %04X\r\n", tmp);
	read_reg_16bit_value_16bit(0x70EA, &tmp); printk("ACT_ACC_MAT_2 : %04X\r\n", tmp);
	read_reg_16bit_value_16bit(0x70EB, &tmp); printk("ACT_ACC_MAT_3 : %04X\r\n", tmp);
	read_reg_16bit_value_16bit(0x70E6, &tmp); printk("X_ACC_GAIN_POL: %04X\r\n", tmp);
	read_reg_16bit_value_16bit(0x70E7, &tmp); printk("Y_ACC_GAIN_POL: %04X\r\n", tmp);
	read_reg_16bit_value_16bit(0x72B0, &tmp); read_reg_16bit_value_16bit(0x72B1, &tmp1); printk("X_ACC_BIQUAD_0 : %08X\r\n", tmp<<16 | tmp1);
	read_reg_16bit_value_16bit(0x72B2, &tmp); read_reg_16bit_value_16bit(0x72B3, &tmp1); printk("X_ACC_BIQUAD_1 : %08X\r\n", tmp<<16 | tmp1);
	read_reg_16bit_value_16bit(0x72B4, &tmp); read_reg_16bit_value_16bit(0x72B5, &tmp1); printk("X_ACC_BIQUAD_2 : %08X\r\n", tmp<<16 | tmp1);
	read_reg_16bit_value_16bit(0x72B6, &tmp); read_reg_16bit_value_16bit(0x72B7, &tmp1); printk("X_ACC_BIQUAD_3 : %08X\r\n", tmp<<16 | tmp1);
	read_reg_16bit_value_16bit(0x72B8, &tmp); read_reg_16bit_value_16bit(0x72B9, &tmp1); printk("X_ACC_BIQUAD_4 : %08X\r\n", tmp<<16 | tmp1);
	read_reg_16bit_value_16bit(0x72BA, &tmp); read_reg_16bit_value_16bit(0x72BB, &tmp1); printk("X_ACC_BIQUAD_5 : %08X\r\n", tmp<<16 | tmp1);
	read_reg_16bit_value_16bit(0x72BC, &tmp); read_reg_16bit_value_16bit(0x72BD, &tmp1); printk("X_ACC_BIQUAD_6 : %08X\r\n", tmp<<16 | tmp1);
	read_reg_16bit_value_16bit(0x72BE, &tmp); read_reg_16bit_value_16bit(0x72BF, &tmp1); printk("X_ACC_BIQUAD_7 : %08X\r\n", tmp<<16 | tmp1);
	read_reg_16bit_value_16bit(0x72C0, &tmp); read_reg_16bit_value_16bit(0x72C1, &tmp1); printk("X_ACC_BIQUAD_8 : %08X\r\n", tmp<<16 | tmp1);
	read_reg_16bit_value_16bit(0x72C2, &tmp); read_reg_16bit_value_16bit(0x72C3, &tmp1); printk("X_ACC_BIQUAD_9 : %08X\r\n", tmp<<16 | tmp1);
	read_reg_16bit_value_16bit(0x72C4, &tmp); read_reg_16bit_value_16bit(0x72C5, &tmp1); printk("X_ACC_BIQUAD_10: %08X\r\n", tmp<<16 | tmp1);
	read_reg_16bit_value_16bit(0x72C6, &tmp); read_reg_16bit_value_16bit(0x72C7, &tmp1); printk("X_ACC_BIQUAD_11: %08X\r\n", tmp<<16 | tmp1);
	read_reg_16bit_value_16bit(0x72C8, &tmp); read_reg_16bit_value_16bit(0x72C9, &tmp1); printk("X_ACC_BIQUAD_12: %08X\r\n", tmp<<16 | tmp1);
	read_reg_16bit_value_16bit(0x72CA, &tmp); read_reg_16bit_value_16bit(0x72CB, &tmp1); printk("X_ACC_BIQUAD_13: %08X\r\n", tmp<<16 | tmp1);
	read_reg_16bit_value_16bit(0x72CC, &tmp); read_reg_16bit_value_16bit(0x72CD, &tmp1); printk("X_ACC_BIQUAD_14: %08X\r\n", tmp<<16 | tmp1);
	read_reg_16bit_value_16bit(0x72CE, &tmp); read_reg_16bit_value_16bit(0x72CF, &tmp1); printk("X_ACC_BIQUAD_15: %08X\r\n", tmp<<16 | tmp1);
	read_reg_16bit_value_16bit(0x72D0, &tmp); read_reg_16bit_value_16bit(0x72D1, &tmp1); printk("X_ACC_BIQUAD_16: %08X\r\n", tmp<<16 | tmp1);
	read_reg_16bit_value_16bit(0x72D2, &tmp); read_reg_16bit_value_16bit(0x72D3, &tmp1); printk("X_ACC_BIQUAD_17: %08X\r\n", tmp<<16 | tmp1);
	read_reg_16bit_value_16bit(0x72D4, &tmp); read_reg_16bit_value_16bit(0x72D5, &tmp1); printk("X_ACC_BIQUAD_18: %08X\r\n", tmp<<16 | tmp1);
	read_reg_16bit_value_16bit(0x72D6, &tmp); read_reg_16bit_value_16bit(0x72D7, &tmp1); printk("X_ACC_BIQUAD_19: %08X\r\n", tmp<<16 | tmp1);
	read_reg_16bit_value_16bit(0x72D8, &tmp); read_reg_16bit_value_16bit(0x72D9, &tmp1); printk("X_ACC_BIQUAD_20: %08X\r\n", tmp<<16 | tmp1);
	read_reg_16bit_value_16bit(0x72DA, &tmp); read_reg_16bit_value_16bit(0x72DB, &tmp1); printk("X_ACC_BIQUAD_21: %08X\r\n", tmp<<16 | tmp1);
	read_reg_16bit_value_16bit(0x72DC, &tmp); read_reg_16bit_value_16bit(0x72DD, &tmp1); printk("X_ACC_BIQUAD_22: %08X\r\n", tmp<<16 | tmp1);
	read_reg_16bit_value_16bit(0x72DE, &tmp); read_reg_16bit_value_16bit(0x72DF, &tmp1); printk("X_ACC_BIQUAD_23: %08X\r\n", tmp<<16 | tmp1);
	read_reg_16bit_value_16bit(0x72E0, &tmp); read_reg_16bit_value_16bit(0x72E1, &tmp1); printk("X_ACC_BIQUAD_24: %08X\r\n", tmp<<16 | tmp1);
	read_reg_16bit_value_16bit(0x72E2, &tmp); read_reg_16bit_value_16bit(0x72E3, &tmp1); printk("Y_ACC_BIQUAD_0 : %08X\r\n", tmp<<16 | tmp1);
	read_reg_16bit_value_16bit(0x72E4, &tmp); read_reg_16bit_value_16bit(0x72E5, &tmp1); printk("Y_ACC_BIQUAD_1 : %08X\r\n", tmp<<16 | tmp1);
	read_reg_16bit_value_16bit(0x72E6, &tmp); read_reg_16bit_value_16bit(0x72E7, &tmp1); printk("Y_ACC_BIQUAD_2 : %08X\r\n", tmp<<16 | tmp1);
	read_reg_16bit_value_16bit(0x72E8, &tmp); read_reg_16bit_value_16bit(0x72E9, &tmp1); printk("Y_ACC_BIQUAD_3 : %08X\r\n", tmp<<16 | tmp1);
	read_reg_16bit_value_16bit(0x72EA, &tmp); read_reg_16bit_value_16bit(0x72EB, &tmp1); printk("Y_ACC_BIQUAD_4 : %08X\r\n", tmp<<16 | tmp1);
	read_reg_16bit_value_16bit(0x72EC, &tmp); read_reg_16bit_value_16bit(0x72ED, &tmp1); printk("Y_ACC_BIQUAD_5 : %08X\r\n", tmp<<16 | tmp1);
	read_reg_16bit_value_16bit(0x72EE, &tmp); read_reg_16bit_value_16bit(0x72EF, &tmp1); printk("Y_ACC_BIQUAD_6 : %08X\r\n", tmp<<16 | tmp1);
	read_reg_16bit_value_16bit(0x72F0, &tmp); read_reg_16bit_value_16bit(0x72F1, &tmp1); printk("Y_ACC_BIQUAD_7 : %08X\r\n", tmp<<16 | tmp1);
	read_reg_16bit_value_16bit(0x72F2, &tmp); read_reg_16bit_value_16bit(0x72F3, &tmp1); printk("Y_ACC_BIQUAD_8 : %08X\r\n", tmp<<16 | tmp1);
	read_reg_16bit_value_16bit(0x72F4, &tmp); read_reg_16bit_value_16bit(0x72F5, &tmp1); printk("Y_ACC_BIQUAD_9 : %08X\r\n", tmp<<16 | tmp1);
	read_reg_16bit_value_16bit(0x72F6, &tmp); read_reg_16bit_value_16bit(0x72F7, &tmp1); printk("Y_ACC_BIQUAD_10: %08X\r\n", tmp<<16 | tmp1);
	read_reg_16bit_value_16bit(0x72F8, &tmp); read_reg_16bit_value_16bit(0x72F9, &tmp1); printk("Y_ACC_BIQUAD_11: %08X\r\n", tmp<<16 | tmp1);
	read_reg_16bit_value_16bit(0x72FA, &tmp); read_reg_16bit_value_16bit(0x72FB, &tmp1); printk("Y_ACC_BIQUAD_12: %08X\r\n", tmp<<16 | tmp1);
	read_reg_16bit_value_16bit(0x72FC, &tmp); read_reg_16bit_value_16bit(0x72FD, &tmp1); printk("Y_ACC_BIQUAD_13: %08X\r\n", tmp<<16 | tmp1);
	read_reg_16bit_value_16bit(0x72FE, &tmp); read_reg_16bit_value_16bit(0x72FF, &tmp1); printk("Y_ACC_BIQUAD_14: %08X\r\n", tmp<<16 | tmp1);
	read_reg_16bit_value_16bit(0x7300, &tmp); read_reg_16bit_value_16bit(0x7301, &tmp1); printk("Y_ACC_BIQUAD_15: %08X\r\n", tmp<<16 | tmp1);
	read_reg_16bit_value_16bit(0x7302, &tmp); read_reg_16bit_value_16bit(0x7303, &tmp1); printk("Y_ACC_BIQUAD_16: %08X\r\n", tmp<<16 | tmp1);
	read_reg_16bit_value_16bit(0x7304, &tmp); read_reg_16bit_value_16bit(0x7305, &tmp1); printk("Y_ACC_BIQUAD_17: %08X\r\n", tmp<<16 | tmp1);
	read_reg_16bit_value_16bit(0x7306, &tmp); read_reg_16bit_value_16bit(0x7307, &tmp1); printk("Y_ACC_BIQUAD_18: %08X\r\n", tmp<<16 | tmp1);
	read_reg_16bit_value_16bit(0x7308, &tmp); read_reg_16bit_value_16bit(0x7309, &tmp1); printk("Y_ACC_BIQUAD_19: %08X\r\n", tmp<<16 | tmp1);
	read_reg_16bit_value_16bit(0x730A, &tmp); read_reg_16bit_value_16bit(0x730B, &tmp1); printk("Y_ACC_BIQUAD_20: %08X\r\n", tmp<<16 | tmp1);
	read_reg_16bit_value_16bit(0x730C, &tmp); read_reg_16bit_value_16bit(0x730D, &tmp1); printk("Y_ACC_BIQUAD_21: %08X\r\n", tmp<<16 | tmp1);
	read_reg_16bit_value_16bit(0x730E, &tmp); read_reg_16bit_value_16bit(0x730F, &tmp1); printk("Y_ACC_BIQUAD_22: %08X\r\n", tmp<<16 | tmp1);
	read_reg_16bit_value_16bit(0x7310, &tmp); read_reg_16bit_value_16bit(0x7311, &tmp1); printk("Y_ACC_BIQUAD_23: %08X\r\n", tmp<<16 | tmp1);
	read_reg_16bit_value_16bit(0x7312, &tmp); read_reg_16bit_value_16bit(0x7313, &tmp1); printk("Y_ACC_BIQUAD_24: %08X\r\n", tmp<<16 | tmp1);
}
