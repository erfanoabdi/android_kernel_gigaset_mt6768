#define kal_uint16 unsigned short
void GenerateFirmwareContexts(void);
int dw9781c_download_ois_fw(void);
int download_fw(void);
void ois_ready_check(void);
void ois_reset(void);
void all_protection_release(void);
unsigned short fw_checksum_verify(void);
int erase_mtp_rewritefw(void);
void erase_mtp(void);
int gyro_offset_calibrtion(void);
void fw_update_item(void);
void calibration_save(void);
void dw9781c_fw_read(void);
void flash_if_ram_read(void);
void flash_ldt_register_read(void);
void check_calibration_data(void);
// prize add by zhuzhengjiang start
void set_OIS_ENABLE(int enable);
void set_OIS_Standby(void);
void set_OIS_degbug(void);
int set_OIS_Mode(int mode);
// prize add by zhuzhengjiang end

/* fw_update_item */
#define UPDATE_ITEM_LENGTH 3

/* project informateion */
#define P4AXIS 0x00
#define PX5ZOOM 0x01

/* fw downlaod */
#define OIS_I2C_SLAVE_ADDR 0xE4 // prize add by zhuzhengjiang

#define DW9781C_CHIP_ID_ADDRESS 0x7000
#define DW9781C_CHIP_ID 0x9781
#define FW_VER_CURR_ADDR 0x7001
#define FW_TYPE_ADDR 0x700D
#define SET_FW 0x8001
#define EOK 0
#define ERROR_SECOND_ID -1
#define ERROR_FW_VERIFY -2
#define MTP_START_ADDRESS 0x8000
#define MAX_FIRMWARE_NUMBER 5
#define DATPKT_SIZE 256

/* gyro offset calibration */
#define GYRO_OFFSET_CAL_OK 0x00
#define GYRO_CAL_TIME_OVER 0xFF
#define X_GYRO_OFFSET_SPEC_OVER_NG 0x0001
#define X_GYRO_RAW_DATA_CHECK 0x0010
#define Y_GYRO_OFFSET_SPEC_OVER_NG 0x0002
#define Y_GYRO_RAW_DATA_CHECK 0x0020
#define GYRO_OFST_CAL_OVERCNT 10
