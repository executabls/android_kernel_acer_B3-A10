#ifndef _LINUX_ELAN_TS_H
#define _LINUX_ELAN_TS_H

/****************************customer info****************************/
#define LCM_X_MAX 1280
#define LCM_Y_MAX 800

/****************************elan data info****************************/
//i2c info
#define ELAN_TS_NAME "mtk-tpd"

#define ELAN_7BITS_ADDR 0x10
#define ELAN_8BITS_ADDR (ELAN_7BITS_ADDR<<1)
#define GESTURE_WAKEUP

//sleep  mod 	
#define PWR_STATE_DEEP_SLEEP	0
#define PWR_STATE_NORMAL		1
#define PWR_STATE_MASK		BIT(3)

//cmd or paket head
#define CMD_S_PKT			0x52
#define CMD_R_PKT			0x53
#define CMD_W_PKT			0x54
#define HELLO_PKT			0x55
#define RAM_PKT				0xcc
#define BUFF_PKT			0x63

//elan IC series(only choose one)
//#define ELAN_2K_XX
#define ELAN_3K_XX
//#define ELAN_RAM_XX

/**********************fingers number macro switch**********************/
//#define TWO_FINGERS
//#define FIVE_FINGERS
#define TEN_FINGERS

#define RECV_MORE_THAN_EIGHT_BYTES

#ifdef TWO_FINGERS
	#define FINGERS_PKT				0x5A
	#define PACKET_SIZE				8
	#define FINGERS_NUM				2
#endif

#ifdef FIVE_FINGERS
	#define FINGERS_PKT				0x5D
	#define PACKET_SIZE				18
	#define FINGERS_NUM				5
#endif

#ifdef TEN_FINGERS
	#define FINGERS_PKT				0x62
	#ifdef ELAN_BUFFER_MODE
	#define PACKET_SIZE				55
	#else
	#define PACKET_SIZE				55
	#endif
	#define FINGERS_NUM				10
#endif


/***********************debug info macro switch***********************/
//#define PRINT_INT_INFO 

#ifdef PRINT_INT_INFO 
	bool debug_flage = true;
	#define elan_info(fmt, args...) do{\
		if(debug_flage)\
                printk("[elan debug]:"fmt"\n", ##args);\
	}while(0);
#else
	#define elan_info(fmt, args...)
#endif


/*************************have button macro switch*********************/
#define TPD_HAVE_BUTTON
#ifdef TPD_HAVE_BUTTON
#define TPD_BUTTON_HEIGH		100
#define TPD_BUTTON_WIDE		    30
#define TPD_BUTTON_MAX		LCM_Y_MAX+TPD_BUTTON_HEIGH+10
#define TPD_KEY_COUNT		3
#define TPD_KEYS				{ KEY_MENU, /*KEY_HOME*/KEY_HOMEPAGE, KEY_BACK}
#define TPD_KEYS_DIM			{	{100,TPD_BUTTON_MAX,TPD_BUTTON_WIDE,TPD_BUTTON_HEIGH},\
								{200,TPD_BUTTON_MAX,TPD_BUTTON_WIDE,TPD_BUTTON_HEIGH},\
								{300,TPD_BUTTON_MAX,TPD_BUTTON_WIDE,TPD_BUTTON_HEIGH}\
							}

static int tpd_keys_local[TPD_KEY_COUNT] = TPD_KEYS;
static int tpd_keys_dim_local[TPD_KEY_COUNT][4] = TPD_KEYS_DIM;
#endif

#define ELAN_KEY_BACK 0x81
#define ELAN_KEY_HOME 0x41
#define ELAN_KEY_MENU 0x21

#define HELLO_PKT		0x55
#define TWO_FINGERS_PKT		0x5A
#define FIVE_FINGERS_PKT	0x5D
#define MTK_FINGERS_PKT		0x6D
#define TEN_FINGERS_PKT		0x62
#define BUFFER_PKT		0x63
#define BUFFER55_PKT		0x66
/*************************dev file macro switch************************/
#define ELAN_IAP_DEV

#ifdef ELAN_IAP_DEV
// For Firmware Update 
#define ELAN_IOCTLID	0xD0
#define IOCTL_I2C_SLAVE	_IOW(ELAN_IOCTLID,  1, int)
#define IOCTL_MAJOR_FW_VER  _IOR(ELAN_IOCTLID, 2, int)
#define IOCTL_MINOR_FW_VER  _IOR(ELAN_IOCTLID, 3, int)
#define IOCTL_RESET  _IOR(ELAN_IOCTLID, 4, int)
#define IOCTL_IAP_MODE_LOCK  _IOR(ELAN_IOCTLID, 5, int)
#define IOCTL_CHECK_RECOVERY_MODE  _IOR(ELAN_IOCTLID, 6, int)
#define IOCTL_FW_VER  _IOR(ELAN_IOCTLID, 7, int)
#define IOCTL_X_RESOLUTION  _IOR(ELAN_IOCTLID, 8, int)
#define IOCTL_Y_RESOLUTION  _IOR(ELAN_IOCTLID, 9, int)
#define IOCTL_FW_ID  _IOR(ELAN_IOCTLID, 10, int)
#define IOCTL_ROUGH_CALIBRATE  _IOR(ELAN_IOCTLID, 11, int)
#define IOCTL_IAP_MODE_UNLOCK  _IOR(ELAN_IOCTLID, 12, int)
#define IOCTL_I2C_INT  _IOR(ELAN_IOCTLID, 13, int)
#define IOCTL_RESUME  _IOR(ELAN_IOCTLID, 14, int)
#define IOCTL_POWER_LOCK  _IOR(ELAN_IOCTLID, 15, int)
#define IOCTL_POWER_UNLOCK  _IOR(ELAN_IOCTLID, 16, int)
#define IOCTL_FW_UPDATE  _IOR(ELAN_IOCTLID, 17, int)
#define IOCTL_BC_VER  _IOR(ELAN_IOCTLID, 18, int)
#define IOCTL_2WIREICE  _IOR(ELAN_IOCTLID, 19, int)
#define IOCTL_VIAROM	_IOR(ELAN_IOCTLID, 20, int) 
#define IOCTL_VIAROM_CHECKSUM	_IOW(ELAN_IOCTLID, 21, unsigned long)

#define IOCTL_GET_UPDATE_PROGREE	_IOR(CUSTOMER_IOCTLID,  2, int)

#endif


/**********************update firmware macro switch*******************/

#define IAP_PORTION  //!!!!

#if defined IAP_PORTION || defined ELAN_RAM_XX

//elan add updata
static uint8_t * file_fw_data_slave = NULL;
static uint8_t * file_fw_data_master = NULL;
	/*The newest firmware, if update must be changed here*/
	static uint8_t file_fw_data_elan_slave_hlt_black[] = {
		#include "fw_data_slave_hlt_black.i"
	};
	
	static uint8_t file_fw_data_elan_master_hlt_black[] = {
		#include "fw_data_master_hlt_black.i"
	};
	
	static uint8_t file_fw_data_elan_slave_dpt_black[] = {
		#include "fw_data_slave_dpt_black.i"
	};
	
	static uint8_t file_fw_data_elan_master_dpt_black[] = {
		#include "fw_data_master_dpt_black.i"
	};
	
	static uint8_t file_fw_data_elan_slave_hlt_white[] = {
		#include "fw_data_slave_hlt_white.i"
	};
	
	static uint8_t file_fw_data_elan_master_hlt_white[] = {
		#include "fw_data_master_hlt_white.i"
	};
	
	static uint8_t file_fw_data_elan_slave_dpt_white[] = {
		#include "fw_data_slave_dpt_white.i"
	};
	
	static uint8_t file_fw_data_elan_master_dpt_white[] = {
		#include "fw_data_master_dpt_white.i"
	};
//end
	enum
	{
		PageSize = 132,
		ACK_Fail = 0x00,
		ACK_OK = 0xAA,
		ACK_REWRITE= 0x55,
		E_FD = -1,
	};
#endif


/**********************elan attr file macro switch*******************/
//#define SYS_ATTR_FILE
#ifdef SYS_ATTR_FILE
	static struct kobject *android_touch_kobj;
#endif

#endif /* _LINUX_ELAN_TS_H */

