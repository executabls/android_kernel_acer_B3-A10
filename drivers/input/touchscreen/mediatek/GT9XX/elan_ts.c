#include "tpd.h"
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/rtpm_prio.h>
#include <linux/wait.h>
#include <linux/time.h>
#include <linux/delay.h>
#include <linux/acpi.h>
#include <linux/completion.h>

#ifdef MT6575
#include <mach/mt6575_pm_ldo.h>
#include <mach/mt6575_typedefs.h>
#include <mach/mt6575_boot.h>

#else

#include <mach/mt_pm_ldo.h>
#include <mach/mt_typedefs.h>
#include <mach/mt_boot.h>
#include <mach/mt_gpio.h>
#endif
// lgx
#include <mach/hardwareinfo.h>  

#include "cust_gpio_usage.h"
#include <cust_eint.h>

#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/dma-mapping.h>
#include <asm/atomic.h>
#include <linux/proc_fs.h> 



#include "elan_ts.h"
#define IRQ_CMD_HANDLER
#define DEVICE_NAME "elan_ktf"
#define reinit_completion(x) INIT_COMPLETION(*(x))

static struct workqueue_struct *init_elan_ic_wq = NULL;
static struct delayed_work init_work;
static unsigned long delay = 2*HZ;

static int X_RESOLUTION=720;
static int Y_RESOLUTION=1280;
int FW_VERSION=0;
int user_flag=0;
int FW_ID=0; //add elan 2015/7/7
int FW_TEST_V=0xF;

#define GESTURE_PROC_NAME  "acer_EnableGesture"
static char mProcData[10];
static bool mIsEnableGestureWakeUp = true;
static bool mIsEnabletwofinger = true;
static bool mIsEnablefivefinger = true;
static bool mIsEnableDoubleTab = true;
static bool mIsEnableVirtualHomeKey = true;
static bool mIsEnableSliderPoweronoff = true;
static bool mIsEnableSmartCover = true;
static struct proc_dir_entry *mProc_dir_entry;

#ifdef	IRQ_CMD_HANDLER
#define	CMD_RESP_MAX	200
struct completion irq_completion;
unsigned int rx_size=PACKET_SIZE;
char cmd_resp[CMD_RESP_MAX];
#define	ELAN_CMD_RESP_TIMEOUT_MSEC	10000
/* Firmware boot mode packets definition */
#define CMD_PACKET_LEN	4
static const char hello_packet[CMD_PACKET_LEN] = {0x55, 0x55, 0x55, 0x55};
static const char recov_packet[CMD_PACKET_LEN] = {0x55, 0x55, 0x80, 0x80};
static const char cal_packet[CMD_PACKET_LEN] = {0x66, 0x66, 0x66, 0x66};
#define CMD_GET_TRACE_TABLE	0x59
#define	CMD_HEADER_RESP		0x52
#define	CMD_HEADER_5B_RESP	0x9B
#define	CMD_HEADER_HELLO	0x55
#define	CMD_HEADER_REK		0x66
#define	CMD_HEADER_RAWDATA		0x98
#define CHD_HEADER_TRACE_TABLE	0x99
#define WDT_RESET_TIMEOUT_MSEC		500
#define ELAN_CMD_TIMEOUT_MSEC		10000
#define RAWDATA_CMD_TIMEOUT_MSEC	10000
#endif


/*********************************custom module********************************************/
#if 1 //defined MT6592 || defined MT6582 || defined MT6589 || defined MT6577
#define I2C_BORAD_REGISTER
#endif

#ifndef MT6575
#define ELAN_I2C_DMA_MOD
static uint8_t *gpDMABuf_va = NULL;
static uint32_t gpDMABuf_pa = NULL;
#endif

//#define ELAN_ESD_CHECK
#ifdef ELAN_ESD_CHECK
    static struct workqueue_struct *esd_wq = NULL;
	static struct delayed_work esd_work;
	static atomic_t elan_cmd_response = ATOMIC_INIT(0);
#endif

/*********************************platform data********************************************/
//must be init first

static const struct i2c_device_id elan_ts_id[] = {
	{ELAN_TS_NAME, 0 },
	{ }
};

#ifdef I2C_BORAD_REGISTER 
	static struct i2c_board_info __initdata i2c_tpd={ I2C_BOARD_INFO(ELAN_TS_NAME, ELAN_7BITS_ADDR)};
#else
	static unsigned short force[] = {0, ELAN_8BITS_ADDR, I2C_CLIENT_END,I2C_CLIENT_END};
	static const unsigned short * const forces[] = { force, NULL };
	static struct i2c_client_address_data addr_data = { .forces = forces,};
#endif
/**********************************elan struct*********************************************/

struct elan_ts_data{
	struct i2c_client *client;
	struct input_dev *input_dev;
//queue or thread handler interrupt	
	struct task_struct *work_thread;
	struct work_struct  work;
#ifdef CONFIG_HAS_EARLYSUSPEND
//used for early_suspend
	struct early_suspend early_suspend;
#endif
	
//Firmware Information
	int fw_ver;
	int fw_id;
	int fw_bcd;
	int x_resolution;
	int y_resolution;
	int recover;//for iap mod
//for suspend or resum lock
 	int power_lock;
 	int circuit_ver;
//for button state
 	int button_state;
//For Firmare Update 
	struct miscdevice firmware;
	struct proc_dir_entry *p;
};

/************************************global elan data*************************************/
static int tpd_flag = 0;
static int boot_normal_flag = 1;
static int tpd_is_suspend = 0;
static int file_fops_addr = ELAN_7BITS_ADDR;

static struct elan_ts_data *private_ts;
extern struct tpd_device *tpd;
static DECLARE_WAIT_QUEUE_HEAD(waiter);

/*********************************global elan function*************************************/
#if 1//defined MT6592 || defined MT6582
extern void mt_eint_unmask(unsigned int line);
extern void mt_eint_mask(unsigned int line);
extern void mt_eint_set_hw_debounce(unsigned int eintno, unsigned int ms);
extern unsigned int mt_eint_set_sens(unsigned int eintno, unsigned int sens);
extern void mt_eint_registration(unsigned int eint_num, unsigned int flag, 
              void (EINT_FUNC_PTR) (void), unsigned int is_auto_umask);
#else
 extern void mt_eint_unmask(unsigned int line);
 extern void mt65xx_eint_mask(unsigned int line);
 extern void mt65xx_eint_set_hw_debounce(kal_uint8 eintno, kal_uint32 ms);
 extern kal_uint32 mt65xx_eint_set_sens(kal_uint8 eintno, kal_bool sens);
 extern void mt65xx_eint_registration(kal_uint8 eintno, kal_bool Dbounce_En,
									  kal_bool ACT_Polarity, void (EINT_FUNC_PTR)(void),
									  kal_bool auto_umask);
 #endif
static int __hello_packet_handler(struct i2c_client *client);
static int __fw_packet_handler(struct i2c_client *client);
static int elan_ts_rough_calibrate(struct i2c_client *client);

/************************************** function list**************************************/
extern hardware_info_struct hardware_info;  // lgx
//u16  fw_version = 0x00 ;

#ifdef IRQ_CMD_HANDLER
/*
* Print Elan message for buffer array data
*/
void elan_array_printk(char * function, uint8_t *buf, int len)
{
	int i = 0;
	printk("%s ", function);
	for(i = 0; i < len; i++)
	{
		printk("%2x ", buf[i]);
	}
	printk("\n");
	return;
}
#endif

static void elan_reset(void)
{
	printk("[elan]:%s enter\n", __func__);
	mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
	mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_CTP_RST_PIN, 1);
	msleep(10);
	mt_set_gpio_out(GPIO_CTP_RST_PIN, 0);
	msleep(10);
	mt_set_gpio_out(GPIO_CTP_RST_PIN, 1);
	msleep(10);
}

static void elan_switch_irq(int on)
{
	printk("[elan] %s enter, irq switch on = %d\n", __func__, on);
	if(on){
    #if 1//defined MT6592 || defined MT6582	    
        mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);	
    #else
        mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);

    #endif		
	}
	else{
    #if 1//defined MT6592 || defined MT6582	    
		mt_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
    #else
        mt65xx_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
    #endif		    
		
	}
}

static int elan_i2c_send_data(struct i2c_client *client, uint8_t *buf, uint8_t len)
{
	int rc = 0;
	int i = 0;
	
#ifdef ELAN_I2C_DMA_MOD	
	if(buf == NULL || gpDMABuf_va == NULL){
		printk("[elan] BUFFER is NULL!!!!!\n");
		return -1;
	}
	
	for(i = 0 ; i < len; i++){
		gpDMABuf_va[i] = buf[i];
		//printk("%02x ", buf[i]);
	}
	
	rc = i2c_master_send(client, gpDMABuf_pa, len);	
	
#else
	rc = i2c_master_send(client, buf, len);
#endif
	return rc;
}

static int elan_i2c_recv_data(struct i2c_client *client, uint8_t *buf, uint8_t len)
{
	int rc = 0;
	int i = 0;

#ifdef ELAN_I2C_DMA_MOD	
	if(buf == NULL || gpDMABuf_va == NULL){
		printk("[elan] BUFFER is NULL!!!!!\n");
		return -1;
	}
	
	memset(buf, 0, len);
	
	rc = i2c_master_recv(client, gpDMABuf_pa, len);
	
	if(rc >= 0){
	    for(i = 0 ; i < len; i++){
			buf[i] = gpDMABuf_va[i];
			//printk("%02x ", buf[i]);
		}
	}
	
#else
	rc = i2c_master_recv(client, buf, len);
#endif
	
	return rc;
}

 static int tpd_detect (struct i2c_client *client, int kind, struct i2c_board_info *info) 
{
	strcpy(info->type, ELAN_TS_NAME);	
	return 0;
}

static int elan_ts_poll(void)
{
	int status = 0, retry = 20;

	do {
		status = mt_get_gpio_in(GPIO_CTP_EINT_PIN);
		printk("[elan]: %s: status = %d\n", __func__, status);
		retry--;
		msleep(40);
	} while (status == 1 && retry > 0);

	printk( "[elan]%s: poll interrupt status %s\n", __func__, status == 1 ? "high" : "low");
	
	return status == 0 ? 0 : -ETIMEDOUT;
}

static int elan_ts_send_cmd(struct i2c_client *client, uint8_t *cmd, size_t size)
{
	printk("[elan] dump cmd: %02x, %02x, %02x, %02x\n", cmd[0], cmd[1], cmd[2], cmd[3]);
	if (elan_i2c_send_data(client, cmd, size) != size) {
		printk("[elan error]%s: elan_ts_send_cmd failed\n", __func__);
		return -EINVAL;
	}
	else{
		elan_info("[elan] elan_ts_send_cmd ok");
	}
	return size;
}

static int elan_ts_get_data(struct i2c_client *client, uint8_t *cmd, uint8_t *buf, size_t size)
{
	int rc;

	if (buf == NULL){
		return -EINVAL;
    }
	if (elan_ts_send_cmd(client, cmd, size) != size){
		return -EINVAL;
    }
	msleep(2);
	
	rc = elan_ts_poll();
	if (rc < 0){
		return -EINVAL;
	}
	else {
		rc = elan_i2c_recv_data(client, buf, size);
		printk("[elan] %s: respone packet %2x:%2X:%2x:%2x\n", __func__, buf[0], buf[1], buf[2], buf[3]); 
		if(buf[0] != CMD_S_PKT || rc != size){ 
			printk("[elan error]%s: cmd respone error\n", __func__);
			return -EINVAL;
		}
	}
	
	return 0;
}

static int __hello_packet_handler(struct i2c_client *client)
{
	int rc;
	uint8_t buf_recv[8] = { 0 };
	uint8_t cmd[] = {0x53, 0x00, 0x00, 0x01};
	
	rc = elan_ts_poll();
	if(rc != 0){
		printk("[elan] %s: Int poll 55 55 55 55 failed!\n", __func__);
	}
			
	rc = elan_i2c_recv_data(client, buf_recv, sizeof(buf_recv));
	if(rc != sizeof(buf_recv)){
		printk("[elan error] __hello_packet_handler recv error\n");
		return -1;
	}
	printk("[elan] %s: hello packet %2x:%2X:%2x:%2x\n", __func__, buf_recv[0], buf_recv[1], buf_recv[2], buf_recv[3]);
	if(buf_recv[0]!=0x55 && buf_recv[1]!=0x55 )
		return -1;
	if(buf_recv[0]==0x55 && buf_recv[1]==0x55 && buf_recv[2]==0x80 && buf_recv[3]==0x80){
		printk("[elan] %s: boot code packet %2x:%2X:%2x:%2x\n", __func__, buf_recv[4], buf_recv[5], buf_recv[6], buf_recv[7]);
		rc = 0x80;
	}
	else if(buf_recv[0]==0x55 && buf_recv[1]==0x55 && buf_recv[2]==0x55 && buf_recv[3]==0x55){
		printk("[elan] __hello_packet_handler recv ok\n");
		rc = 0x0;
	}
	else{
		if(rc != sizeof(buf_recv)){
			rc = elan_i2c_send_data(client, cmd, sizeof(cmd));
			if(rc != sizeof(cmd)){
				msleep(5);
				rc = elan_i2c_recv_data(client, buf_recv, sizeof(buf_recv));
			}
		}
	}
	
	return rc;
}

static int __fw_packet_handler(struct i2c_client *client)
{
	struct elan_ts_data *ts = i2c_get_clientdata(client);
	int rc;
	int major, minor;
	uint8_t cmd[]			= {0x53, 0x00, 0x00, 0x01};/* Get Firmware Version*/
	
	uint8_t cmd_id[] 		= {0x53, 0xf0, 0x00, 0x01}; /*Get firmware ID*/
	uint8_t cmd_bc[]		= {0x53, 0x10, 0x00, 0x01};/* Get BootCode Version*/
	uint8_t cmd_test_v[]		= {0x53,0xe0,0x00,0x01};/*get test_solution* add 2015/7/22*/
#if 1
	int x, y;
	uint8_t cmd_getinfo[] = {0x5B, 0x00, 0x00, 0x00, 0x00, 0x00};
	uint8_t adcinfo_buf[17]={0};
#else
	uint8_t cmd_x[] 		= {0x53, 0x60, 0x00, 0x00}; /*Get x resolution*/
	uint8_t cmd_y[] 		= {0x53, 0x63, 0x00, 0x00}; /*Get y resolution*/
#endif	
	uint8_t buf_recv[4] 	= {0};
// Firmware version
	rc = elan_ts_get_data(client, cmd, buf_recv, 4);
	if (rc < 0)
		return rc;
	major = ((buf_recv[1] & 0x0f) << 4) | ((buf_recv[2] & 0xf0) >> 4);
	minor = ((buf_recv[2] & 0x0f) << 4) | ((buf_recv[3] & 0xf0) >> 4);
	ts->fw_ver = major << 8 | minor;
	FW_VERSION=major << 8 | minor;
	hardware_info.tpd_fw_version= FW_VERSION ;
// Firmware ID
	rc = elan_ts_get_data(client, cmd_id, buf_recv, 4);
	if (rc < 0)
		return rc;
	major = ((buf_recv[1] & 0x0f) << 4) | ((buf_recv[2] & 0xf0) >> 4);
	minor = ((buf_recv[2] & 0x0f) << 4) | ((buf_recv[3] & 0xf0) >> 4);
	ts->fw_id = major << 8 | minor;
	FW_ID= major << 8 | minor; //add elan 2015/7/7
	hardware_info.tpd_module_id = FW_ID;

	// Firmware TEST
	rc = elan_ts_get_data(client, cmd_test_v, buf_recv, 4);
	if (rc < 0)
		return rc;
	FW_TEST_V=((buf_recv[1] & 0x0f) << 4) | ((buf_recv[2] & 0xf0) >> 4);
	hardware_info.tpd_color = FW_TEST_V;
#if 0
	// X Resolution
	rc = elan_ts_get_data(client, cmd_x, buf_recv, 4);
	if (rc < 0)
		return rc;
	minor = ((buf_recv[2])) | ((buf_recv[3] & 0xf0) << 4);
	X_RESOLUTION = minor;

	// Y Resolution	
	rc = elan_ts_get_data(client, cmd_y, buf_recv, 4);
	if (rc < 0)
		return rc;
	minor = ((buf_recv[2])) | ((buf_recv[3] & 0xf0) << 4);
	Y_RESOLUTION = minor;
#else
	elan_i2c_send_data(client, cmd_getinfo, sizeof(cmd_getinfo));
	msleep(10);
	elan_i2c_recv_data(client, adcinfo_buf, 17);
	x  = adcinfo_buf[2]+adcinfo_buf[6]+adcinfo_buf[10]+adcinfo_buf[14];
	y  = adcinfo_buf[3]+adcinfo_buf[7]+adcinfo_buf[11]+adcinfo_buf[15];

	printk( "[elan] %s: x= %d, y=%d\n",__func__,x,y);

	X_RESOLUTION=(x-1)*48;
	Y_RESOLUTION=(y-1)*48;
#endif
	
// Firmware BC
	rc = elan_ts_get_data(client, cmd_bc, buf_recv, 4);
	if (rc < 0)
		return rc;
	major = ((buf_recv[1] & 0x0f) << 4) | ((buf_recv[2] & 0xf0) >> 4);
	minor = ((buf_recv[2] & 0x0f) << 4) | ((buf_recv[3] & 0xf0) >> 4);
	ts->fw_bcd = major << 8 | minor;	
	
	printk( "[elan] %s: firmware version: 0x%4.4x\n",__func__, ts->fw_ver);
	printk( "[elan] %s: firmware ID: 0x%4.4x\n",__func__, ts->fw_id);
	printk( "[elan] %s: firmware BC: 0x%4.4x\n",__func__, ts->fw_bcd);
	printk( "[elan] %s: FW_TEST_V: 0x%x\n",__func__, FW_TEST_V);
	printk( "[elan] %s: x resolution: %d, y resolution: %d\n",__func__, X_RESOLUTION, Y_RESOLUTION);
	
	return 0;
}

#if defined IAP_PORTION || defined ELAN_RAM_XX
int WritePage(struct i2c_client *client, uint8_t * szPage, int byte, int which)
{
	int len = 0;
	
	len = elan_i2c_send_data(client, szPage,  byte);
	if (len != byte) {
		printk("[elan] ERROR: write the %d th page error, write error. len=%d\n", which, len);
		return -1;
	}
	
	return 0;
}

/*every page write to recv 2 bytes ack */
int GetAckData(struct i2c_client *client, uint8_t *ack_buf)
{
	int len = 0;
	
	len=elan_i2c_recv_data(client, ack_buf, 2);
	if (len != 2) {
		printk("[elan] ERROR: GetAckData. len=%d\r\n", len);
		return -1;
	}
	
	if (ack_buf[0] == 0xaa && ack_buf[1] == 0xaa) {
		return ACK_OK;
	}
	else if (ack_buf[0] == 0x55 && ack_buf[1] == 0x55){
		return ACK_REWRITE;
	}
	else{
		return ACK_Fail;
	}
	return 0;
}

/* Check Master & Slave is "55 aa 33 cc" */
int CheckIapMode(struct i2c_client *client)
{
	char buff[4] = {0};
	int rc = 0;
	
	rc = elan_i2c_recv_data(client, buff, 4);
	if (rc != 4) {
		printk("[elan] ERROR: CheckIapMode. len=%d\r\n", rc);
		return -1;
	}
	else
		printk("[elan] Mode= 0x%x 0x%x 0x%x 0x%x\r\n", buff[0], buff[1], buff[2], buff[3]);
		
	return 0;	
}

void update_fw_one(struct i2c_client *client)
{
	uint8_t ack_buf[2] = {0};
	struct elan_ts_data *ts = i2c_get_clientdata(client);
	uint8_t hello_packet[8]={0};
	int res = 0;
	int iPage = 0;
	
	uint8_t data;
	int PageNum_slave;
	int PageNum_master;
	const int PageSize = 132;
	if(ts->fw_id==0x0D38)
	{
    	PageNum_slave = sizeof(file_fw_data_elan_slave_hlt_black)/PageSize;
		PageNum_master = sizeof(file_fw_data_elan_master_hlt_black)/PageSize;
    }
    else if(ts->fw_id==0x0D39)
    {
    	PageNum_slave = sizeof(file_fw_data_elan_slave_dpt_black)/PageSize;
		PageNum_master = sizeof(file_fw_data_elan_master_dpt_black)/PageSize;
    }
	else
	{
    	PageNum_slave = sizeof(file_fw_data_elan_slave_hlt_black)/PageSize;
		PageNum_master = sizeof(file_fw_data_elan_master_hlt_black)/PageSize;
	}
	
	const int PAGERETRY = 10;
	const int IAPRESTART = 3;
	
	int restartCnt = 0; // For IAP_RESTART
	int restartCnt_master = 0; // For IAP_RESTART_master
	int rewriteCnt = 0;// For IAP_REWRITE
	
	int iap_mod;
	
	uint8_t *szBuff = NULL;
	int curIndex = 0;

#ifdef ELAN_2K_XX	
	uint8_t isp_cmd[] = {0x54, 0x00, 0x12, 0x34};	 //54 00 12 34
	iap_mod = 2;
#endif

#ifdef ELAN_3K_XX 
	uint8_t isp_cmd[] = {0x45, 0x49, 0x41, 0x50};	 //45 49 41 50
	iap_mod = 3;	
#endif

#ifdef ELAN_RAM_XX 
	uint8_t isp_cmd[] = {0x22, 0x22, 0x22, 0x22};	 //22 22 22 22
	iap_mod = 1;	
#endif

	elan_switch_irq(0);
	ts->power_lock = 1;
	
	data=0x20;
	printk( "[elan] %s: address data=0x%x iap_mod=%d PageNum_slave = %d  PageNum_master = %d\r\n", __func__, data, iap_mod, PageNum_slave,PageNum_master);
	
IAP_RESTART:
	//reset tp
	if(iap_mod == 3){
		elan_reset();
	}
	
	if((iap_mod != 2) || (ts->recover != 0x80)){
		printk("[elan] Firmware update normal mode !\n");
		//Step 1 enter isp mod
		res = elan_ts_send_cmd(client, isp_cmd, sizeof(isp_cmd));
		//Step 2 Chech IC's status is 55 aa 33 cc
		if(iap_mod == 2||iap_mod == 3){
			res = CheckIapMode(client);
		}
	} else{
		printk("[elan] Firmware update recovery mode !\n");	
	}
	
	//Step 3 Send Dummy Byte
	res = elan_i2c_send_data(client, &data,  sizeof(data));
	if(res!=sizeof(data)){
		printk("[elan] dummy error code = %d\n",res);
		return;
	}
	else{
		printk("[elan] send Dummy byte sucess data:%x", data);
	}	
	
	msleep(10);
	

	//Step 4 Start IAP
	for( iPage = 1; iPage <= PageNum_slave; iPage++ ) {
			
		szBuff = file_fw_data_slave + curIndex;
		curIndex =  curIndex + PageSize;

PAGE_REWRITE:
		res = WritePage(client, szBuff, PageSize, iPage);
#if 1		
		if(iPage==PageNum_slave|| iPage==1){
			msleep(500); 			 
		}
		else{
			msleep(10); 			 
		}
#endif		
		res = GetAckData(client, ack_buf);
		if (ACK_OK != res) {
			
			msleep(50); 
			printk("[elan]: %d page ack error: ack0:%x ack1:%x\n",  iPage, ack_buf[0], ack_buf[1]);
			
			if ( res == ACK_REWRITE ){
				rewriteCnt = rewriteCnt + 1;
				if (rewriteCnt == PAGERETRY){
					printk("[elan] ---%d--- page ReWrite %d times!\n",  iPage, rewriteCnt);
					return;
				}
				else{
					printk("[elan] ---%d--- page ReWrite %d times! failed\n",  iPage, rewriteCnt);
					goto PAGE_REWRITE;
				}
			}
			else{
				restartCnt = restartCnt + 1;
				if (restartCnt >= IAPRESTART){
					printk("[elan] try to ReStart %d times !\n", restartCnt);
					return;
				}
				else{
					printk("[elan] ReStart %d times fails!\n", restartCnt);
					curIndex = 0;
					goto IAP_RESTART;
				}
			}
		}
		else{
			printk("[elan]---%d--- page flash ok\n", iPage);
			rewriteCnt=0;
		}
	}
	
	data=0x10;
	curIndex = 0;
	//Step 5 Send Dummy Byte 
	res = elan_i2c_send_data(client, &data,  sizeof(data));
	if(res!=sizeof(data)){
		printk("[elan] dummy error code = %d\n",res);
		return;
	}
	else{
		printk("[elan] send Dummy byte sucess data:%x", data);
	}		
	//msleep(20);	
	//Step 6 Start IAP
	for( iPage = 1; iPage <= PageNum_master; iPage++ ) {
			
		szBuff = file_fw_data_master+ curIndex;
		curIndex =  curIndex + PageSize;

PAGE_REWRITE_master:
		res = WritePage(client, szBuff, PageSize, iPage);
		 			 
#if 1		
		if(iPage==PageNum_master|| iPage==1){
			msleep(500); 			 
		}
		else{
			msleep(10); 			 
		}
#endif		
		res = GetAckData(client, ack_buf);
		if (ACK_OK != res) {
			
			msleep(50); 
			printk("[elan]: %d page ack error: ack0:%x ack1:%x\n",  iPage, ack_buf[0], ack_buf[1]);
			
			if ( res == ACK_REWRITE ){
				rewriteCnt = rewriteCnt + 1;
				if (rewriteCnt == PAGERETRY){
					printk("[elan] ---%d--- page ReWrite %d times!\n",  iPage, rewriteCnt);
					return;
				}
				else{
					printk("[elan] ---%d--- page ReWrite %d times! failed\n",  iPage, rewriteCnt);
					goto PAGE_REWRITE_master;
				}
			}
			else{
				restartCnt_master = restartCnt_master + 1;
				if (restartCnt_master >= IAPRESTART){
					printk("[elan] try to ReStart %d times !\n", restartCnt_master);
					return;
				}
				else{
					printk("[elan] ReStart %d times fails!\n", restartCnt_master);
					curIndex = 0;
					goto IAP_RESTART;
				}
			}
		}
		else{
			printk("[elan]---%d--- page flash ok\n", iPage);
			rewriteCnt=0;
		}
	}

	elan_reset();
	elan_switch_irq(1);
	ts->power_lock = 0;
	
	printk("[elan] Update ALL Firmware successfully!\n");
	
	return;
}
#endif
static inline int elan_ts_parse_xy(uint8_t *data, uint16_t *x, uint16_t *y)
{
	*x = *y = 0;

	*x = (data[0] & 0xf0);
	*x <<= 4;
	*x |= data[1];

	*y = (data[0] & 0x0f);
	*y <<= 8;
	*y |= data[2];

	return 0;
}

static int elan_ts_setup(struct i2c_client *client)
{
	int rc = 0;
	elan_reset();
	msleep(200);
	
	rc = __hello_packet_handler(client);
	if (rc < 0){
		printk("[elan error] %s, hello_packet_handler fail, rc = %d\n", __func__, rc);
	}
	return rc;
}

static int elan_ts_set_power_state(struct i2c_client *client, int state)
{
	uint8_t cmd[] = {CMD_W_PKT, 0x50, 0x00, 0x01};
	int size = sizeof(cmd);
	
	cmd[1] |= (state << 3);
	if (elan_ts_send_cmd(client, cmd, size) != size){
		return -EINVAL;
	}	

	return 0;
}

static void elan_ts_touch_down(struct elan_ts_data* ts,s32 id,s32 x,s32 y,s32 w)
{
	input_report_key(ts->input_dev, BTN_TOUCH, 1);
	input_report_abs(ts->input_dev, ABS_MT_PRESSURE, w);
	input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, w);
	input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, w);
	input_report_abs(ts->input_dev, ABS_MT_POSITION_X, x);
	input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, y);
	input_mt_sync(ts->input_dev);

	elan_info("Touch ID:%d, X:%d, Y:%d, W:%d down", id, x, y, w); 
}

static void elan_ts_touch_up(struct elan_ts_data* ts,s32 id,s32 x,s32 y)
{
	input_report_key(ts->input_dev, BTN_TOUCH, 0);
	input_report_abs(ts->input_dev, ABS_MT_PRESSURE, 0);
	input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0);
	input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0);
	input_mt_sync(ts->input_dev);
	input_sync(ts->input_dev);
	elan_info("Touch all release!");
}

static void elan_ts_report_key(struct elan_ts_data *ts, uint8_t button_data)
{
	static unsigned int x = 0,y = 0;

	switch (button_data) {
		case ELAN_KEY_MENU:
			x = tpd_keys_dim_local[0][0];
			y = TPD_BUTTON_MAX;
			if(boot_normal_flag == 1){
				elan_ts_touch_down(ts, 0, x, y, 8);
			}
			else{
				tpd_button(x, y, 1);
			}
			break;
		case ELAN_KEY_HOME:
			x = tpd_keys_dim_local[1][0];
			y = TPD_BUTTON_MAX;
			if(boot_normal_flag == 1){
				elan_ts_touch_down(ts, 0, x, y, 8);
			}
			else{
				tpd_button(x, y, 1);
			}
			break;
		case ELAN_KEY_BACK:		
			x = tpd_keys_dim_local[2][0];
			y = TPD_BUTTON_MAX;
			if(boot_normal_flag == 1){
				elan_ts_touch_down(ts, 0, x, y, 8);
			}
			else{
				tpd_button(x, y, 1);
			}
			break;
		default:
			if(boot_normal_flag == 1){
				elan_ts_touch_up(ts, 0, x, y);
			}
			else{
				tpd_button(x, y, 0);
			}
			break;
	}
}

#if defined ELAN_RAM_XX
static void elan_ts_iap_ram_continue(struct i2c_client *client)
{
	uint8_t cmd[] = { 0x33, 0x33, 0x33, 0x33 };
	int size = sizeof(cmd);

	elan_ts_send_cmd(client, cmd, size);
}
#endif

static void elan_ts_handler_event(struct elan_ts_data *ts, uint8_t *buf)
{
	int rc = 0;
	
	if(buf[0] == 0x55){
		if(buf[2] == 0x55){
			ts->recover = 0;
		}
//elan add updata
		else if(buf[2] == 0x80&&buf[4]==0x95){
			//ts->fw_id=0x0B95;
			ts->recover = 0x80;
		}
		else if(buf[2] == 0x80&&buf[4]==0x93){
			//ts->fw_id=0x0B93;
			ts->recover = 0x80;
		}
//end
	}
#ifdef ELAN_ESD_CHECK	
	else if(buf[0] == 0x52 || buf[0] == 0x78){
		atomic_set(&elan_cmd_response, 1);
	}
#endif	
}

#ifdef ELAN_IAP_DEV
int elan_iap_open(struct inode *inode, struct file *filp)
{ 
	elan_info("%s enter", __func__);
	if (private_ts == NULL){
		printk("private_ts is NULL~~~");
	}	
	return 0;
}

int elan_iap_release(struct inode *inode, struct file *filp)
{    
	elan_info("%s enter", __func__);
	return 0;
}

static ssize_t elan_iap_write(struct file *filp, const char *buff, size_t count, loff_t *offp)
{  
	int ret;
	char *tmp;
	struct i2c_client *client = private_ts->client;
	
	elan_info("%s enter", __func__);
	if (count > 8192){
		count = 8192;
	}
	tmp = kmalloc(count, GFP_KERNEL);
	if (tmp == NULL){
		return -ENOMEM;
	}
	if (copy_from_user(tmp, buff, count)) {
		return -EFAULT;
	}
	
	ret = elan_i2c_send_data(client, tmp, count);
	if (ret != count){ 
		printk("elan elan_i2c_send_data fail, ret=%d \n", ret);
	}
	kfree(tmp);
	
	return ret;
}

ssize_t elan_iap_read(struct file *filp, char *buff, size_t count, loff_t *offp)
{    
	char *tmp;
	int ret;  
	long rc;
	
	struct i2c_client *client = private_ts->client;
	
	elan_info("%s enter", __func__);
	
	if (count > 8192){
		count = 8192;
	}
	tmp = kmalloc(count, GFP_KERNEL);
	if (tmp == NULL){
		return -ENOMEM;
	}
	ret = elan_i2c_recv_data(client, tmp, count);
	if (ret != count){ 
		printk("elan elan_i2c_recv_data fail, ret=%d \n", ret);
	}
	if (ret == count){
		rc = copy_to_user(buff, tmp, count);
	}
	kfree(tmp);
	return ret;
}

static long elan_iap_ioctl( struct file *filp, unsigned int cmd, unsigned long arg)
{
	int __user *ip = (int __user *)arg;
	int status = 0; // Paul@0150727
	char buf[4] = {0};
	
	elan_info("%s enter, cmd value %x\n",__func__, cmd);

	switch (cmd) {        
		case IOCTL_I2C_SLAVE:
			printk("[elan debug] pre addr is %X\n",  private_ts->client->addr); 
			private_ts->client->addr = (int __user)arg;
			printk("[elan debug] new addr is %X\n",  private_ts->client->addr); 
			break;
		case IOCTL_RESET:
			elan_reset();
			break;
		case IOCTL_IAP_MODE_LOCK:
			if(private_ts->power_lock==0){
				private_ts->power_lock=1;
				elan_switch_irq(0);
			}
			break;
		case IOCTL_IAP_MODE_UNLOCK:
			if(private_ts->power_lock==1){			
				private_ts->power_lock=0;
				elan_switch_irq(1);
			}
			break;
		case IOCTL_CHECK_RECOVERY_MODE:
			return private_ts->recover;;
			break;
		case IOCTL_FW_VER:
			__fw_packet_handler(private_ts->client);
			return FW_VERSION;
			break;
		case IOCTL_X_RESOLUTION:
			__fw_packet_handler(private_ts->client);
			return X_RESOLUTION;
			break;
		case IOCTL_Y_RESOLUTION:
			__fw_packet_handler(private_ts->client);
			return Y_RESOLUTION;
			break;
		case IOCTL_FW_ID:	//add elan 2015/7/7
			__fw_packet_handler(private_ts->client);
			return FW_ID;
			break;
		case IOCTL_I2C_INT:
			//put_user(mt_get_gpio_in(GPIO_CTP_EINT_PIN), ip); // Paul@20150727
			status = mt_get_gpio_in(GPIO_CTP_EINT_PIN);
			printk("[elan]: %s: status = %d\n", __func__, status);
			put_user(status, ip);
			break;
		default:            
			break;   
	}       
	return 0;
}

struct file_operations elan_touch_fops = {    
	.open = elan_iap_open,    
	.write = elan_iap_write,    
	.read = elan_iap_read,    
	.release =	elan_iap_release,    
	.unlocked_ioctl = elan_iap_ioctl, 
 };

#endif

#ifdef ELAN_ESD_CHECK
static void elan_touch_esd_func(struct work_struct *work)
{	
	int res;	
	uint8_t cmd[] = {0x53, 0x00, 0x00, 0x01};	
	struct i2c_client *client = private_ts->client;	

	elan_info("esd %s: enter.......", __FUNCTION__);
	
	if(private_ts->power_lock == 1){
		goto elan_esd_check_out;
	}
	
	if(atomic_read(&elan_cmd_response) == 0){
		res = elan_ts_send_cmd(client, cmd, sizeof(cmd));
		if (res != sizeof(cmd)){
			elan_info("[elan esd] %s: elan_ts_send_cmd failed reset now", __func__);
		}
		else{
			msleep(10);
			
			if(atomic_read(&elan_cmd_response) == 0){
				elan_info("esd %s: elan_ts_send_cmd successful, response failed", __func__);
			}
			else{
				elan_info("esd %s: elan_ts_send_cmd successful, response ok", __func__);
				goto elan_esd_check_out;
			}
		}
	}
	else{
		elan_info("esd %s: response ok!!!", __func__);
		goto elan_esd_check_out;
	}
	
	elan_reset();
elan_esd_check_out:	
	
	atomic_set(&elan_cmd_response, 0);
	queue_delayed_work(esd_wq, &esd_work, delay);
	elan_info("[elan esd] %s: out.......", __FUNCTION__);	
	return;
}
#endif	

#ifdef IRQ_CMD_HANDLER
/*
* Elan I2C command read block. Write Elan read
* command and receive related data after INT PIN
* low by ISR function.
*/
static int elan_ktf_i2c_read_block(struct i2c_client *client,
			       uint8_t *cmd, int cmd_len, uint8_t *val, int resp_len,
				   int read_timeout)
{
	int ret, error;
	rx_size = resp_len;
	//INIT_COMPLETION(irq_completion);
	reinit_completion(&irq_completion);
	if(cmd_len != 0) {
		ret = elan_ts_send_cmd(client, cmd, cmd_len);
	}
	ret = wait_for_completion_interruptible_timeout(&irq_completion,
				msecs_to_jiffies(read_timeout));
	if (ret <= 0) {
		error = ret < 0 ? ret : -ETIMEDOUT;
		dev_err(&client->dev,
			"error while waiting for elan response to complete: %d\n",
			error);
			rx_size = PACKET_SIZE;
		return error;
	}
	memcpy(val, cmd_resp, resp_len);
	rx_size = PACKET_SIZE;
	return ret;
}
#endif

#ifdef IRQ_CMD_HANDLER
uint8_t adc_buff[6] ={};// { 0x58, 0x03, 0x00, 0x00, 0x06, 0xa6 }; //0:DISC,1:offset,2:base,3:adc,4:dv
int pagereceive=0;
int TPlength=0;
uint8_t rawdata_buf[63*100] ={};
int rawdata_offset=0;

/*
* Set Elan Rawdata command. ex: ADC, BASE, DV etc.
*/
static ssize_t set_rawdata_cmd(struct device *dev,
struct device_attribute *attr, const char *buf, size_t count)
{
	int i;
	printk("%s: rawdata_cmd[%d]=\"%s\"", __func__, count, buf);

	// Scan Get Raw Data Command
	sscanf(buf,"%x %x %x %x %x %x", (unsigned int *)&adc_buff[0], (unsigned int *)&adc_buff[1], (unsigned int *)&adc_buff[2],
				(unsigned int *)&adc_buff[3], (unsigned int *)&adc_buff[4], (unsigned int *)&adc_buff[5]);
	printk(", size=%d.\r\n", count);
	// Debug
	printk("%s: adc_buff[6]=\"", __func__);
	for(i=0;i<5;i++)
		printk("%x ", adc_buff[i]);
	printk("%x\", ", adc_buff[5]);

	// Set Data Length & Page Number to Receive
	TPlength=(adc_buff[4]<<8) | adc_buff[5];

	// If command is get trace table command, change length to dobule size since length unit is word
	if(adc_buff[0] == CMD_GET_TRACE_TABLE)
		TPlength *= 2;

	if ((TPlength % 60) != 0) {
		pagereceive = TPlength / 60 + 1;
		//printf("page:%d\n",pagereceive);
	} else {
		pagereceive = TPlength / 60;
		//printf("page1:%d\n",pagereceive);
	}

	printk("TPlength=%d, pagereceive=%d.\r\n", TPlength, pagereceive);
	return count;
}

int read_continue=0;

/*
* Get Elan Rawdata by IRQ function.
*/
static ssize_t get_rawdata(struct device *dev,
struct device_attribute *attr, char *buf)
{
	int ret, error;
	//ssize_t rawdata_len = 63 * pagereceive;
	ssize_t rawdata_len = 0,
		rawdata_len_1st_transfer = (63 * pagereceive) / 2,
		rawdata_len_2nd_transfer = (63 * pagereceive) - rawdata_len_1st_transfer;

	if(read_continue==0)
	{
		//INIT_COMPLETION(irq_completion);
		reinit_completion(&irq_completion);
		printk("[elan1111]:%s: rawdata_cmd[%d]=\"%x %x %x %x %x %x\", rawdata_len=%d, pagereceive=%d.\r\n", __func__,
				sizeof(adc_buff), adc_buff[0], adc_buff[1], adc_buff[2], adc_buff[3], adc_buff[4], adc_buff[5],
				rawdata_len, pagereceive);

		// Set IRQ Read Parameters
		rx_size = 63;
		rawdata_offset=0;

		// Set Data Length for 1st Transfer
		rawdata_len = rawdata_len_1st_transfer;

		// Send Get Raw Data Command to Touch
		ret = elan_ts_send_cmd(private_ts->client, adc_buff, sizeof(adc_buff));

		// Wait for Receiving Raw Data Completed
		ret = wait_for_completion_interruptible_timeout(&irq_completion,
					msecs_to_jiffies(RAWDATA_CMD_TIMEOUT_MSEC));

		if (ret <= 0) {
			error = ret < 0 ? ret : -ETIMEDOUT;
			dev_err(&private_ts->client->dev,
				"error while waiting for elan response to complete: %d\n",
				error);
			sprintf(buf, "fail\n");

			// Recover IRQ Read Parameters
			rx_size = PACKET_SIZE;

			return error;
		}

		// Copy Data of Raw Data Pages to Buffer
		memcpy(buf, rawdata_buf, rawdata_len);

		// Set read_continue Flag
		read_continue++;

		// Debug
		printk("%s: 1st run of rawdata tranfer.\r\n", __func__);
	}
	else
	{
		// Set Data Length for 2nd Transfer
		rawdata_len = rawdata_len_2nd_transfer;

		// Copy Data in 2nd Transfer
		memcpy(buf, &rawdata_buf[rawdata_len_1st_transfer], rawdata_len);

		// Clear read_continue Flag
		read_continue=0;

		// Debug
		printk("%s: 2nd run of rawdata tranfer.\r\n", __func__);
	}

	// Recover IRQ Read Parameters
	rx_size = PACKET_SIZE;

	// Done
	return rawdata_len;
}

uint8_t elan_cmd[20]={0};
ssize_t elan_cmd_wcnt, elan_cmd_rcnt;
/*
* Set Elan command.
*/
static ssize_t set_elan_cmd(struct device *dev,
struct device_attribute *attr, const char *buf, size_t count)
{
	int i=0;
	char local_buf[64] = {0};
	char *buf_fp;
	printk("%s: buf[%d]=\"%s\".\r\n", __func__, count, buf);
	memset(elan_cmd, 0, sizeof(elan_cmd));

	elan_cmd_wcnt=0;
	elan_cmd_rcnt=0;
	memcpy(local_buf, buf, count);
	buf_fp=local_buf;
	do
	{
		if(elan_cmd_wcnt!=0) buf_fp++; // Skip " " string

		sscanf(buf_fp,"%x ", (unsigned int *)&elan_cmd[elan_cmd_wcnt]);
		elan_cmd_wcnt++;
		buf_fp=strstr(buf_fp," ");
	}while(buf_fp!=NULL);

	printk("%s: elan_cmd[%d]=\"", __func__, elan_cmd_wcnt);
	for(i = 0; i < elan_cmd_wcnt-1; i++)
		printk("%x ", elan_cmd[i]);
	printk("%x\", ", elan_cmd[elan_cmd_wcnt-1]);

	// Compute Data Length to Read
	if(elan_cmd[0]==0x5b)
		elan_cmd_rcnt=18;
	else
		elan_cmd_rcnt=elan_cmd_wcnt;
	printk("w %d r %d.\r\n", elan_cmd_wcnt, elan_cmd_rcnt);

	return count;
}

/*
* Read Elan command by IRQ function.
*/
static ssize_t get_elan_cmd(struct device *dev,
struct device_attribute *attr, char *buf)
{
	int i=0,
	    err = 0;
	size_t count=elan_cmd_rcnt;
	printk("%s: elan_cmd_wcnt=%d, elan_cmd[]=\"%x %x %x %x %x %x\", elan_cmd_rcnt=%d.\r\n", __func__,
			elan_cmd_wcnt, elan_cmd[0], elan_cmd[1], elan_cmd[2], elan_cmd[3], elan_cmd[4], elan_cmd[5], elan_cmd_rcnt);

	err = elan_ktf_i2c_read_block(private_ts->client, elan_cmd, elan_cmd_wcnt, buf, elan_cmd_rcnt, ELAN_CMD_TIMEOUT_MSEC);
	//printk("%s: elan_ktf_i2c_read_block(%d) return %d.\r\n", __func__, ELAN_CMD_TIMEOUT_MSEC, err);
	if (err <= 0) {
		dev_err(&private_ts->client->dev,
			"%s: error while waiting for elan response to complete: %d\n",
			__func__, err);
		sprintf(buf, "fail\n");
		return err;
	}

	// Debug
	printk("%s: data[%d]=\"", __func__, elan_cmd_rcnt);
	for(i = 0; i < elan_cmd_rcnt-1; i++)
		printk("%x ", buf[i]);
	printk("%x\".\r\n", buf[elan_cmd_rcnt-1]);

	return count;
}
#endif

 
#ifdef SYS_ATTR_FILE
static ssize_t elan_debug_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret = 0;
#ifdef PRINT_INT_INFO 	
	debug_flage = !debug_flage;
	if(debug_flage)
		printk("elan debug switch open\n");
	else
		printk("elan debug switch close\n");
#endif	
	return ret;
}
static DEVICE_ATTR(debug, S_IRUGO, elan_debug_show, NULL);


static ssize_t elan_info_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	struct elan_ts_data *ts = private_ts;
	elan_switch_irq(0);
	__fw_packet_handler(ts->client);
	elan_switch_irq(1);
	sprintf(buf, "elan fw ver:%X,id:%X,x:%d,y:%d\n", ts->fw_ver, ts->fw_id, ts->x_resolution, ts->y_resolution);
	ret = strlen(buf) + 1;
	return ret;
}
static DEVICE_ATTR(info, S_IRUGO, elan_info_show, NULL);

static ssize_t set_cmd_store(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf, size_t size)
{
	char cmd[4] = {0};
	if (size > 4)
		return -EINVAL;
	
	if (sscanf(buf, "%02x:%02x:%02x:%02x\n", (int *)&cmd[0], (int *)&cmd[1], (int *)&cmd[2], (int *)&cmd[3]) != 4){
		printk("elan cmd format error\n");
		return -EINVAL;
	}
	elan_ts_send_cmd(private_ts->client, cmd, 4);
	return size;
}
static DEVICE_ATTR(set_cmd, S_IRUGO, NULL, set_cmd_store);

static struct attribute *sysfs_attrs_ctrl[] = {
	&dev_attr_debug.attr,
	&dev_attr_info.attr,
	&dev_attr_set_cmd.attr,
	NULL
};
static struct attribute_group elan_attribute_group[] = {
	{.attrs = sysfs_attrs_ctrl },
};
#endif
static DEVICE_ATTR(rawdata, S_IRUGO | S_IWUGO, get_rawdata, set_rawdata_cmd);
static DEVICE_ATTR(elan_cmd, S_IRUGO | S_IWUGO, get_elan_cmd, set_elan_cmd);

static struct attribute *elan_attributes[] = {
	
	&dev_attr_rawdata.attr,
	&dev_attr_elan_cmd.attr,
	NULL
		
};

static struct attribute_group elan_attribute_group = {
	.name = DEVICE_NAME,
	.attrs = elan_attributes,
};

static void elan_touch_node_init(void)
{
	int ret ;
	struct elan_ts_data *ts = private_ts;
#ifdef SYS_ATTR_FILE		
	android_touch_kobj = kobject_create_and_add("android_touch", NULL) ;
	if (android_touch_kobj == NULL) {
		printk(KERN_ERR "[elan]%s: kobject_create_and_add failed\n", __func__);
		return;
	}
	
	ret = sysfs_create_group(android_touch_kobj, elan_attribute_group);
	if (ret < 0) {
		printk(KERN_ERR "[elan]%s: sysfs_create_group failed\n", __func__);
	}
#endif
	
#ifdef ELAN_IAP_DEV	
	ts->firmware.minor = MISC_DYNAMIC_MINOR;
	ts->firmware.name = "elan-iap";
	ts->firmware.fops = &elan_touch_fops;
	ts->firmware.mode = S_IFREG|S_IRWXUGO; 

	if (misc_register(&ts->firmware) < 0)
		printk("[elan debug] misc_register failed!!\n");
	else
		printk("[elan debug] misc_register ok!!\n");
		
	ts->p = proc_create("elan-iap", 0666, NULL, &elan_touch_fops);
    if (ts->p == NULL){
        printk("[elan debug] proc_create failed!!\n");        
    }
    else{
        printk("[elan debug] proc_create ok!!\n");        
    }	
#endif

	return;
}

static void elan_touch_node_deinit(void)
{
#ifdef SYS_ATTR_FILE
	if(android_touch_kobj){
		sysfs_remove_group(android_touch_kobj, elan_attribute_group);
		kobject_put(android_touch_kobj);
	}	
#endif

#ifdef ELAN_IAP_DEV	
	misc_deregister(&private_ts->firmware);
	remove_proc_entry("elan-iap", NULL);
#endif

}

static int elan_ts_recv_data(struct elan_ts_data *ts, uint8_t *buf)
{
	int rc;
	int i = 0;
	
	//rc = elan_i2c_recv_data(ts->client, buf, PACKET_SIZE);
	
	rc = elan_i2c_recv_data(ts->client, buf, rx_size);
	//if(PACKET_SIZE != rc){
	if(rx_size != rc){
		printk("[elan error] elan_ts_recv_data\n");
		return -1;
	}
#ifdef PRINT_INT_INFO
	for(i = 0; i < (PACKET_SIZE+7)/8; i++){
		elan_info("%02x %02x %02x %02x %02x %02x %02x %02x", buf[i*8+0],buf[i*8+1],buf[i*8+2],buf[i*8+3],buf[i*8+4],buf[i*8+5],buf[i*8+6],buf[i*8+7]);
	}
#endif
	
	if(FINGERS_PKT != buf[0]){
#ifndef PRINT_INT_INFO		
		printk("[elan] other event packet:%02x %02x %02x %02x %02x %02x %02x %02x\n", buf[0],buf[1],buf[2],buf[3],buf[4],buf[5],buf[6],buf[7]);
#endif
		elan_ts_handler_event(ts, buf);
		//return -1;
		return 0;
	}
#ifdef GESTURE_WAKEUP
			struct input_dev *idev = tpd->dev;
			if((user_flag==1)&&(buf[2]==0x2||buf[2]==0x5))
			{
				user_flag=0;
				/*//printk("[elan]:tp user ===========\n");
				input_report_key(idev,KEY_POWER,1);
				input_sync(idev);
				input_report_key(idev,KEY_POWER,0); 			  
				input_sync(idev);
				return 0;*/		
			}
#endif
	
#ifdef ELAN_ESD_CHECK
	atomic_set(&elan_cmd_response, 1);
#endif
	return 0;
}

static void elan_ts_report_data(struct elan_ts_data *ts, uint8_t *buf)
{
	uint16_t fbits=0;
	int reported = 0;
	uint8_t idx;
	int finger_num;
	int num = 0;
	uint16_t x = 0;
	uint16_t y = 0;
	uint16_t elan_pressure=0;
	uint16_t elan_width=0;
	int position = 0;
	uint8_t button_byte = 0;

	finger_num = FINGERS_NUM;	
#ifdef TWO_FINGERS
	num = buf[7] & 0x03; 
	fbits = buf[7] & 0x03;
	idx=1;
	button_byte = buf[PACKET_SIZE-1];
#endif
	
#ifdef FIVE_FINGERS
	num = buf[1] & 0x07; 
	fbits = buf[1] >>3;
	idx=2;
	button_byte = buf[PACKET_SIZE-1];
#endif
	
#ifdef TEN_FINGERS
	fbits = buf[2] & 0x30;	
	fbits = (fbits << 4) | buf[1];  
	num = buf[2] &0x0f;
	idx=3;
	button_byte = buf[PACKET_SIZE-1];
#endif

	if (num == 0){
		elan_ts_report_key(ts, button_byte);
	} 
	else{
		elan_info( "[elan] %d fingers", num);
		input_report_key(ts->input_dev, BTN_TOUCH, 1);
		for(position=0; position<finger_num;position++){
			if((fbits & 0x01)){
				elan_ts_parse_xy(&buf[idx],&x, &y);
				//printk("[elan]:x1=%d,y1=%d\n",x,y);
				//x = ts->x_resolution-x;
				//y = ts->y_resolution-y;
				x = x*LCM_X_MAX/X_RESOLUTION;
				y = y*LCM_Y_MAX/Y_RESOLUTION;
				x=LCM_X_MAX-x;
				y=LCM_Y_MAX-y;
								
				elan_pressure=(buf[position+45]);
				elan_width=(buf[position+35]);
				
				//printk("[elan]:x=%d,y=%d\n",x,y);
				if(x>0&&y>0)
				{
					input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, position);
					input_report_key(ts->input_dev, BTN_TOUCH, 1);
					input_report_abs(ts->input_dev, ABS_MT_PRESSURE, elan_pressure);
					input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, elan_width);
					//input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, elan_width);
					input_report_abs(ts->input_dev, ABS_MT_POSITION_X, x);
					input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, y);
					input_mt_sync(ts->input_dev);
					//printk("[elan]:===elan_pressure=%x,elan_width=%x\n",elan_pressure,elan_width);
					elan_info("Touch ID:%d, X:%d, Y:%d, W:%d down", id, x, y, w); 
					//elan_ts_touch_down(ts, position, x, y, 8);
				}
				reported++;
			}
			fbits = fbits >> 1;
			idx += 3;
		}
	}
	if (reported)
		input_sync(ts->input_dev);
	else 
	{
		input_mt_sync(ts->input_dev);
		input_sync(ts->input_dev);
	}	
	//input_sync(ts->input_dev);
	return;
}

 static void tpd_eint_interrupt_handler(void)
{
	elan_info("----------elan_ts_irq_handler----------");
	tpd_flag = 1;
	wake_up_interruptible(&waiter);
}

#if defined IAP_PORTION

static void check_update_flage(struct elan_ts_data *ts)
{
	int NEW_FW_VERSION = 0;
	int New_FW_ID = 0;
	int rc = 0;
	uint8_t write_flash_cmd[]={0x54,0xc0,0xe1,0x5a};
	uint8_t rek_cmd[]={0x54,0x29,0x00,0x01};
//elan add updata	
	
#if 1
	if((ts->fw_id==0x0D38)&&(FW_TEST_V==1))
	{
    		file_fw_data_master = file_fw_data_elan_master_hlt_black;
    		file_fw_data_slave=file_fw_data_elan_slave_hlt_black;
    	}
    	else if((ts->fw_id==0x0D38)&&(FW_TEST_V==0))
    	{
    		file_fw_data_master = file_fw_data_elan_master_hlt_white;
    		file_fw_data_slave=file_fw_data_elan_slave_hlt_white;
    	}
    	else if((ts->fw_id==0x0D39)&&(FW_TEST_V==1))
    	{
    		file_fw_data_master = file_fw_data_elan_master_dpt_black;
    		file_fw_data_slave=file_fw_data_elan_slave_dpt_black;
    	}
    	else if((ts->fw_id==0x0D39)&&(FW_TEST_V==0))
    	{
    		file_fw_data_master = file_fw_data_elan_master_dpt_white;
    		file_fw_data_slave=file_fw_data_elan_slave_dpt_white;
    	}
#endif

	if(ts->recover == 0x80){
	    file_fw_data_master = file_fw_data_elan_master_hlt_black;
    	    file_fw_data_slave=file_fw_data_elan_slave_hlt_black;
	    //printk("[elan] ***fw is miss, force update!!!!***\n");
	    goto update_elan_fw;    
    }
//end  	
	
#ifdef ELAN_2K_XX
#if 0
	New_FW_ID = file_fw_data[0x7DB3]<<8  | file_fw_data[0x7DB2];        
	NEW_FW_VERSION = file_fw_data[0x7DB1]<<8  | file_fw_data[0x7DB0];
#else
	New_FW_ID = file_fw_data[0x7BD3]<<8 | file_fw_data[0x7BD2];                 
	NEW_FW_VERSION = file_fw_data[0x7BD1]<<8 | file_fw_data[0x7BD0];
#endif
#endif

#ifdef ELAN_3K_XX
	New_FW_ID  = file_fw_data_master[0x7cdf]<<8  | file_fw_data_master[0x7cde];
	NEW_FW_VERSION = file_fw_data_slave[0x7abb]<<8  | file_fw_data_slave[0x7aba];
#endif
	
	printk("[elan] FW_ID=0x%x, New_FW_ID=0x%x \n",ts->fw_id, New_FW_ID);
	printk("[elan] FW_VERSION=0x%x,New_FW_VER=0x%x \n",ts->fw_ver,NEW_FW_VERSION);
	
#if 1
	if((ts->fw_id) != (New_FW_ID)){
		printk("[elan] ***fw id is different, can not update !***\n");
		//goto no_update_elan_fw;
	}
	else{
		printk("[elan] fw id is same !\n");
	}
	
	if((ts->fw_ver)>=(NEW_FW_VERSION)){
		printk("[elan] fw version is newest!!\n");
		goto no_update_elan_fw;
	}
#endif	
update_elan_fw:
    update_fw_one(ts->client);
    msleep(500);
#if 1
    elan_switch_irq(0);
	rc = __fw_packet_handler(ts->client);
	if (rc < 0){
		printk("[elan error] %s, fw_packet_handler fail, rc = %d\n", __func__, rc);
	}
	elan_switch_irq(1);
#endif
	msleep(200);
	elan_ts_send_cmd(private_ts->client, write_flash_cmd, 4);	
	msleep(10);
	elan_ts_send_cmd(private_ts->client, rek_cmd, 4);
	
no_update_elan_fw:
	printk("[elan] %s, fw check end.!!!!!!!!!!.............\n", __func__);
            	
	return;
}
#endif

static int touch_event_handler(void *unused)
{
	uint8_t buf[64] = {0};	
	int rc = 0;
	struct elan_ts_data *ts = private_ts;
	struct sched_param param = { .sched_priority = RTPM_PRIO_TPD};
	sched_setscheduler(current, SCHED_RR, &param);
	
	do{
		set_current_state(TASK_INTERRUPTIBLE);
		wait_event_interruptible(waiter, tpd_flag != 0);
		tpd_flag = 0;
		set_current_state(TASK_RUNNING);
		
		rc = elan_ts_recv_data(ts, buf);
		if(rc < 0){
			continue;
		}
#ifdef IRQ_CMD_HANDLER
		switch (buf[0]) {
		case CMD_HEADER_HELLO:
			elan_array_printk("CMD_HEADER_HELLO", buf, rx_size);
			memcpy(cmd_resp, buf, rx_size);
			complete(&irq_completion);
			break;
		case CMD_HEADER_5B_RESP:
		case CMD_HEADER_RESP:
			elan_array_printk("CMD_HEADER_RESP", buf, rx_size);
			memcpy(cmd_resp, buf, rx_size);
			complete(&irq_completion);
			break;

		case CMD_HEADER_RAWDATA:
			elan_array_printk("CMD_HEADER_RESP", buf, rx_size);
			memcpy(rawdata_buf+63*rawdata_offset, buf, rx_size);
			rawdata_offset++;
			//elan_array_printk("CMD_HEADER_RAWDATA", buf, rx_size);
			if(rawdata_offset >= pagereceive)
			{
				complete(&irq_completion);
				rawdata_offset=0;
				printk("finished!!\n");
			}

			//return IRQ_HANDLED;
			break;

		case CMD_HEADER_REK:
			if(!memcmp(buf, cal_packet, rx_size)) {
				memcpy(cmd_resp, buf, rx_size);
				elan_array_printk("CMD_HEADER_REK", buf, rx_size);
				complete(&irq_completion);
			}
			break;
		case CHD_HEADER_TRACE_TABLE:
			elan_array_printk("CHD_HEADER_TRACE_TABLE", buf, rx_size);

			// Copy 63-byte buffer to rawdata_buf[63*rawdata_offset]
			memcpy(&rawdata_buf[63*rawdata_offset], buf, rx_size);
			rawdata_offset++;

			// Set complete flag after receiving #pagereceve pages
			if(rawdata_offset >= pagereceive)
			{
				complete(&irq_completion);
				rawdata_offset=0;
				printk("finished!!\n");
			}
			// Page Process Done
			break;
			//return IRQ_HANDLED;
		case MTK_FINGERS_PKT:
		case TWO_FINGERS_PKT:
		case FIVE_FINGERS_PKT:
		case TEN_FINGERS_PKT:
			elan_ts_report_data(ts, buf);
			break;
		default:
			elan_array_printk("unknow packet", buf, rx_size);
			break;
	}

#endif
		//elan_ts_report_data(ts, buf);
	}while(!kthread_should_stop());

    return 0;
}

static void elan_ic_init_work(struct work_struct *work)
{
    int rc = 0;
	int retry_cnt = 0;
	
	if(private_ts->recover == 0){
		elan_switch_irq(0);
        for(retry_cnt=0; retry_cnt<3; retry_cnt++){		
    		rc = __fw_packet_handler(private_ts->client);
    		if (rc < 0){
    			printk("[elan error] %s, fw_packet_handler fail, rc = %d\n", __func__, rc);
    		}
    		else{
    		    break;    
    	    }
    	}	
		elan_switch_irq(1);
	}
#if defined IAP_PORTION	
	check_update_flage(private_ts);
#endif

#ifdef ELAN_ESD_CHECK
	INIT_DELAYED_WORK(&esd_work, elan_touch_esd_func);
	esd_wq = create_singlethread_workqueue("esd_wq");	
	if (!esd_wq) {
		return -ENOMEM;
	}
	queue_delayed_work(esd_wq, &esd_work, delay);
#endif        		
} 

static ssize_t acer_wakeup_read_proc(struct file *file, char __user *buf, size_t size, loff_t *ppos)
{
	printk("---read_proc---\n");
	int len = 0;
	len = sprintf(buf, "%s", mProcData);
	return len;
}

static ssize_t acer_wakeup_write_proc(struct file *file, const char __user *buf, size_t count, loff_t *data)
{
	printk("----write_proc count = %d----\n", count);
	if (count == 8) {
		memset(mProcData, 0, sizeof(mProcData));
		sprintf(mProcData, "%s", (char *) buf);

		if (simple_strtoul(mProcData, NULL, 10) > 0) {
			mIsEnableGestureWakeUp = true;
		} else {
			mIsEnableGestureWakeUp = false; 
		}
		
		if(mProcData[0] == '0'){
			mIsEnabletwofinger = false;
		}else{
			mIsEnabletwofinger = true;
		}
		
		if(mProcData[1] == '0'){
			mIsEnablefivefinger = false;
		}else{
			mIsEnablefivefinger = true;
		}
		
		if(mProcData[2] == '0'){
			mIsEnableDoubleTab = false;
		}else{
			mIsEnableDoubleTab = true;
		}
		
		if(mProcData[3] == '0'){
			mIsEnableVirtualHomeKey = false;
		}else{
			mIsEnableVirtualHomeKey = true;
		}
		
		if(mProcData[5] == '0'){
			mIsEnableSliderPoweronoff = false;
		}else{
			mIsEnableSliderPoweronoff = true;
		}
		
		if(mProcData[6] == '0'){
			mIsEnableSmartCover = false;
		}else{
			mIsEnableSmartCover = true;
		}
		printk("--- write_proc mIsEnabletwofinger = %d\n",mIsEnabletwofinger);
		printk("--- write_proc mIsEnablefivefinger = %d\n",mIsEnablefivefinger);
		printk("--- write_proc mIsEnableDoubleTab = %d\n",mIsEnableDoubleTab);
		printk("--- write_proc mIsEnableVirtualHomeKey = %d\n",mIsEnableVirtualHomeKey);
		printk("--- write_proc mIsEnableSliderPoweronoff = %d\n",mIsEnableSliderPoweronoff);
		printk("--- write_proc mIsEnableSmartCover = %d\n",mIsEnableSmartCover);
	}
	return 1;
}

static const struct file_operations acer_wakeup_fops = { 
    .read = acer_wakeup_read_proc,
	.write = acer_wakeup_write_proc,
};

void create_new_proc_entry(void)
{
	mProc_dir_entry = proc_create(GESTURE_PROC_NAME, 0777, NULL, &acer_wakeup_fops);
	if (!mProc_dir_entry)
	{
		printk("-----create proc fail------\n");
		return;
	}
	memset(mProcData, 0, sizeof(mProcData));
	sprintf(mProcData, "%s", "00000");
	return 0;
}

int proc_init(void)
{
	create_new_proc_entry();
	return 0;
}

void proc_cleanup(void)
{
	remove_proc_entry(GESTURE_PROC_NAME, NULL);
}

static int elan_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int retval = 0;
	struct elan_ts_data *ts;
	int err_fw = 0 ; // lgx
	printk("[elan] %s enter i2c addr %x\n", __func__, client->addr);
	// lgx
	uint8_t cmd[]			= {0x53, 0x00, 0x00, 0x01};/* Get Firmware Version*/
	uint8_t buf_recv[4] 	= {0};
	int rc;
	
	int major, minor;
	/*power on, need confirm with SA*/
	mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
	mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);
	msleep(5);
//	DBG(" fts ic reset\n");
//	DBG("wangcq327 --- %d\n",TPD_POWER_SOURCE_CUSTOM);	
#ifdef TPD_POWER_SOURCE_CUSTOM
	hwPowerOn(MT6323_POWER_LDO_VGP2, VOL_2800, "TP");
#else
	hwPowerOn(MT6323_POWER_LDO_VGP1, VOL_3300, "TP");
#endif

#if defined MT6575	
	hwPowerOn(MT65XX_POWER_LDO_VGP2, VOL_2800, "TP");
	hwPowerOn(MT65XX_POWER_LDO_VGP, VOL_1800, "TP");
#endif

#ifdef MT6589
	hwPowerOn(MT65XX_POWER_LDO_VGP4, VOL_2800, "touch");
    hwPowerOn(MT65XX_POWER_LDO_VGP6, VOL_1800, "touch");
#endif

#if defined MT6592 || defined MT6582
	hwPowerOn(MT6323_POWER_LDO_VGP1, VOL_2800, "TP");
#endif
	msleep(10);
	
	/*set INT pin--->direction is input, fuction is gpio, pull up*/
	mt_set_gpio_mode(GPIO_CTP_EINT_PIN, GPIO_CTP_EINT_PIN_M_GPIO);
	mt_set_gpio_dir(GPIO_CTP_EINT_PIN, GPIO_DIR_IN);
	mt_set_gpio_pull_enable(GPIO_CTP_EINT_PIN, GPIO_PULL_ENABLE);
	mt_set_gpio_pull_select(GPIO_CTP_EINT_PIN, GPIO_PULL_UP);

	/*set reset pin--->direction is output, normal level is high*/
	mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
	mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
/*
mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
msleep(20);
mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);
msleep(20);
mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
*/

#ifdef ELAN_I2C_DMA_MOD
	client->addr = client->addr & I2C_MASK_FLAG | I2C_DMA_FLAG | I2C_ENEXT_FLAG;
	gpDMABuf_va = (u8 *)dma_alloc_coherent(NULL, 4096, &gpDMABuf_pa, GFP_KERNEL);
    if(!gpDMABuf_va){
		printk("[elan] Allocate DMA I2C Buffer failed\n");
		return -ENOMEM;
    }
#endif
	
	/*elan IC init here*/
	retval = elan_ts_setup(client);
	if (retval < 0) {
		printk("[elan error]: %s No Elan chip inside, return now\n", __func__);
		return -ENODEV;
	}
	
	/*elan struct elan_ts_data init*/
	ts = kzalloc(sizeof(struct elan_ts_data), GFP_KERNEL);
	if (ts == NULL) {
		printk("[elan error] %s: allocate elan_ts_data failed\n", __func__);
		return -ENOMEM;
	}
	
	ts->recover = retval;
	client->timing = 100;
	ts->client = client;
	i2c_set_clientdata(client, ts);
	private_ts = ts;
	input_set_abs_params(tpd->dev,ABS_MT_PRESSURE,0,255,0,0);
	ts->input_dev = tpd->dev;
	
	elan_touch_node_init();
	
	INIT_DELAYED_WORK(&init_work, elan_ic_init_work);
	init_elan_ic_wq = create_singlethread_workqueue("init_elan_ic_wq");	
	if (!init_elan_ic_wq) {
		return -ENOMEM;
	}
	queue_delayed_work(init_elan_ic_wq, &init_work, delay);
    
	ts->work_thread = kthread_run(touch_event_handler, 0, ELAN_TS_NAME);
	if(IS_ERR(ts->work_thread)) {
		retval = PTR_ERR(ts->work_thread);
		printk("[elan error] failed to create kernel thread: %d\n", retval);
		return -EINVAL;
	}
	set_bit(KEY_POWER,tpd->dev->keybit);
	/*elan interrupt handler registration*/
	//mt_set_gpio_mode(GPIO_CTP_EINT_PIN, GPIO_CTP_EINT_PIN_M_EINT);
	msleep(20);

#if defined MT6592 || defined MT6582
	mt_eint_registration(CUST_EINT_TOUCH_PANEL_NUM, EINTF_TRIGGER_FALLING, tpd_eint_interrupt_handler, 1);
	mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
#else	
	//mt65xx_eint_set_sens(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_SENSITIVE);
	//mt65xx_eint_set_hw_debounce(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_DEBOUNCE_CN);
	//mt65xx_eint_registration(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_DEBOUNCE_EN, CUST_EINT_TOUCH_PANEL_POLARITY, tpd_eint_interrupt_handler, 1);
	//mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
	mt_eint_registration(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_TYPE, tpd_eint_interrupt_handler, 1); 
	mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
#endif	
	tpd_load_status = 1;
	
	if (FACTORY_BOOT == get_boot_mode()|| RECOVERY_BOOT == get_boot_mode()){
		boot_normal_flag = 0;
	}
	printk("[elan]+++++++++===================+++++++++!\n");
#ifdef IRQ_CMD_HANDLER
	init_completion(&irq_completion);
	/* register sysfs */
	if (sysfs_create_group(&client->dev.kobj, &elan_attribute_group))
	dev_err(&client->dev, "sysfs create group error\n");
#endif
	printk("[elan]+++++++++end porbe+++++++++!\n");
		
	return 0;
}

static int elan_ts_remove(struct i2c_client *client)
{
	printk("[elan] elan_ts_remove\n");
#ifdef ELAN_I2C_DMA_MOD		
	if(gpDMABuf_va){
		dma_free_coherent(NULL, 4096, gpDMABuf_va, gpDMABuf_pa);
		gpDMABuf_va = NULL;
		gpDMABuf_pa = NULL;
	}
#endif
	
	return 0;
}

static struct i2c_driver elan_ts_driver = {
	.probe = elan_ts_probe,
	.remove = elan_ts_remove,
	.id_table = elan_ts_id,
	.driver	= {
		.name = ELAN_TS_NAME,
	},
	.detect = tpd_detect,
#ifndef I2C_BORAD_REGISTER 	
	.address_data = &addr_data,
#endif	
};


int tpd_local_init(void) 
{
	if(i2c_add_driver(&elan_ts_driver)!=0) {
		TPD_DMESG("[elan error] unable to add i2c driver.\n");
		return -1;
	}

	if(tpd_load_status == 0){
		TPD_DMESG("[elan error] add error touch panel driver.\n");
		i2c_del_driver(&elan_ts_driver);
		return -1;
	}
	input_set_abs_params(tpd->dev, ABS_MT_TRACKING_ID, 0, 255, 0, 0);
#ifdef TPD_HAVE_BUTTON     
	tpd_button_setting(TPD_KEY_COUNT, tpd_keys_local, tpd_keys_dim_local);// initialize tpd button data
#endif   

#if (defined(TPD_WARP_START) && defined(TPD_WARP_END))    
	TPD_DO_WARP = 1;
	memcpy(tpd_wb_start, tpd_wb_start_local, TPD_WARP_CNT*4);
	memcpy(tpd_wb_end, tpd_wb_start_local, TPD_WARP_CNT*4);
#endif 

#if (defined(TPD_HAVE_CALIBRATION) && !defined(TPD_CUSTOM_CALIBRATION))
	memcpy(tpd_calmat, tpd_def_calmat_local, 8*4);
	memcpy(tpd_def_calmat, tpd_def_calmat_local, 8*4);	
#endif  
	TPD_DMESG("[elan] end %s, %d\n", __FUNCTION__, __LINE__);  
	tpd_type_cap = 1;
	
	return 0;
}

 static int tpd_suspend(struct i2c_client *client, pm_message_t message)
{
	struct elan_ts_data *ts = private_ts;
	int rc = 0;

	printk( "[elan]  %s: enter\n", __func__);
	if(ts->power_lock==0){
		//elan_switch_irq(0);
#ifdef GESTURE_WAKEUP
		if (!mIsEnableGestureWakeUp)
		{ 
			// Gesture WakeUp off
			rc = elan_ts_set_power_state(ts->client, PWR_STATE_DEEP_SLEEP);
			tpd_is_suspend = 1;
		}
		else
		{
			uint8_t cmd[] = {CMD_W_PKT, 0x58, 0x0f, 0x01};
			if (elan_ts_send_cmd(private_ts->client, cmd, 4) != 4)
			{
				return -EINVAL;
			}	
			user_flag=1;
		}
#else
		rc = elan_ts_set_power_state(ts->client, PWR_STATE_DEEP_SLEEP);
		tpd_is_suspend = 1;
#endif
	}
#ifdef ELAN_ESD_CHECK	
	cancel_delayed_work_sync(&esd_work);
#endif	
	return rc;
}

static int tpd_resume(struct i2c_client *client)
{
	struct elan_ts_data *ts = private_ts;
	int rc = 0;
	user_flag=0;
	printk("[elan] %s: enter\n", __func__);
	if(ts->power_lock==0){
		printk("[elan] reset gpio to resum tp\n");		
		elan_reset();
		//elan_switch_irq(1);
		tpd_is_suspend = 0;
	}
#ifdef ELAN_ESD_CHECK
	queue_delayed_work(esd_wq, &esd_work, delay);	
#endif

    elan_ts_touch_up(ts, 0, 0, 0);
    
	return rc;
}

static struct tpd_driver_t tpd_device_driver = {
	.tpd_device_name = "elan_ts",
	.tpd_local_init = tpd_local_init,
	.suspend = tpd_suspend,
	.resume = tpd_resume,
#ifdef TPD_HAVE_BUTTON
	.tpd_have_button = 1,
#else
	.tpd_have_button = 0,
#endif
};

static int __init tpd_driver_init(void) 
{
	printk("[elan] MediaTek elan touch panel driver init\n");
#ifdef I2C_BORAD_REGISTER 	
	i2c_register_board_info(0, &i2c_tpd, 1);
#endif	
	if(tpd_driver_add(&tpd_device_driver) < 0){
		TPD_DMESG("[elan error] add generic driver failed\n");
	}
	proc_init();
	return 0;
}

static void __exit tpd_driver_exit(void) 
{
	TPD_DMESG("[elan] MediaTek elan touch panel driver exit\n");
	tpd_driver_remove(&tpd_device_driver);
	proc_cleanup();
}

module_init(tpd_driver_init);
module_exit(tpd_driver_exit);
