#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
#include <linux/xlog.h>
//#include <linux/kernel.h>

#include "kd_camera_hw.h"

#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_camera_feature.h"
#include <mach/camera_isp.h>
#include <mach/mt_pm_ldo.h>

extern void ISP_MCLK1_EN(BOOL En);

/******************************************************************************
 * Debug configuration
******************************************************************************/
#define PFX "[kd_camera_hw]"
#define PK_DBG_NONE(fmt, arg...)    do {} while (0)
#define PK_DBG_FUNC(fmt, arg...)    xlog_printk(ANDROID_LOG_INFO, PFX , fmt, ##arg)

#define DEBUG_CAMERA_HW_K
#ifdef DEBUG_CAMERA_HW_K
#define PK_DBG PK_DBG_FUNC
#define PK_ERR(fmt, arg...)         xlog_printk(ANDROID_LOG_ERR, PFX , fmt, ##arg)
#define PK_XLOG_INFO(fmt, args...) \
                do {    \
                   xlog_printk(ANDROID_LOG_INFO, PFX , fmt, ##arg); \
                } while(0)
#else
#define PK_DBG(a,...)
#define PK_ERR(a,...)
#define PK_XLOG_INFO(fmt, args...)
#endif



int kdCISModulePowerOn(CAMERA_DUAL_CAMERA_SENSOR_ENUM SensorIdx, char *currSensorName, BOOL On, char* mode_name)
{
#if !defined (MTK_ALPS_BOX_SUPPORT)

u32 pinSetIdx = 0;//default main sensor

#define IDX_PS_CMRST 0
#define IDX_PS_CMPDN 4

#define IDX_PS_MODE 1
#define IDX_PS_ON   2
#define IDX_PS_OFF  3


u32 pinSet[2][8] = {
                    //for main sensor
                    {GPIO_CAMERA_CMRST_PIN,
                        GPIO_CAMERA_CMRST_PIN_M_GPIO,   /* mode */
                        GPIO_OUT_ONE,                   /* ON state */
                        GPIO_OUT_ZERO,                  /* OFF state */
                     GPIO_CAMERA_CMPDN_PIN,
                        GPIO_CAMERA_CMPDN_PIN_M_GPIO,
                        GPIO_OUT_ONE,
                        GPIO_OUT_ZERO,
                    },
                    //for sub sensor
                    {GPIO_CAMERA_CMRST1_PIN,
                     GPIO_CAMERA_CMRST1_PIN_M_GPIO,
                        GPIO_OUT_ONE,
                        GPIO_OUT_ZERO,
                     GPIO_CAMERA_CMPDN1_PIN,
                        GPIO_CAMERA_CMPDN1_PIN_M_GPIO,
                        GPIO_OUT_ONE,
                        GPIO_OUT_ZERO,
                    },
                   };







    if (DUAL_CAMERA_MAIN_SENSOR == SensorIdx){
        pinSetIdx = 0;
    }
    else if (DUAL_CAMERA_SUB_SENSOR == SensorIdx) {
        pinSetIdx = 1;
    }

   
    //power ON
    if (On) {

			 
		PK_DBG("lgx 000 kdCISModulePowerOn -on:currSensorName=%s;\n",currSensorName);
		PK_DBG("kdCISModulePowerOn -on:pinSetIdx=%d\n",pinSetIdx);
		ISP_MCLK1_EN(FALSE);//disable clk by lupingzhong
		 if ((pinSetIdx==0) && currSensorName &&( (0 == strcmp(SENSOR_DRVNAME_HI545_MIPI_RAW, currSensorName))||(0 == strcmp(SENSOR_DRVNAME_HI545_QH_MIPI_RAW, currSensorName))))
		 {
		 #if 1
		 
		 PK_DBG(" lgx eve001 [hi545mipiraw_poweron]start...,pinSetIdx:%d \n",pinSetIdx);
			 int ret;
			 //(1) Set power pin and reset pin (spec: pwdn:low, reset:low)
			 if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN]) {	  
				 if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE])){
					 PK_DBG("[hi545mipiraw] set gpio mode failed!! \n");
				 }	   
				 if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_DIR_OUT)){
					 PK_DBG("[hi545mipiraw] set gpio dir failed!! \n");
				 }		   
				 if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_OFF])){
					 PK_DBG("[hi545mipiraw] set gpio failed!! \n");
				 }	 
			 }
			 
			 if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) {
				 if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){
					 PK_DBG("[hi545mipiraw] set gpio mode failed!! \n");
				 }	 
				 if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT)){
					 PK_DBG("[hi545mipiraw] set gpio dir failed!! \n");
				 }		 
				 if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_OFF])){
					 PK_DBG("[hi545mipiraw] set gpio failed!! \n");
				 }		 
			 }
			 
		 
			 //(2) Power on VCAM_IO, VCAMA, DVDD
			 //IOVDD
			 ret = hwPowerOn(CAMERA_POWER_VCAM_D2, VOL_1800,mode_name);
			 if(TRUE != ret){ 
				 PK_DBG("[hi545mipiraw] Fail to enable digital power\n");
				// goto _kd_hi545_PowerOn_exit_;
			 }
			 udelay(3);//delay >=0us in spec 
			 
			 //AVDD
			 ret = hwPowerOn(CAMERA_POWER_VCAM_A, VOL_2800,mode_name);
			 if(TRUE != ret){ 
				 PK_DBG("[hi545mipiraw] Fail to enable analog power\n");
				// goto _kd_hi545_PowerOn_exit_;
			 }
			 udelay(5);//delay >=0us in spec 
			 
			 printk("[hi545mipiraw] Fail VOL_1200 be\n");
		 			 ret = hwPowerOn(CAMERA_POWER_VCAM_D, VOL_1200,mode_name);
			 if(TRUE != ret){ 
				printk("[hi545mipiraw] Fail VOL_1200\n");
				// goto _kd_hi545_PowerOn_exit_;
			 }
			 
			// printk("[hi545mipiraw] Fail VOL_1200 end\n");
			 udelay(5);//delay >=0us in spec 
			   
		 	 ret = hwPowerOn(CAMERA_POWER_VCAM_A2, VOL_2800,mode_name);
			 if(TRUE != ret){ 
				printk("[hi545mipiraw] af Fail VOL_2800\n");
				// goto _kd_hi545_PowerOn_exit_;
			 }
			 
			
			 udelay(5);//delay >=0us in spec 
			   
			 //(3) Set power pin and reset pin (spec: pwdn:high, reset:high)
		 
			 if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN]) { 
				 if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE])){
					 PK_DBG("[hi545mipiraw] set gpio mode failed!! \n");
				 }
				 if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_DIR_OUT)){
					 PK_DBG("[hi545mipiraw] set gpio dir failed!! \n");
				 }
				 if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_ON])){
					 PK_DBG("[hi545mipiraw] set gpio failed!! \n");
				 }
			 }
			 
			
			 mdelay(1);
			 ISP_MCLK1_EN(TRUE);//disable clk by lupingzhong
			  mdelay(10);//delay >=10ms in spec
			 if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]){
				 //RST pin
				 if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){
					 PK_DBG("[hi545mipiraw] set gpio mode failed!! \n");
				 }
				 if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT)){
					 PK_DBG("[hi545mipiraw] set gpio dir failed!! \n");
				 }
				 if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_ON])){
					 PK_DBG("[hi545mipiraw] set gpio failed!! \n");
				 }
			 }


			 
			 PK_DBG(" lgx eve002 [hi545mipiraw_poweron]start...,pinSetIdx:%d \n",pinSetIdx);
		 #endif

		 }
		else if (currSensorName && (0 == strcmp(SENSOR_DRVNAME_GC0310_MIPI_YUV, currSensorName)))
   		 {
#if 1
   		 PK_DBG("[ON_general 1.8V]sensorIdx:%d lgx 001 \n",SensorIdx);
        //enable active sensor
        if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) {
            //RST pin,active low
            if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
            if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
            if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
            mdelay(1);

            
        }

        //DOVDD
        PK_DBG("[ON_general 1.8V]sensorIdx:%d \n",SensorIdx);
        if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D2, VOL_1800,mode_name))
        {
            PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
            //return -EIO;
            //goto _kdCISModulePowerOn_exit_;
        }
        mdelay(1);

        //AVDD
        if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_A, VOL_2800,mode_name))
        {
            PK_DBG("[CAMERA SENSOR] Fail to enable analog power\n");
            //return -EIO;
            //goto _kdCISModulePowerOn_exit_;
        }
        mdelay(1);    
        //PDN pin,high
            if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
            if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
            if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_OFF])){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
			mdelay(1);
			
			ISP_MCLK1_EN(TRUE);//enable clk by lupingzhong
		if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) {
			//RST pin
			if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
			if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
			if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_ON])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
			mdelay(1);
		
			//PDN pin
			if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
			if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
			if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_ON])){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
			mdelay(1);
			if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_OFF])){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
			mdelay(1);
		}


	 //disable inactive sensor
	 if(pinSetIdx == 0 || pinSetIdx == 2) {//disable sub
		 if (GPIO_CAMERA_INVALID != pinSet[1][IDX_PS_CMRST]) {
			 if(mt_set_gpio_mode(pinSet[1][IDX_PS_CMRST],pinSet[1][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
			 if(mt_set_gpio_mode(pinSet[1][IDX_PS_CMPDN],pinSet[1][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
			 if(mt_set_gpio_dir(pinSet[1][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
			 if(mt_set_gpio_dir(pinSet[1][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
			 if(mt_set_gpio_out(pinSet[1][IDX_PS_CMRST],pinSet[1][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");} //low == reset sensor
			 if(mt_set_gpio_out(pinSet[1][IDX_PS_CMPDN],pinSet[1][IDX_PS_CMPDN+IDX_PS_ON])){PK_DBG("[CAMERA LENS] set gpio failed!! \n");} //high == power down lens module
		 }
	 }
	 else {
		 if (GPIO_CAMERA_INVALID != pinSet[0][IDX_PS_CMRST]) {
			 if(mt_set_gpio_mode(pinSet[0][IDX_PS_CMRST],pinSet[0][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
			 if(mt_set_gpio_mode(pinSet[0][IDX_PS_CMPDN],pinSet[0][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
			 if(mt_set_gpio_dir(pinSet[0][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
			 if(mt_set_gpio_dir(pinSet[0][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
			 if(mt_set_gpio_out(pinSet[0][IDX_PS_CMRST],pinSet[0][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");} //low == reset sensor
			 if(mt_set_gpio_out(pinSet[0][IDX_PS_CMPDN],pinSet[0][IDX_PS_CMPDN+IDX_PS_OFF])){PK_DBG("[CAMERA LENS] set gpio failed!! \n");} //high == power down lens module
		 }
	 }

#endif
	 }
		////niyangadd0528
		else if ( currSensorName &&(0 == strcmp(SENSOR_DRVNAME_GC2355_MIPI_RAW, currSensorName)))
				 {PK_DBG("[gc2355mipiraw_poweron] start...,pinSetIdx:%d \n",pinSetIdx);

				 if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) {
						 //PDN pin
						 if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
						 if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
						 if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_ON])){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
						 mdelay(1);
						 
						 //RST pin
						 if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
						 if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
						 if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
						 mdelay(1);
					 }
					 //DOVDD
					 PK_DBG("[ON_general 1.8V]2355sensorIdx:%d \n",SensorIdx);
					 if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D2, VOL_1800,mode_name))
					 {
						 PK_DBG("[CAMERA SENSOR] Fail to enable D2 power\n");
						 //return -EIO;
						 //goto _kdCISModulePowerOn_exit_;
					 }
					 mdelay(1);
				 
					 //DVDD
					 if(TRUE != hwPowerOn(MT6323_POWER_LDO_VGP1, VOL_1800,mode_name))
					 {
						  PK_DBG("[CAMERA SENSOR] Fail to enable D power\n");
						  //return -EIO;
						  //goto _kdCISModulePowerOn_exit_;
					 }
					 mdelay(1);
					 
					 //AVDD
					 if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_A, VOL_2800,mode_name))
					 {
						 PK_DBG("[CAMERA SENSOR] Fail to enable A power\n");
						 //return -EIO;
						 //goto _kdCISModulePowerOn_exit_;
					 }
					 mdelay(1);

						 ISP_MCLK1_EN(TRUE);//enable clk by lupingzhong
 mdelay(1);
					 //enable active sensor
					 if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) {
						 //PDN pin
						 if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
						 if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
						 if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_OFF])){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
						 mdelay(1);
						 
						 //RST pin
						 if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
						 if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
						 if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_ON])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
						 mdelay(1);
					 }
		
			PK_DBG("[gc2355mipiraw_poweron] done");
				
				}

		////
    	}
    else {//power OFF

if(currSensorName && (0 == strcmp(SENSOR_DRVNAME_GC0310_MIPI_YUV,currSensorName)))
{
#if 1
int ret;
if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]){
	 if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){
		PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");
	 }
	 if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT)){
		PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");
	 }		  
	 if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_OFF])){
		PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");
	 } 
}

if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN]) {
	if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE])){
		PK_DBG("[+++haley+++] set gpio mode failed!! \n");
	}
	if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_DIR_OUT)){
		PK_DBG("[+++haley+++] set gpio dir failed!! \n");
	}
	if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_OFF])){
		PK_DBG("[+++haley+++] set gpio failed!! \n");
	}
}

mdelay(1);
#if 1
if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN]) {
	if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE])){
		PK_DBG("[+++haley+++] set gpio mode failed!! \n");
	}
	if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_DIR_OUT)){
		PK_DBG("[+++haley+++] set gpio dir failed!! \n");
	}
	if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_ON])){
		PK_DBG("[+++haley+++] set gpio failed!! \n");
	}
}
#endif
mdelay(10);

ISP_MCLK1_EN(FALSE);//disable clk by lupingzhong
mdelay(1);
udelay(5);

			//v1.2
 ret = hwPowerDown(CAMERA_POWER_VCAM_D,mode_name);
 if(TRUE != ret)  {
    PK_DBG("[+++haley+++] Fail to OFF analog power : VCAM_A\n");
 }
 udelay(5);
 PK_DBG("+++haley+++,CAMERA_POWER_VCAM_D!\n");

//v2.8
 ret = hwPowerDown(CAMERA_POWER_VCAM_A,mode_name);
 if(TRUE != ret)  {
    PK_DBG("[+++haley+++] Fail to OFF analog power : VCAM_A\n");
 }
 udelay(5);//delay >=0us in spec
 
 PK_DBG("+++haley+++,CAMERA_POWER_VCAM_A!\n");
 //v1.8
 ret = hwPowerDown(CAMERA_POWER_VCAM_D2,mode_name);
 if(TRUE != ret) {
    PK_DBG("[+++haley+++] Fail to OFF digital power: VCAM_D2\n");
 }
 PK_DBG("+++haley+++,CAMERA_POWER_VCAM_D2!\n");
 udelay(5);//delay >=0us in spec


#endif
}
else if(currSensorName && ((0 == strcmp(SENSOR_DRVNAME_HI545_MIPI_RAW,currSensorName))||(0 == strcmp(SENSOR_DRVNAME_HI545_QH_MIPI_RAW,currSensorName))))
{
#if 1
	int ret;
	if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]){
		 if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){
			PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");
		 }
		 if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT)){
			PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");
		 }		  
		 if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_OFF])){
			PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");
		 } 
	}
	

	
	mdelay(15);
	ISP_MCLK1_EN(FALSE);//disable clk by lupingzhong
	mdelay(1);
		if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN]) {
		if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE])){
			PK_DBG("[+++haley+++] set gpio mode failed!! \n");
		}
		if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_DIR_OUT)){
			PK_DBG("[+++haley+++] set gpio dir failed!! \n");
		}
		if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_OFF])){
			PK_DBG("[+++haley+++] set gpio failed!! \n");
		}
	}
	
	udelay(5);
	 ret = hwPowerDown(CAMERA_POWER_VCAM_A2, mode_name);
	 if(TRUE != ret)  {
		PK_DBG("[+++haley+++] Fail to OFF analog power : VCAM_A2\n");
	 }
				//v1.2
	 ret = hwPowerDown(CAMERA_POWER_VCAM_D,mode_name);
	 if(TRUE != ret)  {
		PK_DBG("[+++haley+++] Fail to OFF analog power : VCAM_A\n");
	 }
	 udelay(5);
	 PK_DBG("+++haley+++,CAMERA_POWER_VCAM_D!\n");
	
	//v2.8
	 ret = hwPowerDown(CAMERA_POWER_VCAM_A,mode_name);
	 if(TRUE != ret)  {
		PK_DBG("[+++haley+++] Fail to OFF analog power : VCAM_A\n");
	 }
	 udelay(5);//delay >=0us in spec
	 
	 PK_DBG("+++haley+++,CAMERA_POWER_VCAM_A!\n");
	 //v1.8
	 ret = hwPowerDown(CAMERA_POWER_VCAM_D2,mode_name);
	 if(TRUE != ret) {
		PK_DBG("[+++haley+++] Fail to OFF digital power: VCAM_D2\n");
	 }
	 PK_DBG("+++haley+++,CAMERA_POWER_VCAM_D2!\n");
	 udelay(5);//delay >=0us in spec
	
	if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN]) {
		if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE])){
			PK_DBG("[+++haley+++] set gpio mode failed!! \n");
		}
		if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_DIR_OUT)){
			PK_DBG("[+++haley+++] set gpio dir failed!! \n");
		}
		if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_OFF])){
			PK_DBG("[+++haley+++] set gpio failed!! \n");
		}
	}
	
	
#endif

}
else if(currSensorName && (0 == strcmp(SENSOR_DRVNAME_GC2355_MIPI_RAW,currSensorName)))
{
#if 1
	int ret;
	if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN]) {
		if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE])){
			PK_DBG("[+++haley+++] set gpio mode failed!! \n");
		}
		if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_DIR_OUT)){
			PK_DBG("[+++haley+++] set gpio dir failed!! \n");
		}
		if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_ON])){
			PK_DBG("[+++haley+++] set gpio failed!! \n");
		}
	}
        mdelay(1);
       if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]){
		 if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){
			PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");
		 }
		 if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT)){
			PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");
		 }		  
		 if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_OFF])){
			PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");
		 } 
	}
	
	
	
	mdelay(1);
	ISP_MCLK1_EN(FALSE);//disable clk by lupingzhong
	mdelay(1);
		
	//v2.8
	 ret = hwPowerDown(CAMERA_POWER_VCAM_A,mode_name);
	 if(TRUE != ret)  {
		PK_DBG("[+++haley+++] Fail to OFF analog power : VCAM_A\n");
	 }
	 udelay(5);//delay >=0us in spec			//v1.2
	 
	 ret = hwPowerDown(MT6323_POWER_LDO_VGP1,mode_name);
	 if(TRUE != ret)  {
		PK_DBG("[+++haley+++] Fail to OFF analog power : VCAM_A\n");
	 }
	 udelay(5);
	 //v1.8
	 ret = hwPowerDown(CAMERA_POWER_VCAM_D2,mode_name);
	 if(TRUE != ret) {
		PK_DBG("[+++haley+++] Fail to OFF digital power: VCAM_D2\n");
	 }
	 PK_DBG("+++haley+++,2355CAMERA_POWER_VCAM_D2!\n");
	 udelay(5);//delay >=0us in spec
	#endif
    }//

	}
#endif /* end of defined MTK_ALPS_BOX_SUPPORT */

	return 0;

_kdCISModulePowerOn_exit_:
    return -EIO;
}

EXPORT_SYMBOL(kdCISModulePowerOn);


//!--
//




