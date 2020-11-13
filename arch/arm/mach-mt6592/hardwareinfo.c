/* Huaqin  Inc. (C) 2011. All rights reserved.
 *
 * BY OPENING THIS FILE, RECEIVER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
 * THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("HUAQIN SOFTWARE")
 * RECEIVED FROM HUAQIN AND/OR ITS REPRESENTATIVES ARE PROVIDED TO RECEIVER ON
 * AN "AS-IS" BASIS ONLY. HUAQIN EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
 * NEITHER DOES HUAQIN PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
 * SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
 * SUPPLIED WITH THE HUAQIN SOFTWARE, AND RECEIVER AGREES TO LOOK ONLY TO SUCH
 * THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. RECEIVER EXPRESSLY ACKNOWLEDGES
 * THAT IT IS RECEIVER'S SOLE RESPONSIBILITY TO OBTAIN FROM ANY THIRD PARTY ALL PROPER LICENSES
 * CONTAINED IN HUAQIN SOFTWARE. HUAQIN SHALL ALSO NOT BE RESPONSIBLE FOR ANY HUAQIN
 * SOFTWARE RELEASES MADE TO RECEIVER'S SPECIFICATION OR TO CONFORM TO A PARTICULAR
 * STANDARD OR OPEN FORUM. RECEIVER'S SOLE AND EXCLUSIVE REMEDY AND HUAQIN'S ENTIRE AND
 * CUMULATIVE LIABILITY WITH RESPECT TO THE HUAQIN SOFTWARE RELEASED HEREUNDER WILL BE,
 * AT HUAQIN'S OPTION, TO REVISE OR REPLACE THE HUAQIN SOFTWARE AT ISSUE,
 * OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY RECEIVER TO
 * HUAQIN FOR SUCH HUAQIN SOFTWARE AT ISSUE.
 *

 */


/*******************************************************************************
* Dependency
*******************************************************************************/

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/errno.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <asm/io.h>
#include <asm/uaccess.h>

#include <mach/mtk_nand.h>
#include <mach/mt_boot.h>
#include <mach/mt_gpio.h>
#include "cust_gpio_usage.h"




#include <linux/proc_fs.h>


//#include "lcm_drv.h"
//#include "tpd.h"
#include <mach/hardwareinfo.h>


#define HARDWARE_INFO_VERSION   0x7503



hardware_info_struct hardware_info;

/****************************************************************************** 
 * Function Configuration
******************************************************************************/
//extern LCM_PARAMS *_get_lcm_driver_by_handle(disp_lcm_handle *plcm);

/****************************************************************************** 
 * Debug configuration
******************************************************************************/


//hardware info driver
static ssize_t show_hardware_info(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value = 1;
    int count = 0;

    //show lcm driver 
    printk("show_lcm_info return lcm name\n");
    count += sprintf(buf, "LCM Name:   ");//12 bytes
    if(hardware_info.hq_lcm_name)       
        count += sprintf(buf+count, "%s\n", hardware_info.hq_lcm_name); 
    else
        count += sprintf(buf+count,"no lcd name found\n");


        count += sprintf(buf + count, "ctp name:   ");//12bytes
        if(hardware_info.hq_tpd_name != NULL)
        {
                count += sprintf(buf + count, "%s\n", hardware_info.hq_tpd_name);
                count += sprintf(buf + count, "ctp fw ver :%x\n", hardware_info.tpd_fw_version);
				count += sprintf(buf + count, "ctp color  :%x\n", hardware_info.tpd_color);
				count += sprintf(buf + count, "ctp module id  :%x\n", hardware_info.tpd_module_id);
        }
        else
        {
                count += sprintf(buf + count, "ctp driver NULL !\n");
        }
/*
    //show nand drvier name 
    printk("show nand device name\n");
    count += sprintf(buf+count, "NAND Name:  ");//12 bytes
    if(hardware_info.nand_device_name) 
        count += sprintf(buf+count, "%s\n", hardware_info.nand_device_name);
    else
        count += sprintf(buf+count,"no nand name found\n");    
*/

    //show Main Camera drvier name
    printk("show Main Camera device name\n");
    count += sprintf(buf+count, "Main Camera:");//12 bytes
    if(hardware_info.mainCameraName) 
        count += sprintf(buf+count, "%s\n", hardware_info.mainCameraName);

    //show Sub Camera drvier name
    printk("show Sub camera name\n");
    count += sprintf(buf+count, "Sub Camera: ");//12 bytes
    if(hardware_info.subCameraName) 
        count += sprintf(buf+count, "%s\n", hardware_info.subCameraName);

        
    //A20 using 6620
    //show WIFI
    printk("show WIFI name\n");
    count += sprintf(buf+count, "WIFI:       ");//12 bytes
    if(1) 
        count += sprintf(buf+count, "MT6625L\n");
    else
        count += sprintf(buf+count,"no WIFI name found\n");

    printk("show Bluetooth name\n");
    count += sprintf(buf+count, "Bluetooth:  ");//12 bytes
    if(1) 
        count += sprintf(buf+count, "MT6625L\n");
    else
        count += sprintf(buf+count,"no Bluetooth name found\n");

    printk("show GPS name\n");
    count += sprintf(buf+count, "GPS:        ");//12 bytes
    if(1) 
        count += sprintf(buf+count, "MT6625L\n");
    else
        count += sprintf(buf+count,"no GPS name found\n");

    printk("show FM name\n");
    count += sprintf(buf+count, "FM:         ");//12 bytes
    if(1) 
        count += sprintf(buf+count, "MT6625L\n");
    else
        count += sprintf(buf+count,"no FM name found\n");

    
    ret_value = count;

    return ret_value;
}
static ssize_t store_hardware_info(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    printk("[%s]: Not Support Write Function\n",__func__);      
    return size;
}


static ssize_t show_version(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value = 1;
        
    ret_value = sprintf(buf, "Version:    :0x%x\n", HARDWARE_INFO_VERSION);     

    return ret_value;
}
static ssize_t store_version(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    printk("[%s]: Not Support Write Function\n",__func__);      
    return size;
}




static ssize_t show_lcm(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value = 1;

        //lcm_drv = _get_lcm_driver_by_handle(plcm);
        
    ret_value = sprintf(buf, "lcd name    :%s\n", hardware_info.hq_lcm_name);    

    return ret_value;
}
static ssize_t store_lcm(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    printk("[%s]: Not Support Write Function\n",__func__);      
    return size;
}

extern struct tpd_driver_t *g_tpd_drv;

static ssize_t show_ctp(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value = 1;
    int count = 0;
        
	if(hardware_info.hq_tpd_name != NULL)
	{
		count += sprintf(buf + count, "ctp driver  :%s , ctp fw ver  :%x\n", hardware_info.hq_tpd_name, hardware_info.tpd_fw_version);
		count += sprintf(buf + count, "ctp driver  :%s , ctp color  :%x\n", hardware_info.hq_tpd_name, hardware_info.tpd_color);
		count += sprintf(buf + count, "ctp driver  :%s , ctp module id  :%x\n", hardware_info.hq_tpd_name, hardware_info.tpd_module_id);
	}
	else
	{
		count += sprintf(buf + count, "ctp driver NULL !\n");
	}

        ret_value = count;
        
    return ret_value;
}
static ssize_t store_ctp(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    printk("[%s]: Not Support Write Function\n",__func__);      
    return size;
}


static ssize_t show_main_camera(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value = 1;

   
    if(hardware_info.mainCameraName)
        ret_value = sprintf(buf , "main camera :%s\n", hardware_info.mainCameraName);
    else
        ret_value = sprintf(buf , "main camera :PLS TURN ON CAMERA FIRST\n");
        
        
    return ret_value;
}
static ssize_t store__main_camera(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    printk("[%s]: Not Support Write Function\n",__func__);      
    return size;
}


static ssize_t show_sub_camera(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value = 1;
    int count = 0;

    if(hardware_info.subCameraName)
        ret_value = sprintf(buf , "sub camera  :%s\n", hardware_info.subCameraName);
    else
        ret_value = sprintf(buf , "sub camera  :PLS TURN ON CAMERA FIRST\n");

        
    return ret_value;
}
static ssize_t store_sub_camera(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    printk("[%s]: Not Support Write Function\n",__func__);      
    return size;
}


static ssize_t show_alsps(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value = 1;
    #if defined(CUSTOM_KERNEL_ALSPS)
    if(hardware_info.alsps_name)
        ret_value = sprintf(buf, "AlSPS name  :%s\n",hardware_info.alsps_name);     
    else
        ret_value = sprintf(buf, "AlSPS name  :Not found\n"); 
    #else
    ret_value = sprintf(buf, "AlSPS name  :Not support ALSPS\n");       
    #endif
    return ret_value;
}

static ssize_t store_alsps(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    printk("[%s]: Not Support Write Function\n",__func__);      
    return size;
}


static ssize_t show_gsensor(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value = 1;
    #if defined(CUSTOM_KERNEL_ACCELEROMETER)
    if(hardware_info.gsenor_name)
        ret_value = sprintf(buf, "GSensor name:%s\n",hardware_info.gsenor_name);   
    else
        ret_value = sprintf(buf, "GSensor name:Not found\n"); 
    #else
    ret_value = sprintf(buf, "GSensor name:Not support GSensor\n");     
    #endif
    return ret_value;
}

static ssize_t store_gsensor(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    printk("[%s]: Not Support Write Function\n",__func__);      
    return size;
}


static ssize_t show_msensor(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value = 1;
    #if defined(CUSTOM_KERNEL_MAGNETOMETER)
    if(hardware_info.msensor_name)
        ret_value = sprintf(buf, "MSensor name:%s\n",hardware_info.msensor_name);   
    else
        ret_value = sprintf(buf, "MSensor name:Not found\n"); 
    #else
    ret_value = sprintf(buf, "MSensor name:Not support MSensor\n");     
    #endif
    return ret_value;
}

static ssize_t store_msensor(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    printk("[%s]: Not Support Write Function\n",__func__);      
    return size;
}

static ssize_t show_gyro(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value = 1;
    #if defined(CUSTOM_KERNEL_GYROSCOPE)
    if(hardware_info.gyro_name)
        ret_value = sprintf(buf, "Gyro  name  :%s\n",hardware_info.gyro_name);      
    else
        ret_value = sprintf(buf, "Gyro  name  :Not found\n"); 
    #else
    ret_value = sprintf(buf, "Gyro  name  :Not support Gyro\n");        
    #endif
    return ret_value;
}

static ssize_t store_gyro(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    printk("[%s]: Not Support Write Function\n",__func__);      
    return size;
}

static ssize_t show_speaker(struct device *dev,struct device_attribute *attr, char *buf)
{
    printk("[%s]: Not Support read Function\n",__func__);      
    return 1;
}

static ssize_t store_speaker(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    char tmp[2];
	memset(tmp, 0, sizeof(tmp));
	sprintf(tmp, "%s", (char *) buf);
	printk("speaker control size %d\n", size);
	if (size == 3) {
		simple_strtoul(tmp, NULL, 2);
		printk("speaker control status %c %c %s\n", tmp[0], tmp[1], buf);
		if (tmp[0] == '1') {
			printk("enable left speaker\n");
			mt_set_gpio_mode(GPIO_SPEAKER_EN_PIN, GPIO_MODE_00);
			mt_set_gpio_dir(GPIO_SPEAKER_EN_PIN, GPIO_DIR_OUT);
			//mt_set_gpio_out(GPIO_SPEAKER_EN_PIN,GPIO_OUT_ONE); 
		} else {
			mt_set_gpio_mode(GPIO_SPEAKER_EN_PIN, GPIO_MODE_00);
			mt_set_gpio_dir(GPIO_SPEAKER_EN_PIN, GPIO_DIR_IN);
			//mt_set_gpio_out(GPIO_SPEAKER_EN_PIN,GPIO_OUT_ZERO); 
		}

		if (tmp[1] == '1') {
			printk("enable right speaker\n");
			mt_set_gpio_mode(GPIO_EXTERNAL_AMPLIFIER_PIN, GPIO_MODE_00);
			mt_set_gpio_dir(GPIO_EXTERNAL_AMPLIFIER_PIN, GPIO_DIR_OUT);
			//mt_set_gpio_out(GPIO_EXTERNAL_AMPLIFIER_PIN,GPIO_OUT_ONE); 
		} else {
			mt_set_gpio_mode(GPIO_EXTERNAL_AMPLIFIER_PIN, GPIO_MODE_00);
			mt_set_gpio_dir(GPIO_EXTERNAL_AMPLIFIER_PIN, GPIO_DIR_IN);
			//mt_set_gpio_out(GPIO_EXTERNAL_AMPLIFIER_PIN,GPIO_OUT_ZERO); 
		}
	}
    return 1;
}







static DEVICE_ATTR(99_version, 0644, show_version, store_version);
static DEVICE_ATTR(00_lcm, 0644, show_lcm, store_lcm);
static DEVICE_ATTR(01_ctp, 0644, show_ctp, store_ctp);
static DEVICE_ATTR(02_main_camera, 0644, show_main_camera, store__main_camera);
static DEVICE_ATTR(03_sub_camera, 0644, show_sub_camera, store_sub_camera);

static DEVICE_ATTR(05_gsensor, 0644, show_gsensor, store_gsensor);
static DEVICE_ATTR(06_msensor, 0644, show_msensor, store_msensor);
static DEVICE_ATTR(08_gyro, 0644, show_gyro, store_gyro);
static DEVICE_ATTR(09_speaker, 0666, show_speaker, store_speaker);


static DEVICE_ATTR(07_alsps, 0644, show_alsps, store_alsps);
static DEVICE_ATTR(hq_hardwareinfo, 0644, show_hardware_info, store_hardware_info);

///////////////////////////////////////////////////////////////////////////////////////////
//// hq factory apk support
///////////////////////////////////////////////////////////////////////////////////////////

#define PROC_NAME1	"boot_status"
static struct proc_dir_entry *fts_proc_entry1;
extern BOOTMODE get_boot_mode_ex(void);
static ssize_t hq_apk_debug_read1(struct file *file, char __user *buf, size_t size, loff_t *ppos)
{
	unsigned char mode[1];
	int ret = 1;
	int boot_mode_ex = get_boot_mode_ex();
	printk("boot_mode_ex=%d\n",boot_mode_ex);
	if (11 == boot_mode_ex){
			printk("boot_mode_ex_11 proc_read success\n");
			mode[0] = 1;
	} 
	else if(12 == boot_mode_ex)
	{
			printk("boot_mode_ex_12 proc_read success\n");
			mode[0] = 2;
	}
	else if(13 == boot_mode_ex)// //demomode
	{
			printk("boot_mode_ex_13 proc_read success\n");
			mode[0] = 3;
	}
	else
	{
			mode[0] = 0;
	}
	ret = copy_to_user(buf, mode, sizeof(mode));
	if (ret)
			printk("factory flag copy to user failed\n");
	else
			printk("factory flag copy to user success\n");
	ret = sizeof(mode);
	return ret;

}

static const struct file_operations boot_status_fops = { 
    .read = hq_apk_debug_read1
};

int hq_apk_create_proc_boot_status(void )
{
	fts_proc_entry1 = proc_create(PROC_NAME1, 0777, NULL, &boot_status_fops);
	if (NULL == fts_proc_entry1) {
		printk("create_proc_entry %s failed\n", PROC_NAME1);
		return -ENOMEM;
		}
	return 0;
}


///////////////////////////////////////////////////////////////////////////////////////////
//// platform_driver API 
///////////////////////////////////////////////////////////////////////////////////////////
static int HardwareInfo_driver_probe(struct platform_device *dev)       
{       
        int ret_device_file = 0;

    printk("** HardwareInfo_driver_probe!! **\n" );
    if((ret_device_file = device_create_file(&(dev->dev), &dev_attr_00_lcm)) != 0) goto exit_error;
    if((ret_device_file = device_create_file(&(dev->dev), &dev_attr_01_ctp)) != 0) goto exit_error;
    if((ret_device_file = device_create_file(&(dev->dev), &dev_attr_02_main_camera)) != 0) goto exit_error;
    if((ret_device_file = device_create_file(&(dev->dev), &dev_attr_03_sub_camera)) != 0) goto exit_error;
  
    if((ret_device_file = device_create_file(&(dev->dev), &dev_attr_05_gsensor)) != 0) goto exit_error;
    if((ret_device_file = device_create_file(&(dev->dev), &dev_attr_06_msensor)) != 0) goto exit_error;
    if((ret_device_file = device_create_file(&(dev->dev), &dev_attr_08_gyro)) != 0) goto exit_error;
    if((ret_device_file = device_create_file(&(dev->dev), &dev_attr_07_alsps)) != 0) goto exit_error;
    if((ret_device_file = device_create_file(&(dev->dev), &dev_attr_09_speaker)) != 0) goto exit_error;
    if((ret_device_file = device_create_file(&(dev->dev), &dev_attr_99_version)) != 0) goto exit_error;   
    if((ret_device_file = device_create_file(&(dev->dev), &dev_attr_hq_hardwareinfo)) != 0) goto exit_error;
	hq_apk_create_proc_boot_status();

exit_error:     
    return ret_device_file;
}

static int HardwareInfo_driver_remove(struct platform_device *dev)
{
        printk("** HardwareInfo_drvier_remove!! **");

        device_remove_file(&(dev->dev), &dev_attr_00_lcm);
    device_remove_file(&(dev->dev), &dev_attr_01_ctp);
    device_remove_file(&(dev->dev), &dev_attr_02_main_camera);
    device_remove_file(&(dev->dev), &dev_attr_03_sub_camera);
    
    device_remove_file(&(dev->dev), &dev_attr_05_gsensor);
    device_remove_file(&(dev->dev), &dev_attr_06_msensor);
    device_remove_file(&(dev->dev), &dev_attr_08_gyro);
    device_remove_file(&(dev->dev), &dev_attr_07_alsps);
    device_remove_file(&(dev->dev), &dev_attr_09_speaker);
	
    device_remove_file(&(dev->dev), &dev_attr_99_version);
    return 0;
}





static struct platform_driver HardwareInfo_driver = {
    .probe              = HardwareInfo_driver_probe,
    .remove     = HardwareInfo_driver_remove,
    .driver     = {
        .name = "HardwareInfo",
    },
};

static struct platform_device HardwareInfo_device = {
    .name   = "HardwareInfo",
    .id     = -1,
};




static int __init HardwareInfo_mod_init(void)
{
    int ret = 0;


    ret = platform_device_register(&HardwareInfo_device);
    if (ret) {
        printk("**HardwareInfo_mod_init  Unable to driver register(%d)\n", ret);
        goto  fail_2;
    }
    

    ret = platform_driver_register(&HardwareInfo_driver);
    if (ret) {
        printk("**HardwareInfo_mod_init  Unable to driver register(%d)\n", ret);
        goto  fail_1;
    }

    goto ok_result;

    
fail_1:
        platform_driver_unregister(&HardwareInfo_driver);
fail_2:
        platform_device_unregister(&HardwareInfo_device);
ok_result:

    return ret;
}


/*****************************************************************************/
static void __exit HardwareInfo_mod_exit(void)
{
    platform_driver_unregister(&HardwareInfo_driver);
        platform_device_unregister(&HardwareInfo_device);
}
/*****************************************************************************/
module_init(HardwareInfo_mod_init);
module_exit(HardwareInfo_mod_exit);
/*****************************************************************************/
MODULE_AUTHOR("Kaka Ni <nigang@huaqin.com>");
MODULE_DESCRIPTION("MT6575 Hareware Info driver");
MODULE_LICENSE("GPL");

