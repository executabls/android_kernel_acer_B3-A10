/*****************************************************************************
*                E X T E R N A L      R E F E R E N C E S
******************************************************************************
*/

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/interrupt.h>
#include <linux/vmalloc.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/wait.h>
#include <linux/spinlock.h>
#include <linux/semaphore.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include "yusu_android_speaker.h"

#include <mach/mt_gpio.h>
#include <mach/mt_typedefs.h>

/*****************************************************************************
*                C O M P I L E R      F L A G S
******************************************************************************
*/
//#define CONFIG_DEBUG_MSG
#ifdef CONFIG_DEBUG_MSG
#define PRINTK(format, args...) printk( KERN_EMERG format,##args )
#else
#define PRINTK(format, args...)
#endif


/*****************************************************************************
*                          C O N S T A N T S
******************************************************************************
*/

#define GPIO_SPEAKER_EN_PIN (GPIO16 | 0x80000000)
#define GPIO_EXTERNAL_AMPLIFIER_PIN (GPIO18 | 0x80000000)
#define SPK_WARM_UP_TIME        (40) //unit is ms
#define SPK_COOL_DOWN_TIME      (200) //unit is us
/*****************************************************************************
*                         D A T A      T Y P E S
******************************************************************************
*/
static int Speaker_Volume=0;
static bool gsk_on=false; // speaker is open?
static bool gsk_left_on=false;
static bool gsk_right_on=false;
static bool gsk_resume=false;
static bool gsk_forceon=false;

/*****************************************************************************
*                  F U N C T I O N        D E F I N I T I O N
******************************************************************************
*/
extern void Yusu_Sound_AMP_Switch(BOOL enable);

bool Speaker_Init(void)
{
   PRINTK("+Speaker_Init Success");
   mt_set_gpio_mode(GPIO_SPEAKER_EN_PIN,GPIO_MODE_00);  // gpio mode
   mt_set_gpio_pull_enable(GPIO_SPEAKER_EN_PIN,GPIO_PULL_ENABLE);
   mt_set_gpio_dir(GPIO_SPEAKER_EN_PIN,GPIO_DIR_OUT); // output
   
   mt_set_gpio_mode(GPIO_EXTERNAL_AMPLIFIER_PIN,GPIO_MODE_00);  // gpio mode
   mt_set_gpio_pull_enable(GPIO_EXTERNAL_AMPLIFIER_PIN,GPIO_PULL_ENABLE);
   mt_set_gpio_dir(GPIO_EXTERNAL_AMPLIFIER_PIN,GPIO_DIR_OUT); // output

   PRINTK("-Speaker_Init Success");
   return true;
}

bool Speaker_Register(void)
{
    return false;
}

int ExternalAmp(void)
{
	return 0;
}

bool Speaker_DeInit(void)
{
	return false;
}
/* 0.75us<TL<10us; 0.75us<TH<10us */
#define GAP (2) //unit: us
#define AW8736_MODE1 /*1.2w*/ \
    mt_set_gpio_out(GPIO_SPEAKER_AMP_EN,GPIO_OUT_ONE);

#define AW8736_MODE2 /*1.0w*/ \
    mt_set_gpio_out(GPIO_SPEAKER_AMP_EN,GPIO_OUT_ONE); \
    udelay(GAP); \
    mt_set_gpio_out(GPIO_SPEAKER_AMP_EN,GPIO_OUT_ZERO); \
    udelay(GAP); \
    mt_set_gpio_out(GPIO_SPEAKER_AMP_EN,GPIO_OUT_ONE);

#define AW8736_MODE3 /*0.8w*/ \
    mt_set_gpio_out(GPIO_SPEAKER_AMP_EN,GPIO_OUT_ONE); \
    udelay(GAP); \
    mt_set_gpio_out(GPIO_SPEAKER_AMP_EN,GPIO_OUT_ZERO); \
    udelay(GAP); \
    mt_set_gpio_out(GPIO_SPEAKER_AMP_EN,GPIO_OUT_ONE); \
    udelay(GAP); \
    mt_set_gpio_out(GPIO_SPEAKER_AMP_EN,GPIO_OUT_ZERO); \
    udelay(GAP); \
    mt_set_gpio_out(GPIO_SPEAKER_AMP_EN,GPIO_OUT_ONE);

#define AW8736_MODE4 /*it depends on THD, range: 1.5 ~ 2.0w*/ \
    mt_set_gpio_out(GPIO_SPEAKER_AMP_EN,GPIO_OUT_ONE); \
    udelay(GAP); \
    mt_set_gpio_out(GPIO_SPEAKER_AMP_EN,GPIO_OUT_ZERO); \
    udelay(GAP); \
    mt_set_gpio_out(GPIO_SPEAKER_AMP_EN,GPIO_OUT_ONE); \
    udelay(GAP); \
    mt_set_gpio_out(GPIO_SPEAKER_AMP_EN,GPIO_OUT_ZERO); \
    udelay(GAP); \
    mt_set_gpio_out(GPIO_SPEAKER_AMP_EN,GPIO_OUT_ONE); \
    udelay(GAP); \
    mt_set_gpio_out(GPIO_SPEAKER_AMP_EN,GPIO_OUT_ZERO); \
    udelay(GAP); \
    mt_set_gpio_out(GPIO_SPEAKER_AMP_EN,GPIO_OUT_ONE);


void Sound_SpeakerL_SetVolLevel(int level)
{
   PRINTK(" Sound_SpeakerL_SetVolLevel level=%d\n",level);
}

void Sound_SpeakerR_SetVolLevel(int level)
{
   PRINTK(" Sound_SpeakerR_SetVolLevel level=%d\n",level);
}

void Sound_Speaker_Turnon(int channel)
{
    PRINTK("Sound_Speaker_Turnon channel = %d\n",channel);

	if(Channel_Left ==channel)
	{
		if(gsk_left_on)
			return;
		
		mt_set_gpio_out(GPIO_SPEAKER_EN_PIN,GPIO_OUT_ONE); // high
		udelay(2);
		mt_set_gpio_out(GPIO_SPEAKER_EN_PIN,GPIO_OUT_ZERO); // high
		udelay(2);
		mt_set_gpio_out(GPIO_SPEAKER_EN_PIN,GPIO_OUT_ONE); // high
		udelay(2);
		mt_set_gpio_out(GPIO_SPEAKER_EN_PIN,GPIO_OUT_ZERO); // high
		udelay(2);
		mt_set_gpio_out(GPIO_SPEAKER_EN_PIN,GPIO_OUT_ONE); // high

		gsk_left_on = true;
		msleep(SPK_WARM_UP_TIME);
	}
	else if(Channel_Right==channel)
	{
		if(gsk_right_on)
			return;
		
		mt_set_gpio_out(GPIO_EXTERNAL_AMPLIFIER_PIN,GPIO_OUT_ONE); 
		udelay(2);
		mt_set_gpio_out(GPIO_EXTERNAL_AMPLIFIER_PIN,GPIO_OUT_ZERO); 
		udelay(2);
		mt_set_gpio_out(GPIO_EXTERNAL_AMPLIFIER_PIN,GPIO_OUT_ONE); 
		udelay(2);
		mt_set_gpio_out(GPIO_EXTERNAL_AMPLIFIER_PIN,GPIO_OUT_ZERO); 
		udelay(2);
		mt_set_gpio_out(GPIO_EXTERNAL_AMPLIFIER_PIN,GPIO_OUT_ONE); 
		gsk_right_on = true;
		msleep(SPK_WARM_UP_TIME);
	}
	else
	{
		if(!gsk_left_on)
		{
			mt_set_gpio_out(GPIO_SPEAKER_EN_PIN,GPIO_OUT_ONE); // high
			udelay(2);
			mt_set_gpio_out(GPIO_SPEAKER_EN_PIN,GPIO_OUT_ZERO); // high
			udelay(2);
			mt_set_gpio_out(GPIO_SPEAKER_EN_PIN,GPIO_OUT_ONE); // high
			udelay(2);
			mt_set_gpio_out(GPIO_SPEAKER_EN_PIN,GPIO_OUT_ZERO); // high
			udelay(2);
			mt_set_gpio_out(GPIO_SPEAKER_EN_PIN,GPIO_OUT_ONE); // high
			gsk_left_on =true;
		}
		if(!gsk_right_on)
		{
			mt_set_gpio_out(GPIO_EXTERNAL_AMPLIFIER_PIN,GPIO_OUT_ONE); 
			udelay(2);
			mt_set_gpio_out(GPIO_EXTERNAL_AMPLIFIER_PIN,GPIO_OUT_ZERO); 
			udelay(2);
			mt_set_gpio_out(GPIO_EXTERNAL_AMPLIFIER_PIN,GPIO_OUT_ONE); 
			udelay(2);
			mt_set_gpio_out(GPIO_EXTERNAL_AMPLIFIER_PIN,GPIO_OUT_ZERO); 
			udelay(2);
			mt_set_gpio_out(GPIO_EXTERNAL_AMPLIFIER_PIN,GPIO_OUT_ONE); 

			gsk_right_on= true;
		}
		msleep(SPK_WARM_UP_TIME);
	}
}

void Sound_Speaker_Turnoff(int channel)
{
    PRINTK("Sound_Speaker_Turnoff channel = %d\n",channel);

	if(Channel_Left ==channel)
	{
		if(!gsk_left_on)
			return;
		mt_set_gpio_out(GPIO_SPEAKER_EN_PIN,GPIO_OUT_ZERO); // high
		gsk_left_on = false;
	}
	else if(Channel_Right==channel)
	{
		if(!gsk_right_on)
			return;
		mt_set_gpio_out(GPIO_EXTERNAL_AMPLIFIER_PIN,GPIO_OUT_ZERO); 
		gsk_right_on=false;
	}
	else
	{
		if(gsk_left_on)
		{
			mt_set_gpio_out(GPIO_SPEAKER_EN_PIN,GPIO_OUT_ZERO); // high
			gsk_left_on =false;
			
		}
		if(gsk_right_on)
		{
			mt_set_gpio_out(GPIO_EXTERNAL_AMPLIFIER_PIN,GPIO_OUT_ZERO); 
			gsk_right_on= false;
		}

	}
}

void Sound_Speaker_SetVolLevel(int level)
{
    Speaker_Volume =level;
}

void Sound_Headset_Turnon(void)
{

}

void Sound_Headset_Turnoff(void)
{
}

void Sound_Earpiece_Turnon(void)
{
}

void Sound_Earpiece_Turnoff(void)
{
}

//kernal use
void AudioAMPDevice_Suspend(void)
{
	PRINTK("AudioDevice_Suspend\n");
	if(gsk_on)
	{
		Sound_Speaker_Turnoff(Channel_Stereo);
		gsk_resume = true;
	}

}
void AudioAMPDevice_Resume(void)
{
	PRINTK("AudioDevice_Resume\n");
	if(gsk_resume)
		Sound_Speaker_Turnon(Channel_Stereo);
	gsk_resume = false;
}
void AudioAMPDevice_SpeakerLouderOpen(void)
{
	PRINTK("AudioDevice_SpeakerLouderOpen\n");
	gsk_forceon = false;
	if(gsk_on)
		return;
	Sound_Speaker_Turnon(Channel_Stereo);
	gsk_forceon = true;
	return ;

}
void AudioAMPDevice_SpeakerLouderClose(void)
{
	PRINTK("AudioDevice_SpeakerLouderClose\n");

	if(gsk_forceon)
		Sound_Speaker_Turnoff(Channel_Stereo);
	gsk_forceon = false;

}
void AudioAMPDevice_mute(void)
{
	PRINTK("AudioDevice_mute\n");
	if(gsk_on)
		Sound_Speaker_Turnoff(Channel_Stereo);
}

int Audio_eamp_command(unsigned int type, unsigned long args, unsigned int count)
{
	return 0;
}
static char *ExtFunArray[] =
{
    "InfoMATVAudioStart",
    "InfoMATVAudioStop",
    "End",
};

kal_int32 Sound_ExtFunction(const char* name, void* param, int param_size)
{
	int i = 0;
	int funNum = -1;

	//Search the supported function defined in ExtFunArray
	while(strcmp("End",ExtFunArray[i]) != 0 ) {		//while function not equal to "End"

	    if (strcmp(name,ExtFunArray[i]) == 0 ) {		//When function name equal to table, break
	    	funNum = i;
	    	break;
	    }
	    i++;
	}

	switch (funNum) {
	    case 0:			//InfoMATVAudioStart
	        printk("RunExtFunction InfoMATVAudioStart \n");
	        break;

	    case 1:			//InfoMATVAudioStop
	        printk("RunExtFunction InfoMATVAudioStop \n");
	        break;

	    default:
	    	 break;
	}

	return 1;
}


