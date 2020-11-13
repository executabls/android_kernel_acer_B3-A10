#include <linux/types.h>
#include <linux/string.h>
#include <mach/mt_pm_ldo.h>
#include <cust_mag.h>


static struct mag_hw cust_mag_hw = {
    .i2c_num = 2,
    .direction = 6,
    .power_id = MT65XX_POWER_NONE,  /*!< LDO is not used */
    .power_vol= VOL_DEFAULT,        /*!< LDO is not used */
};
#if 0
static struct mag_hw cust_mag_hw_rotation270 = {
    .i2c_num = 2,
    .direction = 7,
    .power_id = MT65XX_POWER_NONE,  /*!< LDO is not used */
    .power_vol= VOL_DEFAULT,        /*!< LDO is not used */
};
#endif
struct mag_hw* get_cust_mag_hw(void) 
{
#if 0
    if(0 == strncmp(MTK_LCM_PHYSICAL_ROTATION, "270", 3))
        return &cust_mag_hw_rotation270; /*for p2[wsvga]*/
    else
#endif
        return &cust_mag_hw;
}
