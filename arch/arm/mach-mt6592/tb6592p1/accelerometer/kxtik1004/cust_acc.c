#include <linux/types.h>
#include <cust_acc.h>
#include <mach/mt_pm_ldo.h>

#define KXTIK1004_I2C_SLAVE_ADDR		0x1C

/*---------------------------------------------------------------------------*/
static struct acc_hw cust_acc_hw = {
    .i2c_num = 2,
    .direction = 5,
    .power_id = MT65XX_POWER_NONE,  /*!< LDO is not used */
    .power_vol= VOL_DEFAULT,        /*!< LDO is not used */
    .firlen = 0, //old value 16                /*!< don't enable low pass fileter */
    .i2c_addr = {KXTIK1004_I2C_SLAVE_ADDR,0},   /* customize for special slave address */
};
/*---------------------------------------------------------------------------*/
struct acc_hw* get_cust_acc_hw(void) 
{
    return &cust_acc_hw;
}
