
include $(srctree)/drivers/misc/mediatek/Makefile.custom

# Module project related part

ifeq ($(CONFIG_MTK_MPU6515A),y)
obj-y	+=  mpu6515/
endif

ifeq ($(CONFIG_MTK_KXTJ2_1009_NEW),y)
obj-y	+=  KXTJ2_1009-new/
endif

ifeq ($(CONFIG_MTK_BMA250E),y)
obj-y	+=  bma250e/
endif

ifeq ($(CONFIG_MTK_MPU60X0),y)
obj-y   +=  mpu60x0/
endif

ifeq ($(CONFIG_MTK_KXTIK1004),y)
obj-y   +=  kxtik1004/
endif
