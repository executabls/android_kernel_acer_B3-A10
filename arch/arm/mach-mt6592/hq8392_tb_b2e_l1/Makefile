
include $(srctree)/drivers/misc/mediatek/Makefile.custom

# Module project related part

ifeq ($(CONFIG_CUSTOM_KERNEL_ACCELEROMETER),y)
obj-y   +=  accelerometer/
endif

ifeq ($(CONFIG_CUSTOM_KERNEL_GYROSCOPE),y)
obj-y   +=  gyroscope/
endif

ifeq ($(CONFIG_CUSTOM_KERNEL_MAGNETOMETER),y)
obj-y	+=  magnetometer/
endif

ifeq ($(CONFIG_CUSTOM_KERNEL_ALSPS),y)
obj-y   +=  alsps/
endif

ifneq ($(CONFIG_ARCH_MTK_PROJECT),)
ARCH_MTK_PROJECT := $(shell echo $(CONFIG_ARCH_MTK_PROJECT))
obj-y	+= core/
obj-y	+= dct/dct/
endif

obj-y	+=  camera/camera/
obj-y   +=  flashlight/constant_flashlight/
obj-y	+=  vibrator/
obj-y	+=  leds/
obj-y	+=  keypad/
obj-y	+=  sound/
obj-y	+=  accdet/

