include $(srctree)/drivers/misc/mediatek/Makefile.custom

# Linux driver folder
ccflags-y += -I$(srctree)/arch/arm/mach-$(MTK_PLATFORM)/$(ARCH_MTK_PROJECT)/touchpanel/GT9XX/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/GT9XX/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/

obj-y	+=  elan_ts.o

