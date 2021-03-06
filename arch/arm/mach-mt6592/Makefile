
# Makefile for the linux kernel.
#

include $(srctree)/drivers/misc/mediatek/Makefile.custom

ifeq ($(CONFIG_MTK_INTERNAL), y)
ifeq ($(MTK_USE_RESERVED_EXT_MEM), yes)
KBUILD_CFLAGS += -DMTK_USE_RESERVED_EXT_MEM
endif
endif

ifeq ($(CONFIG_MTK_SECURITY_SW_SUPPORT), y)
ifeq ($(MTK_SEC_FASTBOOT_UNLOCK_SUPPORT), yes)
KBUILD_CFLAGS += -DMTK_SEC_FASTBOOT_UNLOCK_SUPPORT
endif
endif

ifeq ($(MTK_TB_WIFI_3G_MODE), WIFI_ONLY)
KBUILD_CFLAGS += -DMTK_TB_WIFI_3G_MODE_WIFI_ONLY
endif

obj-y := core.o 
obj-y += timer.o 
# obj-y += clock.o
obj-y += mt_dbg.o
obj-y += mt_devs.o 
obj-y += mt_gpt.o 
obj-y += board.o 
obj-y += ca7_timer.o
obj-y += mt_cpuxgpt.o
#obj-y += mt_dma.o
obj-y += mt_reg_dump.o
obj-y += mt_device_apc.o
ifneq ($(CONFIG_MTK_FPGA),y)
obj-y += mt_pm_init.o
endif
ifeq ($(CONFIG_SMP),y)
ccflags-y += -I$(srctree)
obj-y += mt-smp.o 
obj-y += hotplug.o 
obj-y += mt-headsmp.o 
obj-y += mt_hotplug.o
obj-y += mt_hotplug_mechanism.o
endif
# TO-FIX add # for do early porting in JB migration
obj-y += eint.o 
obj-y += mt_freqhopping.o
obj-$(CONFIG_MT65XX_TRACER) += pmu_v7.o mon_interface.o mt_mon.o mt_emi_bm.o
obj-$(CONFIG_MTK_FLIPER) += pmu_v7.o mt_emi_bm.o mt_mem_bw.o
obj-y += camera_isp.o
obj-y += camera_sysram.o 
obj-y += camera_pipe_mgr.o
#obj-y += camera_eis.o
#obj-y += camera_fdvt.o
obj-y += mt_pm_ldo.o
obj-y += innercache.o 
obj-y += mt_l2c.o


ifneq ($(CONFIG_MTK_FPGA),y)
obj-y += mt_spm.o mt_spm_sleep.o
endif
obj-y += mt_spm_mtcmos.o
#obj-y += mt_spm.o 
#obj-y += mt_spm_
#obj-y += sleep.o
#obj-y += mt_spm_mtcmos.o
ifneq ($(CONFIG_MTK_FPGA),y)
obj-y += mt_sleep.o
endif
#obj-y += mt_cpufreq.o mt_gpufreq.o mt_cpu_ss.o
ifneq ($(CONFIG_MTK_FPGA),y)
obj-y += mt_cpufreq.o mt_cpu_ss.o
endif
#obj-y += mt_emifreq.o
obj-y += mt_dormant.o
obj-y += cpu_dormant.o
obj-y += mt_dcm.o 
obj-y += mt_idle.o mt_spm_idle.o
obj-y += mt_clkmgr_common.o
obj-y += mt_clkmgr.o
obj-y += sbchk_base.o
obj-y += mtk_ccci_helper.o
obj-y += mtk_eemcs_helper.o

ifneq ($(CONFIG_MTK_FPGA),y)
obj-y += mt_ptp.o
endif

ifneq ($(CONFIG_MTK_CCCI_DEVICES),y)
ccflags-y  += -D__USING_DUMMY_CCCI_API__
endif

ifeq ($(CONFIG_MTK_EEMCS_DRIVER),y)
ccflags-y  += -D__EEMCS_DRV_EN__
endif

obj-y += mt_dramc.o
obj-$(CONFIG_MTK_ETM) += pftracer.o

obj-y += mssv.o
obj-y += mt_cci400.o

# Hibernation
#AFLAGS_swsusp.o	:= -DTEXT_OFFSET=$(TEXT_OFFSET)
obj-$(CONFIG_HIBERNATION)	+= cpu_hibernate.o swsusp.o

ifneq ($(CONFIG_MTK_TABLET_HARDWARE), )
ccflags-y += -DMTK_TABLET_PLATFORM=\"$(shell echo $(CONFIG_MTK_TABLET_HARDWARE) | tr a-z A-Z )\"
endif

KBUILD_AFLAGS	:=$(KBUILD_AFLAGS:-msoft-float=-Wa,-mfpu=neon)

obj-y += $(ARCH_MTK_PROJECT)/

obj-y += hardwareinfo.o
