#
# Makefile for the kernel flashlight device drivers.
#

flashlight-y := flashlight-core.o
flashlight-y += flashlight-device.o
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/

obj-$(CONFIG_MTK_FLASHLIGHT)             += flashlight.o
obj-$(CONFIG_MTK_V4L2_FLASHLIGHT)        += mtk-composite.o
obj-$(CONFIG_MTK_FLASHLIGHT_LED191)      += flashlights-led191.o
obj-$(CONFIG_MTK_FLASHLIGHT_LM3642)      += flashlights-lm3642.o
obj-$(CONFIG_MTK_FLASHLIGHT_LM3643)      += flashlights-lm3643.o
obj-$(CONFIG_MTK_FLASHLIGHT_LM3644)      += flashlights-lm3644.o
obj-$(CONFIG_MTK_FLASHLIGHT_RT4505)      += flashlights-rt4505.o
obj-$(CONFIG_MTK_FLASHLIGHT_RT5081)      += flashlights-rt5081.o
obj-$(CONFIG_MTK_FLASHLIGHT_MT6336)      += flashlights-mt6336.o
obj-$(CONFIG_MTK_FLASHLIGHT_MT6370)      += flashlights-mt6370.o
obj-$(CONFIG_MTK_FLASHLIGHT_MT6360)      += flashlights-mt6360.o
obj-$(CONFIG_MTK_FLASHLIGHT_AW3644)      += flashlights-aw3644.o
obj-$(CONFIG_MTK_FLASHLIGHT_SYWT78)      += flashlights-sywt78.o
obj-$(CONFIG_MTK_FLASHLIGHT_AW36515)      += flashlights-aw36515.o
# prize add by linchong 20220111 start
obj-$(CONFIG_MTK_FLASHLIGHT_AW36518)      += flashlights-aw36518.o
# prize add by linchong 20220111 start
obj-$(CONFIG_MTK_FLASHLIGHT_DUMMY)       += flashlights-dummy.o
obj-$(CONFIG_MTK_FLASHLIGHT_DUMMY_GPIO)  += flashlights-dummy-gpio.o
#/* prize add by zhuzhengjiang for flash ocp8111a 20211223 start*/
obj-$(CONFIG_MTK_FLASHLIGHT_OCP8111A_GPIO)  += flashlights-ocp8111a-gpio.o
#/* prize add by zhuzhengjiang for flash ocp8111a 20211223 end*/
#/* prize add by zhuzhengjiang for flash aw3641e 20201224 start*/
obj-$(CONFIG_MTK_FLASHLIGHT_AW3641E_GPIO)  += flashlights-aw3641e-gpio.o
#/* prize add by zhuzhengjiang for flash aw3641e 20201224 end*/

ccflags-$(CONFIG_MTK_FLASHLIGHT_DEBUG)     := -DDEBUG
ccflags-$(CONFIG_MTK_FLASHLIGHT_SYWT78)    += -DCONFIG_MTK_FLASHLIGHT_SYWT78
ccflags-$(CONFIG_MTK_FLASHLIGHT_PT)        += -DCONFIG_MTK_FLASHLIGHT_PT
ccflags-$(CONFIG_MTK_FLASHLIGHT_PT_STRICT) += -DCONFIG_MTK_FLASHLIGHT_PT_STRICT
ccflags-$(CONFIG_MTK_FLASHLIGHT_DLPT)      += -DCONFIG_MTK_FLASHLIGHT_DLPT

MTK_PLATFORM := $(subst ",,$(CONFIG_MTK_PLATFORM))
MTK_PROJECT := $(subst ",,$(CONFIG_ARCH_MTK_PROJECT))
MTK_CUSTOM_PATH := $(srctree)/drivers/misc/mediatek

ccflags-y += -D$(MTK_PLATFORM) -D$(MTK_PROJECT)
ifeq ($(CONFIG_MTK_FLASHLIGHT_AW3644),y)
ccflags-y += -DCONFIG_MTK_FLASHLIGHT_AW3644
endif
subdir-ccflags-y += \
	-I$(MTK_CUSTOM_PATH)/include/mt-plat \
	-I$(MTK_CUSTOM_PATH)/include/mt-plat/$(MTK_PLATFORM)/include/mach \
	-I$(MTK_CUSTOM_PATH)/pmic/include \
	-I$(MTK_CUSTOM_PATH)/pmic \
	-I$(MTK_CUSTOM_PATH)/power_throttling
#prize add by lipengpeng 20210208 start 
obj-$(CONFIG_MTK_FLASHLIGHT_SGM37895)      += flashlights-sgm37895.o
obj-$(CONFIG_MTK_FLASHLIGHT_SGM37895A)      += flashlights-sgm37895a.o
obj-$(CONFIG_MTK_FLASHLIGHT_AW36515B)     += flashlights-aw36515b.o
#prize add by lipengpeng 20210208 end 
obj-$(CONFIG_MTK_FLASHLIGHT_AW36515IR)     += flashlights-aw36515ir.o
###prize
obj-$(CONFIG_MTK_FLASHLIGHT_AW3641_GPIO)     += flashlights-aw3641-gpio.o
obj-$(CONFIG_MTK_FLASHLIGHT_ISINK)     += flashlights-isink.o
