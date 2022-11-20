# Makefile for the focaltech touchscreen drivers.

ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/$(CONFIG_TOUCHSCREEN_FOCALTECH_TOUCH_V33)
ccflags-y += -I$(srctree)/drivers/spi/mediatek/$(MTK_PLATFORM)
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include/
ccflags-y += -I$(srctree)/drivers/base
ccflags-y += -I$(srctree)/drivers/misc/mediatek/hwmon/include
ccflags-y += -I$(srctree)/drivers/misc/mediatek/alsps/inc/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/sensors-1.0/hwmon/include
ccflags-y += -I$(srctree)/drivers/misc/mediatek/sensors-1.0/alsps/inc/

obj-$(CONFIG_TOUCHSCREEN_FOCALTECH_TOUCH_V33)	+=  focaltech_core.o
obj-$(CONFIG_TOUCHSCREEN_FOCALTECH_TOUCH_V33)	+=  focaltech_ex_fun.o
obj-$(CONFIG_TOUCHSCREEN_FOCALTECH_TOUCH_V33)	+=  focaltech_ex_mode.o
obj-$(CONFIG_TOUCHSCREEN_FOCALTECH_TOUCH_V33)	+=  focaltech_gesture.o
obj-$(CONFIG_TOUCHSCREEN_FOCALTECH_TOUCH_V33)	+=  focaltech_esdcheck.o
obj-$(CONFIG_TOUCHSCREEN_FOCALTECH_TOUCH_V33)   +=  focaltech_point_report_check.o
obj-$(CONFIG_TOUCHSCREEN_FOCALTECH_TOUCH_V33)   +=  focaltech_proximity.o


obj-$(CONFIG_TOUCHSCREEN_FOCALTECH_TOUCH_V33)	+=  focaltech_flash.o
obj-$(CONFIG_TOUCHSCREEN_FOCALTECH_TOUCH_V33)   +=  focaltech_flash/



obj-$(CONFIG_TOUCHSCREEN_FOCALTECH_TOUCH_V33)   +=  focaltech_i2c.o
