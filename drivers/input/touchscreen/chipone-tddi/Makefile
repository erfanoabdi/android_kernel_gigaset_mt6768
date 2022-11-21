ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/ILITEK_V2/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include/

#obj-y   +=  tpd_control.o
#obj-y   +=  tpd_button.o
#obj-y   +=  tpd_debug.o
#obj-y   +=  tpd_default.o
#obj-y   +=  tpd_misc.o
#obj-y   +=  tpd_setting.o


obj-$(CONFIG_TOUCHSCREEN_CHIPONE) += chipone-tddi.o

chipone-tddi-objs := cts_tpd_driver.o cts_i2c_driver.o cts_core.o cts_platform.o cts_sfctrlv2.o cts_spi_flash.o cts_firmware.o cts_tool.o cts_sysfs.o cts_test.o

