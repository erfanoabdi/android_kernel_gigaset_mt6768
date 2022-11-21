ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include
#prize-add TP-pengzhipeng-20220322-start
ccflags-y += -I$(srctree)/drivers/gpu/drm/mediatek
#prize-add TP-pengzhipeng-20220322-start
# In case the platform does NOT support this type of sensors
obj-y   +=  mtk_tpd.o
obj-y   +=  tpd_button.o
obj-y   +=  tpd_calibrate.o
obj-y   +=  tpd_debug.o
obj-y   +=  tpd_default.o
obj-y   +=  tpd_misc.o
obj-y   +=  tpd_setting.o
obj-$(CONFIG_TOUCHSCREEN_MTK_SOLOMON)	+=  SOLOMON/
obj-$(CONFIG_TOUCHSCREEN_FTS)	+=  focaltech_touch/
obj-$(CONFIG_TOUCHSCREEN_MTK_GT1151)	+=  GT1151/
obj-$(CONFIG_TOUCHSCREEN_MTK_GT911)	+=  GT911/
obj-$(CONFIG_TOUCHSCREEN_MTK_GSLX680)	+=  gslX680/
obj-$(CONFIG_TOUCHSCREEN_MTK_NT36xxx)	+=  NT36xxx/
obj-$(CONFIG_TOUCHSCREEN_MTK_GT9XX_LM)	+=  GT9XX_LM/
obj-$(CONFIG_TOUCHSCREEN_FT5X26)	+=  FT5X26/
obj-$(CONFIG_TOUCHSCREEN_MTK_GT5688)	+=  GT5688/
obj-$(CONFIG_TOUCHSCREEN_HIMAX_I2C)	+=  hxchipset/
obj-$(CONFIG_TOUCHSCREEN_MTK_FOCALTECH_FHD)	+=  focaltech_fhd_touch/
obj-$(CONFIG_TOUCHSCREEN_MTK_GT9157)    +=  GT9157/
obj-$(CONFIG_TOUCHSCREEN_HIMAX_CHIPSET_8789P1_8185P3)    +=  hxchipset_8789p1_8185p3/
obj-$(CONFIG_TOUCHSCREEN_HIMAX_CHIPSET) += hxchipset_hx83102p/
obj-$(CONFIG_TOUCHSCREEN_GOODIX_BRL) += goodix_berlin_driver/
#prize-add-lishuwen-20191226-start
obj-$(CONFIG_TOUCHSCREEN_MTK_GT9XX_PRIZE)	+=  GT9XX_prize/
#prize-add-lishuwen-20191226-end
#prize-add-songhezhu-20191231-start
obj-$(CONFIG_TOUCHSCREEN_CHIPONE) += chipone-tddi/
#prize-add-songhezhu-20191231-end
#prize-add galaxycore TP-pengzhipeng-20220322-start
obj-$(CONFIG_TOUCHSCREEN_GC7302_SPI) += galaxycore/
obj-$(CONFIG_TOUCHSCREEN_MTK_ILITEK_V3)    +=  ILITEK_V3/
#prize-add galaxycore TP-pengzhipeng-20220322-end
#prize by xuejian add fts 20220514 start
obj-$(CONFIG_TOUCHSCREEN_FOCALTECH_TOUCH_V33)	+=  focaltech_touch_V33/
#prize by xuejian add fts 20220514 end
# prize add by liaoxingen start
obj-$(CONFIG_TOUCHSCREEN_ILITEK9882_SPI) += ILITEK9882_SPI/ 
# prize add by liaoxingen end 
