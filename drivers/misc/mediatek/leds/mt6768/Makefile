#
# Copyright (C) 2015 MediaTek Inc.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 2 as
# published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#

obj-$(CONFIG_MTK_LEDS) := mtk_leds.o

# prize wyq 20191015 add for aw99703 backlight led-start
obj-$(CONFIG_BACKLIGHT_SUPPORT_AW99703) += leds_aw99703.o
# prize wyq 20191015 add for aw99703 backlight led-end
ccflags-y += -I$(srctree)/drivers/misc/mediatek/video/include
