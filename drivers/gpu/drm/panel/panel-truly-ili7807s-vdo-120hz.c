/*
 * Copyright (c) 2015 MediaTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/backlight.h>
#include <drm/drmP.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_panel.h>

#include <linux/gpio/consumer.h>
#include <linux/regulator/consumer.h>

#include <video/mipi_display.h>
#include <video/of_videomode.h>
#include <video/videomode.h>

#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/of_graph.h>
#include <linux/platform_device.h>

#define CONFIG_MTK_PANEL_EXT
#if defined(CONFIG_MTK_PANEL_EXT)
#include "../mediatek/mtk_panel_ext.h"
#include "../mediatek/mtk_log.h"
#include "../mediatek/mtk_drm_graphics_base.h"
#endif

#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
#include "../mediatek/mtk_corner_pattern/mtk_data_hw_roundedpattern.h"
#endif
//prize add by lvyuanchuan for lcd hardware info 20220331 start
#if defined(CONFIG_PRIZE_HARDWARE_INFO)
#include "../../../misc/mediatek/hardware_info/hardware_info.h"
extern struct hardware_info current_lcm_info;
#endif
//prize add by lvyuanchuan for lcd hardware info 20220331 end
struct lcm {
	struct device *dev;
	struct drm_panel panel;
	struct backlight_device *backlight;
	struct gpio_desc *reset_gpio;
//prize-add gpio ldo1.8v-pengzhipeng-20220514-start
	struct gpio_desc *ldo18_gpio;
//prize-add gpio ldo1.8v-pengzhipeng-20220514-end
	struct gpio_desc *bias_pos, *bias_neg;

	bool prepared;
	bool enabled;

	int error;
};

#define lcm_dcs_write_seq(ctx, seq...) \
({\
	const u8 d[] = { seq };\
	BUILD_BUG_ON_MSG(ARRAY_SIZE(d) > 64, "DCS sequence too big for stack");\
	lcm_dcs_write(ctx, d, ARRAY_SIZE(d));\
})

#define lcm_dcs_write_seq_static(ctx, seq...) \
({\
	static const u8 d[] = { seq };\
	lcm_dcs_write(ctx, d, ARRAY_SIZE(d));\
})

static inline struct lcm *panel_to_lcm(struct drm_panel *panel)
{
	return container_of(panel, struct lcm, panel);
}

static void lcm_dcs_write(struct lcm *ctx, const void *data, size_t len)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	ssize_t ret;
	char *addr;

	if (ctx->error < 0)
		return;

	addr = (char *)data;
	if ((int)*addr < 0xB0)
		ret = mipi_dsi_dcs_write_buffer(dsi, data, len);
	else
		ret = mipi_dsi_generic_write(dsi, data, len);
	if (ret < 0) {
		dev_info(ctx->dev, "error %zd writing seq: %ph\n", ret, data);
		ctx->error = ret;
	}
}

#ifdef PANEL_SUPPORT_READBACK
static int lcm_dcs_read(struct lcm *ctx, u8 cmd, void *data, size_t len)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	ssize_t ret;

	if (ctx->error < 0)
		return 0;

	ret = mipi_dsi_dcs_read(dsi, cmd, data, len);
	if (ret < 0) {
		dev_info(ctx->dev, "error %d reading dcs seq:(%#x)\n",
		ret, cmd);
		ctx->error = ret;
	}

	return ret;
}

static void lcm_panel_get_data(struct lcm *ctx)
{
	u8 buffer[3] = {0};
	static int ret;

	if (ret == 0) {
		ret = lcm_dcs_read(ctx,  0x0A, buffer, 1);
		dev_info(ctx->dev, "return %d data(0x%08x) to dsi engine\n",
			ret, buffer[0] | (buffer[1] << 8));
	}
}
#endif



static void lcm_panel_init(struct lcm *ctx)
{
	ctx->reset_gpio =
		devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio)) {
		dev_info(ctx->dev, "%s: cannot get reset-gpios %ld\n",
			__func__, PTR_ERR(ctx->reset_gpio));
		return;
	}
	gpiod_set_value(ctx->reset_gpio, 0);
	udelay(15 * 1000);
	gpiod_set_value(ctx->reset_gpio, 1);
	udelay(15 * 1000);
	gpiod_set_value(ctx->reset_gpio, 0);
	udelay(15 * 1000);
	gpiod_set_value(ctx->reset_gpio, 1);
	mdelay(50);
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);
	lcm_dcs_write_seq_static(ctx,0xFF,0x78,0x07,0x01);
lcm_dcs_write_seq_static(ctx,0x00,0x62);
lcm_dcs_write_seq_static(ctx,0x01,0x11);
lcm_dcs_write_seq_static(ctx,0x02,0x00);
lcm_dcs_write_seq_static(ctx,0x03,0x00);
lcm_dcs_write_seq_static(ctx,0x04,0x00);
lcm_dcs_write_seq_static(ctx,0x05,0x00);
lcm_dcs_write_seq_static(ctx,0x06,0x00);
lcm_dcs_write_seq_static(ctx,0x07,0x00);
lcm_dcs_write_seq_static(ctx,0x08,0xA9);
lcm_dcs_write_seq_static(ctx,0x09,0x0A);
lcm_dcs_write_seq_static(ctx,0x0A,0x30);
lcm_dcs_write_seq_static(ctx,0x0B,0x00);
lcm_dcs_write_seq_static(ctx,0x0C,0x00);
lcm_dcs_write_seq_static(ctx,0x0D,0x00);
lcm_dcs_write_seq_static(ctx,0x0E,0x04);
lcm_dcs_write_seq_static(ctx,0x31,0x07);		//GOUTR01      DUMMY
lcm_dcs_write_seq_static(ctx,0x32,0x07);  	//GOUTR02      DUMMY
lcm_dcs_write_seq_static(ctx,0x33,0x07);  	//GOUTR03      DUMMY
lcm_dcs_write_seq_static(ctx,0x34,0x07);  	//GOUTR04      DUMMY
lcm_dcs_write_seq_static(ctx,0x35,0x25);  	//GOUTR05      EN_TOUCH
lcm_dcs_write_seq_static(ctx,0x36,0x02);  	//GOUTR06      CTSW
lcm_dcs_write_seq_static(ctx,0x37,0x41);  	//GOUTR07      CTSW_VCOM
lcm_dcs_write_seq_static(ctx,0x38,0x41);  	//GOUTR08      CTSW_VCOM
lcm_dcs_write_seq_static(ctx,0x39,0x28);  	//GOUTR09      VGL_GL
lcm_dcs_write_seq_static(ctx,0x3A,0x28);  	//GOUTR10      VGL_GL
lcm_dcs_write_seq_static(ctx,0x3B,0x30);  	//GOUTR11      MUXB1
lcm_dcs_write_seq_static(ctx,0x3C,0x2F);  	//GOUTR12      MUXG1
lcm_dcs_write_seq_static(ctx,0x3D,0x2E);  	//GOUTR13      MUXR1
lcm_dcs_write_seq_static(ctx,0x3E,0x11);  	//GOUTR14      CLK4L
lcm_dcs_write_seq_static(ctx,0x3F,0x10);  	//GOUTR15      CLK3L
lcm_dcs_write_seq_static(ctx,0x40,0x13);  	//GOUTR16      CLK2L
lcm_dcs_write_seq_static(ctx,0x41,0x12);  	//GOUTR17      CLK1L
lcm_dcs_write_seq_static(ctx,0x42,0x2C);  	//GOUTR18      RESETL
lcm_dcs_write_seq_static(ctx,0x43,0x40);  	//GOUTR19      VGH_GL
lcm_dcs_write_seq_static(ctx,0x44,0x40);  	//GOUTR20      VGH_GL
lcm_dcs_write_seq_static(ctx,0x45,0x01);  	//GOUTR21      CNL
lcm_dcs_write_seq_static(ctx,0x46,0x00);  	//GOUTR22      CNBL
lcm_dcs_write_seq_static(ctx,0x47,0x09);  	//GOUTR23      STVL2
lcm_dcs_write_seq_static(ctx,0x48,0x08);  	//GOUTR24      STVL1	
lcm_dcs_write_seq_static(ctx,0x49,0x07);		//GOUTL01      DUMMY
lcm_dcs_write_seq_static(ctx,0x4A,0x07);  	//GOUTL02      DUMMY
lcm_dcs_write_seq_static(ctx,0x4B,0x07);  	//GOUTL03      DUMMY
lcm_dcs_write_seq_static(ctx,0x4C,0x07);  	//GOUTL04      DUMMY
lcm_dcs_write_seq_static(ctx,0x4D,0x25);  	//GOUTL05      EN_TOUCH
lcm_dcs_write_seq_static(ctx,0x4E,0x02);  	//GOUTL06      CTSW
lcm_dcs_write_seq_static(ctx,0x4F,0x41);  	//GOUTL07      CTSW_VCOM
lcm_dcs_write_seq_static(ctx,0x50,0x41);  	//GOUTL08      CTSW_VCOM
lcm_dcs_write_seq_static(ctx,0x51,0x28);  	//GOUTL09      VGL_GR
lcm_dcs_write_seq_static(ctx,0x52,0x28);  	//GOUTL10      VGL_GR
lcm_dcs_write_seq_static(ctx,0x53,0x30);  	//GOUTL11      MUXB1
lcm_dcs_write_seq_static(ctx,0x54,0x2F);  	//GOUTL12      MUXG1
lcm_dcs_write_seq_static(ctx,0x55,0x2E);  	//GOUTL13      MUXR1
lcm_dcs_write_seq_static(ctx,0x56,0x11);  	//GOUTL14      CLK4R
lcm_dcs_write_seq_static(ctx,0x57,0x10);  	//GOUTL15      CLK3R
lcm_dcs_write_seq_static(ctx,0x58,0x13);  	//GOUTL16      CLK2R
lcm_dcs_write_seq_static(ctx,0x59,0x12);  	//GOUTL17      CLK1R
lcm_dcs_write_seq_static(ctx,0x5A,0x2C);  	//GOUTL18      RESETR
lcm_dcs_write_seq_static(ctx,0x5B,0x40);  	//GOUTL19      VGH_GR
lcm_dcs_write_seq_static(ctx,0x5C,0x40);  	//GOUTL20      VGH_GR
lcm_dcs_write_seq_static(ctx,0x5D,0x01);  	//GOUTL21      CNR
lcm_dcs_write_seq_static(ctx,0x5E,0x00);  	//GOUTL22      CNBR
lcm_dcs_write_seq_static(ctx,0x5F,0x09);  	//GOUTL23      STVR2
lcm_dcs_write_seq_static(ctx,0x60,0x08);  	//GOUTL24      STVR1						
lcm_dcs_write_seq_static(ctx,0x61,0x07);  	//GOUTR01      DUMMY
lcm_dcs_write_seq_static(ctx,0x62,0x07);  	//GOUTR02      DUMMY
lcm_dcs_write_seq_static(ctx,0x63,0x07);  	//GOUTR03      DUMMY
lcm_dcs_write_seq_static(ctx,0x64,0x07);  	//GOUTR04      DUMMY
lcm_dcs_write_seq_static(ctx,0x65,0x25);  	//GOUTR05      EN_TOUCH
lcm_dcs_write_seq_static(ctx,0x66,0x02);  	//GOUTR06      CTSW
lcm_dcs_write_seq_static(ctx,0x67,0x41);  	//GOUTR07      CTSW_VCOM
lcm_dcs_write_seq_static(ctx,0x68,0x41);  	//GOUTR08      CTSW_VCOM
lcm_dcs_write_seq_static(ctx,0x69,0x28);  	//GOUTR09      VGL_GL
lcm_dcs_write_seq_static(ctx,0x6A,0x28);  	//GOUTR10      VGL_GL
lcm_dcs_write_seq_static(ctx,0x6B,0x30);  	//GOUTR11      MUXB1
lcm_dcs_write_seq_static(ctx,0x6C,0x2F);  	//GOUTR12      MUXG1
lcm_dcs_write_seq_static(ctx,0x6D,0x2E);  	//GOUTR13      MUXR1
lcm_dcs_write_seq_static(ctx,0x6E,0x11);  	//GOUTR14      CLK4L
lcm_dcs_write_seq_static(ctx,0x6F,0x10);  	//GOUTR15      CLK3L
lcm_dcs_write_seq_static(ctx,0x70,0x13);  	//GOUTR16      CLK2L
lcm_dcs_write_seq_static(ctx,0x71,0x12);  	//GOUTR17      CLK1L
lcm_dcs_write_seq_static(ctx,0x72,0x2C);  	//GOUTR18      RESETL
lcm_dcs_write_seq_static(ctx,0x73,0x40);  	//GOUTR19      VGH_GL
lcm_dcs_write_seq_static(ctx,0x74,0x40);  	//GOUTR20      VGH_GL
lcm_dcs_write_seq_static(ctx,0x75,0x01);  	//GOUTR21      CNL
lcm_dcs_write_seq_static(ctx,0x76,0x00);  	//GOUTR22      CNBL
lcm_dcs_write_seq_static(ctx,0x77,0x09);  	//GOUTR23      STVL2
lcm_dcs_write_seq_static(ctx,0x78,0x08);  	//GOUTR24      STVL1                                   
lcm_dcs_write_seq_static(ctx,0x79,0x07);   	//GOUTL01 	   DUMMY
lcm_dcs_write_seq_static(ctx,0x7A,0x07);   	//GOUTL02      DUMMY
lcm_dcs_write_seq_static(ctx,0x7B,0x07);   	//GOUTL03      DUMMY
lcm_dcs_write_seq_static(ctx,0x7C,0x07);   	//GOUTL04      DUMMY
lcm_dcs_write_seq_static(ctx,0x7D,0x25);   	//GOUTL05      EN_TOUCH
lcm_dcs_write_seq_static(ctx,0x7E,0x02);   	//GOUTL06      CTSW
lcm_dcs_write_seq_static(ctx,0x7F,0x41);   	//GOUTL07      CTSW_VCOM
lcm_dcs_write_seq_static(ctx,0x80,0x41);   	//GOUTL08      CTSW_VCOM
lcm_dcs_write_seq_static(ctx,0x81,0x28);   	//GOUTL09      VGL_GR
lcm_dcs_write_seq_static(ctx,0x82,0x28);   	//GOUTL10      VGL_GR
lcm_dcs_write_seq_static(ctx,0x83,0x30);   	//GOUTL11      MUXB1
lcm_dcs_write_seq_static(ctx,0x84,0x2F);   	//GOUTL12      MUXG1
lcm_dcs_write_seq_static(ctx,0x85,0x2E);   	//GOUTL13      MUXR1
lcm_dcs_write_seq_static(ctx,0x86,0x11);   	//GOUTL14      CLK4R
lcm_dcs_write_seq_static(ctx,0x87,0x10);   	//GOUTL15      CLK3R
lcm_dcs_write_seq_static(ctx,0x88,0x13);   	//GOUTL16      CLK2R
lcm_dcs_write_seq_static(ctx,0x89,0x12);   	//GOUTL17      CLK1R
lcm_dcs_write_seq_static(ctx,0x8A,0x2C);   	//GOUTL18      RESETR
lcm_dcs_write_seq_static(ctx,0x8B,0x40);   	//GOUTL19      VGH_GR
lcm_dcs_write_seq_static(ctx,0x8C,0x40);   	//GOUTL20      VGH_GR
lcm_dcs_write_seq_static(ctx,0x8D,0x01);   	//GOUTL21      CNR
lcm_dcs_write_seq_static(ctx,0x8E,0x00);   	//GOUTL22      CNBR
lcm_dcs_write_seq_static(ctx,0x8F,0x09);   	//GOUTL23      STVR2
lcm_dcs_write_seq_static(ctx,0x90,0x08);   	//GOUTL24      STVR1
lcm_dcs_write_seq_static(ctx,0xA0,0x4C);
lcm_dcs_write_seq_static(ctx,0xA1,0x4A);
lcm_dcs_write_seq_static(ctx,0xA2,0x00);
lcm_dcs_write_seq_static(ctx,0xA3,0x00);
lcm_dcs_write_seq_static(ctx,0xA7,0x10);
lcm_dcs_write_seq_static(ctx,0xAA,0x00);
lcm_dcs_write_seq_static(ctx,0xAB,0x00);
lcm_dcs_write_seq_static(ctx,0xAC,0x00);
lcm_dcs_write_seq_static(ctx,0xAE,0x00);
lcm_dcs_write_seq_static(ctx,0xB0,0x20);
lcm_dcs_write_seq_static(ctx,0xB1,0x00);
lcm_dcs_write_seq_static(ctx,0xB2,0x02);
lcm_dcs_write_seq_static(ctx,0xB3,0x04);
lcm_dcs_write_seq_static(ctx,0xB4,0x05);
lcm_dcs_write_seq_static(ctx,0xB5,0x00);
lcm_dcs_write_seq_static(ctx,0xB6,0x00);
lcm_dcs_write_seq_static(ctx,0xB7,0x00);
lcm_dcs_write_seq_static(ctx,0xB8,0x00);
lcm_dcs_write_seq_static(ctx,0xC0,0x0C);
lcm_dcs_write_seq_static(ctx,0xC1,0x1E);  //60180   20210422
lcm_dcs_write_seq_static(ctx,0xC2,0x00);
lcm_dcs_write_seq_static(ctx,0xC3,0x00);
lcm_dcs_write_seq_static(ctx,0xC4,0x00);
lcm_dcs_write_seq_static(ctx,0xC5,0x2B);
lcm_dcs_write_seq_static(ctx,0xC6,0x00);
lcm_dcs_write_seq_static(ctx,0xC7,0x28);
lcm_dcs_write_seq_static(ctx,0xC8,0x00);
lcm_dcs_write_seq_static(ctx,0xC9,0x00);
lcm_dcs_write_seq_static(ctx,0xCA,0x01);
lcm_dcs_write_seq_static(ctx,0xD0,0x01);
lcm_dcs_write_seq_static(ctx,0xD1,0x00);
lcm_dcs_write_seq_static(ctx,0xD2,0x10);
lcm_dcs_write_seq_static(ctx,0xD3,0x41);
lcm_dcs_write_seq_static(ctx,0xD4,0x89);
lcm_dcs_write_seq_static(ctx,0xD5,0x06);
lcm_dcs_write_seq_static(ctx,0xD6,0x49);
lcm_dcs_write_seq_static(ctx,0xD7,0x40);
lcm_dcs_write_seq_static(ctx,0xD8,0x09);
lcm_dcs_write_seq_static(ctx,0xD9,0x96);
lcm_dcs_write_seq_static(ctx,0xDA,0xAA);
lcm_dcs_write_seq_static(ctx,0xDB,0xAA);
lcm_dcs_write_seq_static(ctx,0xDC,0x8A);
lcm_dcs_write_seq_static(ctx,0xDD,0xA8);
lcm_dcs_write_seq_static(ctx,0xDE,0x05);
lcm_dcs_write_seq_static(ctx,0xDF,0x42);
lcm_dcs_write_seq_static(ctx,0xE0,0x1E);
lcm_dcs_write_seq_static(ctx,0xE1,0x68);
lcm_dcs_write_seq_static(ctx,0xE2,0x05);
lcm_dcs_write_seq_static(ctx,0xE3,0x11);		//91
lcm_dcs_write_seq_static(ctx,0xE4,0x42);		//73
lcm_dcs_write_seq_static(ctx,0xE5,0x4B);		//4A
lcm_dcs_write_seq_static(ctx,0xE6,0x2A);
lcm_dcs_write_seq_static(ctx,0xE7,0x0C);
lcm_dcs_write_seq_static(ctx,0xE8,0x00);
lcm_dcs_write_seq_static(ctx,0xE9,0x00);
lcm_dcs_write_seq_static(ctx,0xEA,0x00);
lcm_dcs_write_seq_static(ctx,0xEB,0x00);
lcm_dcs_write_seq_static(ctx,0xEC,0x80);
lcm_dcs_write_seq_static(ctx,0xED,0x56);
lcm_dcs_write_seq_static(ctx,0xEE,0x00);
lcm_dcs_write_seq_static(ctx,0xEF,0x32);
lcm_dcs_write_seq_static(ctx,0xF0,0x00);
lcm_dcs_write_seq_static(ctx,0xF1,0xC0);
lcm_dcs_write_seq_static(ctx,0xF2,0xFF);
lcm_dcs_write_seq_static(ctx,0xF3,0xFF);
lcm_dcs_write_seq_static(ctx,0xF4,0x54);
lcm_dcs_write_seq_static(ctx,0xFF,0x78,0x07,0x11);
lcm_dcs_write_seq_static(ctx,0x00,0x00);
lcm_dcs_write_seq_static(ctx,0x01,0x03);		//0316 
lcm_dcs_write_seq_static(ctx,0x38,0x00);
lcm_dcs_write_seq_static(ctx,0x39,0x04);		//0316 
lcm_dcs_write_seq_static(ctx,0x18,0x0F);  //120180 20210421
lcm_dcs_write_seq_static(ctx,0x50,0x14);  //90180   20210422
lcm_dcs_write_seq_static(ctx,0xFF,0x78,0x07,0x02);
lcm_dcs_write_seq_static(ctx,0x40,0x05);     	//60_t8_de
lcm_dcs_write_seq_static(ctx,0x41,0x00);     	//60_t7p_de
lcm_dcs_write_seq_static(ctx,0x42,0x07);     	//60_t9_de
lcm_dcs_write_seq_static(ctx,0x43,0x2F);     	//60_t7_de  CKH ASW2 800ns ??920ns
lcm_dcs_write_seq_static(ctx,0x53,0x07);     	//60 sdt
lcm_dcs_write_seq_static(ctx,0x1B,0x00);      //FR_sel 00:120_180 01:90_180 02:60_180	
lcm_dcs_write_seq_static(ctx,0x46,0x22);     	//DUMMY CKH  0924
lcm_dcs_write_seq_static(ctx,0x47,0x03);		//CKH CONNECT	
lcm_dcs_write_seq_static(ctx,0x4F,0x01);		//CKH CONNECT	
lcm_dcs_write_seq_static(ctx,0x76,0x1F);		//save power SRC bias through rate	
lcm_dcs_write_seq_static(ctx,0x80,0x35);		//save power SRC bias current	
lcm_dcs_write_seq_static(ctx,0x06,0x6A);     	//BIST RTN=6.688us	
lcm_dcs_write_seq_static(ctx,0x08,0x00);     	//BIST RTN[10:8]	
lcm_dcs_write_seq_static(ctx,0x0E,0x27);     	//BIST VBP	
lcm_dcs_write_seq_static(ctx,0x0F,0x28);     	//BIST VFP	
lcm_dcs_write_seq_static(ctx,0xFF,0x78,0x07,0x12);
lcm_dcs_write_seq_static(ctx,0x48,0x05);     	//90_t8_de
lcm_dcs_write_seq_static(ctx,0x49,0x00);     	//90_t7p_de
lcm_dcs_write_seq_static(ctx,0x4A,0x08);     	//90_t9_de
lcm_dcs_write_seq_static(ctx,0x4B,0x1A);     	//90_t7_de  CKH ASW2 800ns ??920ns
lcm_dcs_write_seq_static(ctx,0x4E,0x08);     	//90 sdt
lcm_dcs_write_seq_static(ctx,0x52,0x1F);		//save power SRC bias through rate
lcm_dcs_write_seq_static(ctx,0x53,0x25);		//save power SRC bias current
lcm_dcs_write_seq_static(ctx,0xC8,0x47);     	//BIST RTN=4.5us	
lcm_dcs_write_seq_static(ctx,0xC9,0x00);     	//BIST RTN[10:8]	
lcm_dcs_write_seq_static(ctx,0xCA,0x20);     	//BIST VBP	
lcm_dcs_write_seq_static(ctx,0xCB,0x20);     	//BIST VFP	
lcm_dcs_write_seq_static(ctx,0x10,0x05);     	//120_t8_de
lcm_dcs_write_seq_static(ctx,0x11,0x00);     	//120_t7p_de     
lcm_dcs_write_seq_static(ctx,0x12,0x08);     	//120_t9_de 
lcm_dcs_write_seq_static(ctx,0x13,0x14);     	//120_t7_de
lcm_dcs_write_seq_static(ctx,0x16,0x08);      //SDT
lcm_dcs_write_seq_static(ctx,0x17,0x00);      //0402 eq
lcm_dcs_write_seq_static(ctx,0x1A,0x1F);		//save power SRC bias through rate
lcm_dcs_write_seq_static(ctx,0x1B,0x25);		//save power SRC bias current
lcm_dcs_write_seq_static(ctx,0xC0,0x35);     	//BIST RTN=3.375us	
lcm_dcs_write_seq_static(ctx,0xC1,0x00);     	//BIST RTN[10:8]	
lcm_dcs_write_seq_static(ctx,0xC2,0x29);     	//BIST VBP	
lcm_dcs_write_seq_static(ctx,0xC3,0x20);     	//BIST VFP	
lcm_dcs_write_seq_static(ctx,0xFF,0x78,0x07,0x05);
lcm_dcs_write_seq_static(ctx,0x1B,0x00);		
lcm_dcs_write_seq_static(ctx,0x1C,0x87);		//120_VCOM=-0.1V
lcm_dcs_write_seq_static(ctx,0x1D,0x00);		
lcm_dcs_write_seq_static(ctx,0x1E,0x87);		//90_VCOM=-0.1V
lcm_dcs_write_seq_static(ctx,0x1F,0x00);		
lcm_dcs_write_seq_static(ctx,0x20,0x91);		//60_VCOM=-0.1V	
lcm_dcs_write_seq_static(ctx,0x72,0x7E);		//VGH=11V
lcm_dcs_write_seq_static(ctx,0x74,0x56);		//VGL=-9V
lcm_dcs_write_seq_static(ctx,0x76,0x79);		//VGHO=10V
lcm_dcs_write_seq_static(ctx,0x7A,0x51);		//VGLO=-8V		
lcm_dcs_write_seq_static(ctx,0x7B,0x79);        //88 GVDDP=4.9V    20210420
lcm_dcs_write_seq_static(ctx,0x7C,0x79);        //88 GVDDN=-4.9V   20210420
lcm_dcs_write_seq_static(ctx,0x46,0x55);		//PWR_TCON_VGHO_EN
lcm_dcs_write_seq_static(ctx,0x47,0x75);		//PWR_TCON_VGLO_EN
lcm_dcs_write_seq_static(ctx,0xB5,0x55);		//PWR_D2A_HVREG_VGHO_EN
lcm_dcs_write_seq_static(ctx,0xB7,0x75);		//PWR_D2A_HVREG_VGLO_EN
lcm_dcs_write_seq_static(ctx,0xC6,0x1B);
lcm_dcs_write_seq_static(ctx,0x56,0xFF);      //0915 TD P2P
lcm_dcs_write_seq_static(ctx,0xFF,0x78,0x07,0x06);
lcm_dcs_write_seq_static(ctx,0xC0,0x6C);		//Res=1080*2412
lcm_dcs_write_seq_static(ctx,0xC1,0x19);		//Res=1080*2412
lcm_dcs_write_seq_static(ctx,0xC3,0x06);		//SS_REG
lcm_dcs_write_seq_static(ctx,0x13,0x13);		//force otp DDI
lcm_dcs_write_seq_static(ctx,0xFF,0x78,0x07,0x07);
lcm_dcs_write_seq_static(ctx,0x29,0xCF);
lcm_dcs_write_seq_static(ctx,0xFF,0x78,0x07,0x17);	//Slice high=12
lcm_dcs_write_seq_static(ctx,0x20,0x00,0x00,0x00,0x00,0x00,0x11,0x00,0x00,0x89,0x30,0x80,0x09,0x6c,0x04,0x38,0x00,0x0c,0x02,0x1c,0x02,0x1c,0x00,0xaa,0x02,0x0e,0x00,0x20,0x00,0x43,0x00,0x07,
		   0x00,0x0c,0x08,0xbb,0x08,0x7a,0x18,0x00,0x1b,0xa0,0x03,0x0c,0x20,0x00,0x06,0x0b,0x0b,0x33,0x0e,0x1c,0x2a,0x38,0x46,0x54,0x62,0x69,0x70,0x77,0x79,0x7b,0x7d,
		   0x7e,0x01,0x02,0x01,0x00,0x09,0x40,0x09,0xbe,0x19,0xfc,0x19,0xfa,0x19,0xf8,0x1a,0x38,0x1a,0x78,0x1a,0xb6,0x2a,0xf6,0x2b,0x34,0x2b,0x74,0x3b,0x74,0x6b,0xf4,
		   0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
		   0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00);
lcm_dcs_write_seq_static(ctx,0xFF,0x78,0x07,0x08);                //20210513               		
lcm_dcs_write_seq_static(ctx,0xE0,0x00,0x00,0x19,0x40,0x00,0x7C,0xA7,0xCB,0x15,0x04,0x30,0x70,0x25,0xA2,0xEC,0x27,0x2A,0x61,0xA4,0xCC,0x3F,0x02,0x28,0x51,0x3F,0x6B,0x8C,0xBB,0x0F,0xD8,0xD9);									
lcm_dcs_write_seq_static(ctx,0xE1,0x00,0x00,0x19,0x40,0x00,0x7C,0xA7,0xCB,0x15,0x04,0x30,0x70,0x25,0xA2,0xEC,0x27,0x2A,0x61,0xA4,0xCC,0x3F,0x02,0x28,0x51,0x3F,0x6B,0x8C,0xBB,0x0F,0xD8,0xD9);	
lcm_dcs_write_seq_static(ctx,0xFF,0x78,0x07,0x0B);	// AUTOTRIM
lcm_dcs_write_seq_static(ctx,0x94,0x88);     	//VDO_DIV_SEL (Frame)	
lcm_dcs_write_seq_static(ctx,0x95,0x1E);     	//VDO_CNT_IDEAL[12:0]	
lcm_dcs_write_seq_static(ctx,0x96,0x06);     	//UB [4:0]	
lcm_dcs_write_seq_static(ctx,0x97,0x06);     	//LB [4:0]	
lcm_dcs_write_seq_static(ctx,0x98,0xCB);     	//Keep trim code range MAX[7:0]	
lcm_dcs_write_seq_static(ctx,0x99,0xCB);     	//Keep trim code range MIN[7:0]	
lcm_dcs_write_seq_static(ctx,0x9A,0x06);     	//VDO_LN_DIV_SEL, cnt_line	
lcm_dcs_write_seq_static(ctx,0x9B,0xAC);     	//VDO_CNT_IDEAL[12:0]	
lcm_dcs_write_seq_static(ctx,0x9C,0x05);     	//UB [4:0]	
lcm_dcs_write_seq_static(ctx,0x9D,0x05);     	//LB [4:0]	
lcm_dcs_write_seq_static(ctx,0x9E,0xA7);     	//Keep trim code range MAX[7:0]	
lcm_dcs_write_seq_static(ctx,0x9F,0xA7);     	//Keep trim code range MIN[7:0]	
lcm_dcs_write_seq_static(ctx,0xAA,0x02);     	//D[4]: Trim_ln_num	
lcm_dcs_write_seq_static(ctx,0xC0,0x84);     	//VDO_DIV_SEL (Frame)	
lcm_dcs_write_seq_static(ctx,0xC1,0x11);     	//VDO_CNT_IDEAL[12:0]	
lcm_dcs_write_seq_static(ctx,0xC2,0x03);     	//UB [4:0]	
lcm_dcs_write_seq_static(ctx,0xC3,0x03);     	//LB [4:0]	
lcm_dcs_write_seq_static(ctx,0xC4,0x65);     	//Keep trim code range MAX[7:0]	
lcm_dcs_write_seq_static(ctx,0xC5,0x65);     	//Keep trim code range MIN[7:0]	
lcm_dcs_write_seq_static(ctx,0xD2,0x03);     	//VDO_LN_DIV_SEL, cnt_line	
lcm_dcs_write_seq_static(ctx,0xD3,0x58);     	//VDO_CNT_IDEAL[12:0]	
lcm_dcs_write_seq_static(ctx,0xD4,0x03);     	//UB [4:0]	
lcm_dcs_write_seq_static(ctx,0xD5,0x03);     	//LB [4:0]	
lcm_dcs_write_seq_static(ctx,0xD6,0x53);     	//Keep trim code range MAX[7:0]	
lcm_dcs_write_seq_static(ctx,0xD7,0x53);     	//Keep trim code range MIN[7:0]	
lcm_dcs_write_seq_static(ctx,0xAA,0x02);     	//D[4]: Trim_ln_num	
lcm_dcs_write_seq_static(ctx,0xC6,0x85);     	//VDO_DIV_SEL (Frame)	
lcm_dcs_write_seq_static(ctx,0xC7,0x6B);     	//VDO_CNT_IDEAL[12:0]	
lcm_dcs_write_seq_static(ctx,0xC8,0x04);     	//UB [4:0]	
lcm_dcs_write_seq_static(ctx,0xC9,0x04);     	//LB [4:0]	
lcm_dcs_write_seq_static(ctx,0xCA,0x87);     	//Keep trim code range MAX[7:0]	
lcm_dcs_write_seq_static(ctx,0xCB,0x87);     	//Keep trim code range MIN[7:0]	
lcm_dcs_write_seq_static(ctx,0xD8,0x04);     	//VDO_LN_DIV_SEL, cnt_line	
lcm_dcs_write_seq_static(ctx,0xD9,0x74);     	//VDO_CNT_IDEAL[12:0]	
lcm_dcs_write_seq_static(ctx,0xDA,0x03);     	//UB [4:0]	
lcm_dcs_write_seq_static(ctx,0xDB,0x03);     	//LB [4:0]	
lcm_dcs_write_seq_static(ctx,0xDC,0x6F);     	//Keep trim code range MAX[7:0]	
lcm_dcs_write_seq_static(ctx,0xDD,0x6F);     	//Keep trim code range MIN[7:0]	
lcm_dcs_write_seq_static(ctx,0xAA,0x02);     	//D[4]: Trim_ln_num	
lcm_dcs_write_seq_static(ctx,0xAB,0xE0);     	//O	SC auto trim en		
lcm_dcs_write_seq_static(ctx,0xFF,0x78,0x07,0x0C);//TP MODULATION  20210519
lcm_dcs_write_seq_static(ctx,0x00,0x3F);
lcm_dcs_write_seq_static(ctx,0x01,0xD3);
lcm_dcs_write_seq_static(ctx,0x02,0x3F);
lcm_dcs_write_seq_static(ctx,0x03,0xCF);
lcm_dcs_write_seq_static(ctx,0x04,0x3F);
lcm_dcs_write_seq_static(ctx,0x05,0xD6);
lcm_dcs_write_seq_static(ctx,0x06,0x3F);
lcm_dcs_write_seq_static(ctx,0x07,0xD1);
lcm_dcs_write_seq_static(ctx,0x08,0x3F);
lcm_dcs_write_seq_static(ctx,0x09,0xC9);
lcm_dcs_write_seq_static(ctx,0x0A,0x3F);
lcm_dcs_write_seq_static(ctx,0x0B,0xDF);
lcm_dcs_write_seq_static(ctx,0x0C,0x3F);
lcm_dcs_write_seq_static(ctx,0x0D,0xDD);
lcm_dcs_write_seq_static(ctx,0x0E,0x3F);
lcm_dcs_write_seq_static(ctx,0x0F,0xD9);
lcm_dcs_write_seq_static(ctx,0x10,0x3F);
lcm_dcs_write_seq_static(ctx,0x11,0xD8);
lcm_dcs_write_seq_static(ctx,0x12,0x3F);
lcm_dcs_write_seq_static(ctx,0x13,0xE4);
lcm_dcs_write_seq_static(ctx,0x14,0x3F);
lcm_dcs_write_seq_static(ctx,0x15,0xCD);
lcm_dcs_write_seq_static(ctx,0x16,0x3F);
lcm_dcs_write_seq_static(ctx,0x17,0xD4);
lcm_dcs_write_seq_static(ctx,0x18,0x3F);
lcm_dcs_write_seq_static(ctx,0x19,0xE1);
lcm_dcs_write_seq_static(ctx,0x1A,0x3F);
lcm_dcs_write_seq_static(ctx,0x1B,0xD0);
lcm_dcs_write_seq_static(ctx,0x1C,0x3F);
lcm_dcs_write_seq_static(ctx,0x1D,0xD5);
lcm_dcs_write_seq_static(ctx,0x1E,0x3F);
lcm_dcs_write_seq_static(ctx,0x1F,0xE0);
lcm_dcs_write_seq_static(ctx,0x20,0x3F);
lcm_dcs_write_seq_static(ctx,0x21,0xDC);
lcm_dcs_write_seq_static(ctx,0x22,0x3F);
lcm_dcs_write_seq_static(ctx,0x23,0xE3);
lcm_dcs_write_seq_static(ctx,0x24,0x3F);
lcm_dcs_write_seq_static(ctx,0x25,0xCB);
lcm_dcs_write_seq_static(ctx,0x26,0x3F);
lcm_dcs_write_seq_static(ctx,0x27,0xDA);
lcm_dcs_write_seq_static(ctx,0x28,0x3F);
lcm_dcs_write_seq_static(ctx,0x29,0xDE);
lcm_dcs_write_seq_static(ctx,0x2A,0x3F);
lcm_dcs_write_seq_static(ctx,0x2B,0xC8);
lcm_dcs_write_seq_static(ctx,0x2C,0x3F);
lcm_dcs_write_seq_static(ctx,0x2D,0xE2);
lcm_dcs_write_seq_static(ctx,0x2E,0x3F);
lcm_dcs_write_seq_static(ctx,0x2F,0xCA);
lcm_dcs_write_seq_static(ctx,0x30,0x3F);
lcm_dcs_write_seq_static(ctx,0x31,0xDB);
lcm_dcs_write_seq_static(ctx,0x32,0x3F);
lcm_dcs_write_seq_static(ctx,0x33,0xD7);
lcm_dcs_write_seq_static(ctx,0x34,0x3F);
lcm_dcs_write_seq_static(ctx,0x35,0xCC);
lcm_dcs_write_seq_static(ctx,0x36,0x3F);
lcm_dcs_write_seq_static(ctx,0x37,0xCE);
lcm_dcs_write_seq_static(ctx,0x38,0x3F);
lcm_dcs_write_seq_static(ctx,0x39,0xD2);
lcm_dcs_write_seq_static(ctx,0x40,0x3F);
lcm_dcs_write_seq_static(ctx,0x41,0x68);
lcm_dcs_write_seq_static(ctx,0x42,0x3F);
lcm_dcs_write_seq_static(ctx,0x43,0x51);
lcm_dcs_write_seq_static(ctx,0x44,0x3F);
lcm_dcs_write_seq_static(ctx,0x45,0x51);
lcm_dcs_write_seq_static(ctx,0x46,0x3F);
lcm_dcs_write_seq_static(ctx,0x47,0x58);
lcm_dcs_write_seq_static(ctx,0x48,0x3F);
lcm_dcs_write_seq_static(ctx,0x49,0x5F);
lcm_dcs_write_seq_static(ctx,0x4A,0x3F);
lcm_dcs_write_seq_static(ctx,0x4B,0x62);
lcm_dcs_write_seq_static(ctx,0x4C,0x3F);
lcm_dcs_write_seq_static(ctx,0x4D,0x5B);
lcm_dcs_write_seq_static(ctx,0x4E,0x3F);
lcm_dcs_write_seq_static(ctx,0x4F,0x67);
lcm_dcs_write_seq_static(ctx,0x50,0x3F);
lcm_dcs_write_seq_static(ctx,0x51,0x5E);
lcm_dcs_write_seq_static(ctx,0x52,0x3F);
lcm_dcs_write_seq_static(ctx,0x53,0x5A);
lcm_dcs_write_seq_static(ctx,0x54,0x3F);
lcm_dcs_write_seq_static(ctx,0x55,0x5C);
lcm_dcs_write_seq_static(ctx,0x56,0x3F);
lcm_dcs_write_seq_static(ctx,0x57,0x6C);
lcm_dcs_write_seq_static(ctx,0x58,0x3F);
lcm_dcs_write_seq_static(ctx,0x59,0x57);
lcm_dcs_write_seq_static(ctx,0x5A,0x3F);
lcm_dcs_write_seq_static(ctx,0x5B,0x60);
lcm_dcs_write_seq_static(ctx,0x5C,0x3F);
lcm_dcs_write_seq_static(ctx,0x5D,0x6B);
lcm_dcs_write_seq_static(ctx,0x5E,0x3F);
lcm_dcs_write_seq_static(ctx,0x5F,0x59);
lcm_dcs_write_seq_static(ctx,0x60,0x3F);
lcm_dcs_write_seq_static(ctx,0x61,0x66);
lcm_dcs_write_seq_static(ctx,0x62,0x3F);
lcm_dcs_write_seq_static(ctx,0x63,0x56);
lcm_dcs_write_seq_static(ctx,0x64,0x3F);
lcm_dcs_write_seq_static(ctx,0x65,0x61);
lcm_dcs_write_seq_static(ctx,0x66,0x3F);
lcm_dcs_write_seq_static(ctx,0x67,0x6A);
lcm_dcs_write_seq_static(ctx,0x68,0x3F);
lcm_dcs_write_seq_static(ctx,0x69,0x63);
lcm_dcs_write_seq_static(ctx,0x6A,0x3F);
lcm_dcs_write_seq_static(ctx,0x6B,0x69);
lcm_dcs_write_seq_static(ctx,0x6C,0x3F);
lcm_dcs_write_seq_static(ctx,0x6D,0x64);
lcm_dcs_write_seq_static(ctx,0x6E,0x3F);
lcm_dcs_write_seq_static(ctx,0x6F,0x50);
lcm_dcs_write_seq_static(ctx,0x70,0x3F);
lcm_dcs_write_seq_static(ctx,0x71,0x5D);
lcm_dcs_write_seq_static(ctx,0x72,0x3F);
lcm_dcs_write_seq_static(ctx,0x73,0x65);
lcm_dcs_write_seq_static(ctx,0x74,0x3F);
lcm_dcs_write_seq_static(ctx,0x75,0x55);
lcm_dcs_write_seq_static(ctx,0x76,0x3F);
lcm_dcs_write_seq_static(ctx,0x77,0x53);
lcm_dcs_write_seq_static(ctx,0x78,0x3F);
lcm_dcs_write_seq_static(ctx,0x79,0x52);
lcm_dcs_write_seq_static(ctx,0x80,0x2E);
lcm_dcs_write_seq_static(ctx,0x81,0xA8);
lcm_dcs_write_seq_static(ctx,0x82,0x2C);
lcm_dcs_write_seq_static(ctx,0x83,0x96);
lcm_dcs_write_seq_static(ctx,0x84,0x2E);
lcm_dcs_write_seq_static(ctx,0x85,0xA6);
lcm_dcs_write_seq_static(ctx,0x86,0x2E);
lcm_dcs_write_seq_static(ctx,0x87,0xAC);
lcm_dcs_write_seq_static(ctx,0x88,0x2D);
lcm_dcs_write_seq_static(ctx,0x89,0x9D);
lcm_dcs_write_seq_static(ctx,0x8A,0x2D);
lcm_dcs_write_seq_static(ctx,0x8B,0x9B);
lcm_dcs_write_seq_static(ctx,0x8C,0x2E);
lcm_dcs_write_seq_static(ctx,0x8D,0xB0);
lcm_dcs_write_seq_static(ctx,0x8E,0x2C);
lcm_dcs_write_seq_static(ctx,0x8F,0x98);
lcm_dcs_write_seq_static(ctx,0x90,0x2E);
lcm_dcs_write_seq_static(ctx,0x91,0xAF);
lcm_dcs_write_seq_static(ctx,0x92,0x2D);
lcm_dcs_write_seq_static(ctx,0x93,0xA0);
lcm_dcs_write_seq_static(ctx,0x94,0x2E);
lcm_dcs_write_seq_static(ctx,0x95,0xA3);
lcm_dcs_write_seq_static(ctx,0x96,0x2E);
lcm_dcs_write_seq_static(ctx,0x97,0xAD);
lcm_dcs_write_seq_static(ctx,0x98,0x2E);
lcm_dcs_write_seq_static(ctx,0x99,0xAE);
lcm_dcs_write_seq_static(ctx,0x9A,0x2C);
lcm_dcs_write_seq_static(ctx,0x9B,0x97);
lcm_dcs_write_seq_static(ctx,0x9C,0x2E);
lcm_dcs_write_seq_static(ctx,0x9D,0xAB);
lcm_dcs_write_seq_static(ctx,0x9E,0x2E);
lcm_dcs_write_seq_static(ctx,0x9F,0xA9);
lcm_dcs_write_seq_static(ctx,0xA0,0x2E);
lcm_dcs_write_seq_static(ctx,0xA1,0xA7);
lcm_dcs_write_seq_static(ctx,0xA2,0x2E);
lcm_dcs_write_seq_static(ctx,0xA3,0xA5);
lcm_dcs_write_seq_static(ctx,0xA4,0x2D);
lcm_dcs_write_seq_static(ctx,0xA5,0x9E);
lcm_dcs_write_seq_static(ctx,0xA6,0x2E);
lcm_dcs_write_seq_static(ctx,0xA7,0xAA);
lcm_dcs_write_seq_static(ctx,0xA8,0x2E);
lcm_dcs_write_seq_static(ctx,0xA9,0xB2);
lcm_dcs_write_seq_static(ctx,0xAA,0x2D);
lcm_dcs_write_seq_static(ctx,0xAB,0x9C);
lcm_dcs_write_seq_static(ctx,0xAC,0x2E);
lcm_dcs_write_seq_static(ctx,0xAD,0xA4);
lcm_dcs_write_seq_static(ctx,0xAE,0x2D);
lcm_dcs_write_seq_static(ctx,0xAF,0x9F);
lcm_dcs_write_seq_static(ctx,0xB0,0x2C);
lcm_dcs_write_seq_static(ctx,0xB1,0x99);
lcm_dcs_write_seq_static(ctx,0xB2,0x2E);
lcm_dcs_write_seq_static(ctx,0xB3,0xA1);
lcm_dcs_write_seq_static(ctx,0xB4,0x2E);
lcm_dcs_write_seq_static(ctx,0xB5,0xB1);
lcm_dcs_write_seq_static(ctx,0xB6,0x2E);
lcm_dcs_write_seq_static(ctx,0xB7,0xA2);
lcm_dcs_write_seq_static(ctx,0xB8,0x2D);
lcm_dcs_write_seq_static(ctx,0xB9,0x9A);
lcm_dcs_write_seq_static(ctx,0xFF,0x78,0x07,0x0E);
lcm_dcs_write_seq_static(ctx,0x00,0xA3);		//LH MODE
lcm_dcs_write_seq_static(ctx,0x02,0x0F);
lcm_dcs_write_seq_static(ctx,0x04,0x06);		//TSHD_1VP Off & TSVD FREE RUN
lcm_dcs_write_seq_static(ctx,0x13,0x04);      //LV_TSHD_pos		
lcm_dcs_write_seq_static(ctx,0xC0,0x12);		//TP3 UNIT ASW prescan off 0903
lcm_dcs_write_seq_static(ctx,0xB0,0x21);		//TP3 UNIT ASW prescan off 0903
lcm_dcs_write_seq_static(ctx,0x20,0x05);     	//TP term num	
lcm_dcs_write_seq_static(ctx,0x25,0x0B);     	//TP2_unit0=179.9us	
lcm_dcs_write_seq_static(ctx,0x26,0x3E);     	//TP2_unit0=179.9us	
lcm_dcs_write_seq_static(ctx,0x27,0x10);     	//unit_line_num	
lcm_dcs_write_seq_static(ctx,0x29,0x92);     	//unit_line_num	
lcm_dcs_write_seq_static(ctx,0x2D,0x5B);     	//RTN=2.875us	
lcm_dcs_write_seq_static(ctx,0x30,0x00);     	//RTN[9:8]	
lcm_dcs_write_seq_static(ctx,0x2B,0x1C);		//TP modulation	
lcm_dcs_write_seq_static(ctx,0x41,0x17);     	//LH_TSVD1_pos	
lcm_dcs_write_seq_static(ctx,0x43,0x17);     	//LH_TSVD2_pos	
lcm_dcs_write_seq_static(ctx,0x40,0x0B);     	//TP term num	
lcm_dcs_write_seq_static(ctx,0x45,0x0A);     	//TP2_unit0=174.8us	
lcm_dcs_write_seq_static(ctx,0x46,0xEB);     	//TP2_unit0=174.8us	
lcm_dcs_write_seq_static(ctx,0x47,0x00);     	//unit_line_num	
lcm_dcs_write_seq_static(ctx,0x49,0xC9);     	//unit_line_num	
lcm_dcs_write_seq_static(ctx,0xC8,0x5C);     	//TP3-2 period	
lcm_dcs_write_seq_static(ctx,0xC9,0x5C);     	//TP3-1 period	
lcm_dcs_write_seq_static(ctx,0x4D,0xB6);     	//RTN=5.719us	
lcm_dcs_write_seq_static(ctx,0x50,0x00);     	//RTN[9:8]	
lcm_dcs_write_seq_static(ctx,0x4B,0x1C);		//TP modulation
lcm_dcs_write_seq_static(ctx,0x05,0x20);		//TP modulation off 24h , on = 20
lcm_dcs_write_seq_static(ctx,0xE0,0x05);      //60_t8_de_tp
lcm_dcs_write_seq_static(ctx,0xE2,0x07);      //60_t9_de_tp
lcm_dcs_write_seq_static(ctx,0xE3,0x2F);      //60_t7_de_tp
lcm_dcs_write_seq_static(ctx,0xE5,0x07);      //60_SDT_tp
//*********** 90 HZ TABLE**************
lcm_dcs_write_seq_static(ctx,0xFF,0x78,0x07,0x1E);
lcm_dcs_write_seq_static(ctx,0xBD,0x01);     	//90_tpt_fr_sel	
lcm_dcs_write_seq_static(ctx,0xB1,0x1F);     	//LAT2_OFFSET	
lcm_dcs_write_seq_static(ctx,0x61,0x22);     	//LH_TSVD1_pos	
lcm_dcs_write_seq_static(ctx,0x63,0x22);     	//LH_TSVD2_pos	
lcm_dcs_write_seq_static(ctx,0x60,0x07);     	//TP term num	
lcm_dcs_write_seq_static(ctx,0x65,0x0A);     	//TP2_unit0=173.4us	
lcm_dcs_write_seq_static(ctx,0x66,0xD6);     	//TP2_unit0=173.4us	
lcm_dcs_write_seq_static(ctx,0x67,0x10);     	//unit_line_num	
lcm_dcs_write_seq_static(ctx,0x69,0x2E);     	//unit_line_num	
lcm_dcs_write_seq_static(ctx,0x16,0x3E);     	//TP1_period	
lcm_dcs_write_seq_static(ctx,0x1E,0x3E);     	//TP3-2 period	
lcm_dcs_write_seq_static(ctx,0x1F,0x3E);     	//TP3-1 period	
lcm_dcs_write_seq_static(ctx,0x6D,0x7A);     	//RTN=3.844us
lcm_dcs_write_seq_static(ctx,0x6B,0x1C);	
lcm_dcs_write_seq_static(ctx,0x70,0x00);     	//RTN[9:8]	
lcm_dcs_write_seq_static(ctx,0xB4,0x1D);     	//Qsync_T1	
lcm_dcs_write_seq_static(ctx,0xB5,0x31);     	//Qsync_T3	
lcm_dcs_write_seq_static(ctx,0xB6,0x31);     	//Qsync_T4	
lcm_dcs_write_seq_static(ctx,0xB7,0x27);     	//Qsync_T2=173.81us	
lcm_dcs_write_seq_static(ctx,0xBA,0x00);     	//Qsync_T5	
lcm_dcs_write_seq_static(ctx,0x20,0x05);      //90_t8_de_tp
lcm_dcs_write_seq_static(ctx,0x22,0x08);      //90_t9_de_tp
lcm_dcs_write_seq_static(ctx,0x23,0x1A);      //90_t7_de_tp
lcm_dcs_write_seq_static(ctx,0x24,0x08);      //90_SDT_tp
lcm_dcs_write_seq_static(ctx,0xC9,0x02);     	//60_tpt_fr_sel	
lcm_dcs_write_seq_static(ctx,0xC0,0x1F);     	//LAT2_OFFSET	
lcm_dcs_write_seq_static(ctx,0xC1,0x1D);     	//Qsync_T1	
lcm_dcs_write_seq_static(ctx,0xC2,0x2A);     	//Qsync_T3	
lcm_dcs_write_seq_static(ctx,0xC3,0x2A);     	//Qsync_T4	
lcm_dcs_write_seq_static(ctx,0xC4,0x1B);     	//Qsync_T2=180us	
lcm_dcs_write_seq_static(ctx,0xC7,0x00);     	//Qsync_T5	
lcm_dcs_write_seq_static(ctx,0xAD,0x00);     	//120_tpt_fr_sel	
lcm_dcs_write_seq_static(ctx,0xA1,0x1F);     	//LAT2_OFFSET	
lcm_dcs_write_seq_static(ctx,0x00,0x2F);     	//TP1_period	
lcm_dcs_write_seq_static(ctx,0x08,0x2E);     	//TP3-2 period	
lcm_dcs_write_seq_static(ctx,0x09,0x2E);     	//TP3-1 period	
lcm_dcs_write_seq_static(ctx,0xA4,0x1D);     	//Qsync_T1	
lcm_dcs_write_seq_static(ctx,0xA5,0x51);     	//Qsync_T3	
lcm_dcs_write_seq_static(ctx,0xA6,0x41);     	//Qsync_T4	
lcm_dcs_write_seq_static(ctx,0xA7,0x36);     	//Qsync_T2=180.59us	
lcm_dcs_write_seq_static(ctx,0xAA,0x00);     	//Qsync_T5	
lcm_dcs_write_seq_static(ctx,0x0A,0x05);      //120_t8_de_tp
lcm_dcs_write_seq_static(ctx,0x0C,0x08);      //120_t9_de_tp
lcm_dcs_write_seq_static(ctx,0x0D,0x14);      //120_t7_de_tp
lcm_dcs_write_seq_static(ctx,0x0E,0x08);      //120_SDT_tp
lcm_dcs_write_seq_static(ctx,0xFF,0x78,0x07,0x00);//PAGE0
lcm_dcs_write_seq_static(ctx, 0x11);
mdelay(120);
lcm_dcs_write_seq_static(ctx, 0x29);
mdelay(20);
lcm_dcs_write_seq_static(ctx,0x35,0x00);
}

static int ili_lcm_disable(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);

	if (!ctx->enabled)
		return 0;

	if (ctx->backlight) {
		ctx->backlight->props.power = FB_BLANK_POWERDOWN;
		backlight_update_status(ctx->backlight);
	}

	ctx->enabled = false;

	return 0;
}

static int ili_lcm_unprepare(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);

	if (!ctx->prepared)
		return 0;
	pr_info("%s\n", __func__);

	ctx->error = 0;
	ctx->prepared = false;

	ctx->bias_neg = devm_gpiod_get_index(ctx->dev,
		"bias", 1, GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->bias_neg)) {
		dev_err(ctx->dev, "%s: cannot get bias_neg %ld\n",
			__func__, PTR_ERR(ctx->bias_neg));
		return PTR_ERR(ctx->bias_neg);
	}
	gpiod_set_value(ctx->bias_neg, 0);
	devm_gpiod_put(ctx->dev, ctx->bias_neg);

	udelay(1000);

	ctx->bias_pos = devm_gpiod_get_index(ctx->dev,
		"bias", 0, GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->bias_pos)) {
		dev_err(ctx->dev, "%s: cannot get bias_pos %ld\n",
			__func__, PTR_ERR(ctx->bias_pos));
		return PTR_ERR(ctx->bias_pos);
	}
	gpiod_set_value(ctx->bias_pos, 0);
	devm_gpiod_put(ctx->dev, ctx->bias_pos);
	udelay(1000);
//prize-add gpio ldo1.8v-pengzhipeng-20220514-start
	ctx->ldo18_gpio = devm_gpiod_get(ctx->dev, "ldo18", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->ldo18_gpio)) {
		dev_info(ctx->dev, "cannot get ldo18-gpios %ld\n",
			PTR_ERR(ctx->ldo18_gpio));
		return PTR_ERR(ctx->ldo18_gpio);
	}
	gpiod_set_value(ctx->ldo18_gpio, 0);
	devm_gpiod_put(ctx->dev, ctx->ldo18_gpio);
	udelay(1000);
//prize-add gpio ldo1.8v-pengzhipeng-20220514-end
	pr_info("%s ok\n", __func__);

	return 0;
}

static int ili_lcm_prepare(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
	int ret;

	pr_info("%s\n", __func__);
	if (ctx->prepared)
		return 0;

//prize-add gpio ldo1.8v-pengzhipeng-20220514-start
	ctx->ldo18_gpio = devm_gpiod_get(ctx->dev, "ldo18", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->ldo18_gpio)) {
		dev_info(ctx->dev, "cannot get ldo18-gpios %ld\n",
			PTR_ERR(ctx->ldo18_gpio));
		return PTR_ERR(ctx->ldo18_gpio);
	}
	gpiod_set_value(ctx->ldo18_gpio, 1);
	devm_gpiod_put(ctx->dev, ctx->ldo18_gpio);
	udelay(1000);
//prize-add gpio ldo1.8v-pengzhipeng-20220514-end
	ctx->bias_pos = devm_gpiod_get_index(ctx->dev,
		"bias", 0, GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->bias_pos)) {
		dev_err(ctx->dev, "%s: cannot get bias_pos %ld\n",
			__func__, PTR_ERR(ctx->bias_pos));
		return PTR_ERR(ctx->bias_pos);
	}
	gpiod_set_value(ctx->bias_pos, 1);
	devm_gpiod_put(ctx->dev, ctx->bias_pos);

	udelay(2000);

	ctx->bias_neg = devm_gpiod_get_index(ctx->dev,
		"bias", 1, GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->bias_neg)) {
		dev_err(ctx->dev, "%s: cannot get bias_neg %ld\n",
			__func__, PTR_ERR(ctx->bias_neg));
		return PTR_ERR(ctx->bias_neg);
	}
	gpiod_set_value(ctx->bias_neg, 1);
	devm_gpiod_put(ctx->dev, ctx->bias_neg);
	udelay(2000);
	lcm_panel_init(ctx);

	ret = ctx->error;
	if (ret < 0)
		ili_lcm_unprepare(panel);

	ctx->prepared = true;

#if defined(CONFIG_MTK_PANEL_EXT)
	mtk_panel_tch_rst(panel);
#endif
#ifdef PANEL_SUPPORT_READBACK
	lcm_panel_get_data(ctx);
#endif
	pr_info("%s ret=%d\n", __func__, ret);
	return ret;
}

static int ili_lcm_enable(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);

	if (ctx->enabled)
		return 0;

	if (ctx->backlight) {
		ctx->backlight->props.power = FB_BLANK_UNBLANK;
		backlight_update_status(ctx->backlight);
	}

	ctx->enabled = true;

	return 0;
}

#define VAC (2412)
#define HAC (1080)
#define HFP (22)
#define HSA (4)
#define HBP (22)
#define VFP (36)
#define VSA (4)
#define VBP (35)

static u32 fake_heigh = 2412;
static u32 fake_width = 1080;
static bool need_fake_resolution;

static struct drm_display_mode performance_mode_120 = {
	.clock = 850000,
	.hdisplay = 1080,
	.hsync_start = 1080 + HFP,
	.hsync_end = 1080 + HFP + HSA,
	.htotal = 1080 + HFP + HSA + HBP,//1136 //1133
	.vdisplay = 2412,
	.vsync_start = 2412 + VFP,
	.vsync_end = 2412 + VFP + VSA,
	.vtotal = 2412 + VFP + VSA + VBP,//3732
	.vrefresh = 120,
};

static struct drm_display_mode performance_mode_90 = {
	.clock = 850000,
	.hdisplay = 1080,
	.hsync_start = 1080 + HFP,
	.hsync_end = 1080 + HFP + HSA,
	.htotal = 1080 + HFP + HSA + HBP,//1136 //1133
	.vdisplay = 2412,
	.vsync_start = 2412 + 850,
	.vsync_end = 2412 + 850 + VSA,
	.vtotal = 2412 + 850 + VSA + VBP,//3732
	.vrefresh = 90,
};

static struct drm_display_mode default_mode = {
	.clock = 850000,
	.hdisplay = 1080,
	.hsync_start = 1080 + HFP,
	.hsync_end = 1080 + HFP + HSA,
	.htotal = 1080 + HFP + HSA + HBP,//1136 //1133
	.vdisplay = 2412,
	.vsync_start = 2412 + 2500,
	.vsync_end = 2412 + 2500 + VSA,
	.vtotal = 2412 + 2500 + VSA + VBP,//3732
	.vrefresh = 60,
};

#if defined(CONFIG_MTK_PANEL_EXT)
static struct mtk_panel_params ili_ext_params_120 = {
	.vfp_low_power = 2500,//60hz
	.pll_clk = 425,
	.cust_esd_check = 1,
	.esd_check_enable = 1,
	.lcm_esd_check_table[0] = {
		.cmd = 0x0A, .count = 1, .para_list[0] = 0x9C,
	},
	.data_rate = 850,
	.output_mode = MTK_PANEL_DSC_SINGLE_PORT,
	.dsc_params = {
		.enable = 1,
		.ver = 17,
		.slice_mode = 1,
		.rgb_swap = 0,
		.dsc_cfg = 34,
		.rct_on = 1,
		.bit_per_channel = 8,
		.dsc_line_buf_depth = 9,
		.bp_enable = 1,
		.bit_per_pixel = 128,
		.pic_height = 2412,
		.pic_width = 1080,
		.slice_height = 12,
		.slice_width = 540,
		.chunk_size = 540,
		.xmit_delay = 170,
		.dec_delay = 526,
		.scale_value = 32,
		.increment_interval = 67,
		.decrement_interval = 7,
		.line_bpg_offset = 12,
		.nfl_bpg_offset = 2235,
		.slice_bpg_offset = 2170,
		.initial_offset = 6144,
		.final_offset = 7072,
		.flatness_minqp = 3,
		.flatness_maxqp = 12,
		.rc_model_size = 8192,
		.rc_edge_factor = 6,
		.rc_quant_incr_limit0 = 11,
		.rc_quant_incr_limit1 = 11,
		.rc_tgt_offset_hi = 3,
		.rc_tgt_offset_lo = 3,
		},
};
static struct mtk_panel_params ili_ext_params_90 = {
	.vfp_low_power = 2500,//60hz
	.pll_clk = 425,
	.cust_esd_check = 1,
	.esd_check_enable = 1,
	.lcm_esd_check_table[0] = {
		.cmd = 0x0A, .count = 1, .para_list[0] = 0x9C,
	},
	.data_rate = 850,
	.output_mode = MTK_PANEL_DSC_SINGLE_PORT,
	.dsc_params = {
		.enable = 1,
		.ver = 17,
		.slice_mode = 1,
		.rgb_swap = 0,
		.dsc_cfg = 34,
		.rct_on = 1,
		.bit_per_channel = 8,
		.dsc_line_buf_depth = 9,
		.bp_enable = 1,
		.bit_per_pixel = 128,
		.pic_height = 2412,
		.pic_width = 1080,
		.slice_height = 12,
		.slice_width = 540,
		.chunk_size = 540,
		.xmit_delay = 170,
		.dec_delay = 526,
		.scale_value = 32,
		.increment_interval = 67,
		.decrement_interval = 7,
		.line_bpg_offset = 12,
		.nfl_bpg_offset = 2235,
		.slice_bpg_offset = 2170,
		.initial_offset = 6144,
		.final_offset = 7072,
		.flatness_minqp = 3,
		.flatness_maxqp = 12,
		.rc_model_size = 8192,
		.rc_edge_factor = 6,
		.rc_quant_incr_limit0 = 11,
		.rc_quant_incr_limit1 = 11,
		.rc_tgt_offset_hi = 3,
		.rc_tgt_offset_lo = 3,
		},

};
static struct mtk_panel_params ili_ext_params_60 = {
	//.vfp_low_power = 2540,//60hz
	.pll_clk = 425,
	.cust_esd_check = 1,
	.esd_check_enable = 1,
	.lcm_esd_check_table[0] = {
		.cmd = 0x0A, .count = 1, .para_list[0] = 0x9C,
	},
	.data_rate = 850,
	.output_mode = MTK_PANEL_DSC_SINGLE_PORT,
	.dsc_params = {
		.enable = 1,
		.ver = 17,
		.slice_mode = 1,
		.rgb_swap = 0,
		.dsc_cfg = 34,
		.rct_on = 1,
		.bit_per_channel = 8,
		.dsc_line_buf_depth = 9,
		.bp_enable = 1,
		.bit_per_pixel = 128,
		.pic_height = 2412,
		.pic_width = 1080,
		.slice_height = 12,
		.slice_width = 540,
		.chunk_size = 540,
		.xmit_delay = 170,
		.dec_delay = 526,
		.scale_value = 32,
		.increment_interval = 67,
		.decrement_interval = 7,
		.line_bpg_offset = 12,
		.nfl_bpg_offset = 2235,
		.slice_bpg_offset = 2170,
		.initial_offset = 6144,
		.final_offset = 7072,
		.flatness_minqp = 3,
		.flatness_maxqp = 12,
		.rc_model_size = 8192,
		.rc_edge_factor = 6,
		.rc_quant_incr_limit0 = 11,
		.rc_quant_incr_limit1 = 11,
		.rc_tgt_offset_hi = 3,
		.rc_tgt_offset_lo = 3,
		},

};

struct drm_display_mode *ili_get_mode_by_id(struct drm_panel *panel,
	unsigned int mode)
{
	struct drm_display_mode *m;
	unsigned int i = 0;

	list_for_each_entry(m, &panel->connector->modes, head) {
		if (i == mode)
			return m;
		i++;
	}
	return NULL;
}
static int current_fps = 60;

static int ili_mtk_panel_ext_param_set(struct drm_panel *panel,
			 unsigned int mode)
{
	struct mtk_panel_ext *ext = find_panel_ext(panel);
	int ret = 0;
	struct drm_display_mode *m = ili_get_mode_by_id(panel, mode);


	if (m->vrefresh == 60)
		ext->params = &ili_ext_params_60;
	else if (m->vrefresh == 90)
		ext->params = &ili_ext_params_90;
	else if (m->vrefresh == 120)
		ext->params = &ili_ext_params_120;
	else
		ret = 1;
	if (!ret)
		current_fps = m->vrefresh;
	return ret;
}

static int ili_panel_ext_reset(struct drm_panel *panel, int on)
{
	struct lcm *ctx = panel_to_lcm(panel);

	ctx->reset_gpio =
		devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio)) {
		dev_info(ctx->dev, "%s: cannot get reset-gpios %ld\n",
			__func__, PTR_ERR(ctx->reset_gpio));
		return PTR_ERR(ctx->reset_gpio);
	}
	gpiod_set_value(ctx->reset_gpio, on);
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);

	return 0;
}

static int ili_panel_ata_check(struct drm_panel *panel)
{
	pr_info("%s success\n", __func__);
	return 1;
}

/*static int ili_lcm_setbacklight_cmdq(void *dsi, dcs_write_gce cb,
	void *handle, unsigned int level)
{
	char bl_tb0[] = {0x51, 0xFF};

	bl_tb0[1] = level;

	if (!cb)
		return -1;

	cb(dsi, handle, bl_tb0, ARRAY_SIZE(bl_tb0));

	return 0;
}*/

//static int lcm_get_virtual_heigh(void)
//{
//	return VAC;
//}

//static int lcm_get_virtual_width(void)
//{
//	return HAC;
//}

static struct mtk_panel_funcs ili_ext_funcs = {
	.reset = ili_panel_ext_reset,
	//.set_backlight_cmdq = ili_lcm_setbacklight_cmdq,
	.ext_param_set = ili_mtk_panel_ext_param_set,
	.ata_check = ili_panel_ata_check,
	//.get_virtual_heigh = lcm_get_virtual_heigh,
	//.get_virtual_width = lcm_get_virtual_width,
};
#endif

struct panel_desc {
	const struct drm_display_mode *modes;
	unsigned int num_modes;

	unsigned int bpc;

	struct {
		unsigned int width;
		unsigned int height;
	} size;

	struct {
		unsigned int prepare;
		unsigned int enable;
		unsigned int disable;
		unsigned int unprepare;
	} delay;
};

static void ili_change_drm_disp_mode_params(struct drm_display_mode *mode)
{
	if (fake_heigh > 0 && fake_heigh < VAC) {
		mode->vsync_start = mode->vsync_start - mode->vdisplay
					+ fake_heigh;
		mode->vsync_end = mode->vsync_end - mode->vdisplay + fake_heigh;
		mode->vtotal = mode->vtotal - mode->vdisplay + fake_heigh;
		mode->vdisplay = fake_heigh;
	}
	if (fake_width > 0 && fake_width < HAC) {
		mode->hsync_start = mode->hsync_start - mode->hdisplay
					+ fake_width;
		mode->hsync_end = mode->hsync_end - mode->hdisplay + fake_width;
		mode->htotal = mode->htotal - mode->hdisplay + fake_width;
		mode->hdisplay = fake_width;
	}
}

static int ili_lcm_get_modes(struct drm_panel *panel)
{
	struct drm_display_mode *mode_60;
	struct drm_display_mode *mode_90;
	struct drm_display_mode *mode_120;
	if (need_fake_resolution)
		ili_change_drm_disp_mode_params(&default_mode);

	mode_60 = drm_mode_duplicate(panel->drm, &default_mode);
	if (!mode_60) {
		dev_info(panel->drm->dev, "failed to add mode_60 %ux%ux@%u\n",
			default_mode.hdisplay, default_mode.vdisplay,
			default_mode.vrefresh);
		return -ENOMEM;
	}

	drm_mode_set_name(mode_60);
	mode_60->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	drm_mode_probed_add(panel->connector, mode_60);


	mode_90 = drm_mode_duplicate(panel->drm, &performance_mode_90);
	if (!mode_90) {
		dev_info(panel->drm->dev, "failed to add mode_90 %ux%ux@%u\n",
			performance_mode_90.hdisplay, performance_mode_90.vdisplay,
			performance_mode_90.vrefresh);
		return -ENOMEM;
	}
	drm_mode_set_name(mode_90);
	mode_90->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	drm_mode_probed_add(panel->connector, mode_90);
	
	
	mode_120 = drm_mode_duplicate(panel->drm, &performance_mode_120);
	if (!mode_120) {
		dev_info(panel->drm->dev, "failed to add mode_120 %ux%ux@%u\n",
			performance_mode_120.hdisplay, performance_mode_120.vdisplay,
			performance_mode_120.vrefresh);
		return -ENOMEM;
	}
	drm_mode_set_name(mode_120);
	mode_120->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	drm_mode_probed_add(panel->connector, mode_120);
//FIXME:prize-Resolving screen size display errors-pengzhipeng-20220726-start
	panel->connector->display_info.width_mm = 68;
	panel->connector->display_info.height_mm = 157;
//FIXME:prize-Resolving screen size display errors-pengzhipeng-20220726-end
//FIXME:prize-Solve the problem that after turning on the 120Hz refresh rate and then turning off and on again it is lk set to 60hz-pengzhipeng-20220818-start
	return 3;
//FIXME:prize-Solve the problem that after turning on the 120Hz refresh rate and then turning off and on again it is lk set to 60hz-pengzhipeng-20220818-end
}

static const struct drm_panel_funcs lcm_drm_funcs = {
	.disable = ili_lcm_disable,
	.unprepare = ili_lcm_unprepare,
	.prepare = ili_lcm_prepare,
	.enable = ili_lcm_enable,
	.get_modes = ili_lcm_get_modes,
};

static void ili_check_is_need_fake_resolution(struct device *dev)
{
	unsigned int ret = 0;

	ret = of_property_read_u32(dev->of_node, "fake_heigh", &fake_heigh);
	if (ret)
		need_fake_resolution = false;
	ret = of_property_read_u32(dev->of_node, "fake_width", &fake_width);
	if (ret)
		need_fake_resolution = false;
	if (fake_heigh > 0 && fake_heigh < VAC)
		need_fake_resolution = false;
	if (fake_width > 0 && fake_width < HAC)
		need_fake_resolution = false;
}

static int ili_lcm_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct lcm *ctx;
	struct device_node *backlight;
	int ret;
	struct device_node *dsi_node, *remote_node = NULL, *endpoint = NULL;

	dsi_node = of_get_parent(dev->of_node);
	if (dsi_node) {
		endpoint = of_graph_get_next_endpoint(dsi_node, NULL);
		if (endpoint) {
			remote_node = of_graph_get_remote_port_parent(endpoint);
			if (!remote_node) {
				pr_info("No panel connected,skip probe lcm\n");
				return -ENODEV;
			}
			pr_info("device node name:%s\n", remote_node->name);
		}
	}
	if (remote_node != dev->of_node) {
		pr_info("%s+ skip probe due to not current lcm\n", __func__);
		return -ENODEV;
	}

	ctx = devm_kzalloc(dev, sizeof(struct lcm), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	mipi_dsi_set_drvdata(dsi, ctx);

	ctx->dev = dev;
	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO
					| MIPI_DSI_MODE_VIDEO_SYNC_PULSE
					| MIPI_DSI_MODE_LPM
					| MIPI_DSI_MODE_EOT_PACKET
					| MIPI_DSI_CLOCK_NON_CONTINUOUS;

	backlight = of_parse_phandle(dev->of_node, "backlight", 0);
	if (backlight) {
		ctx->backlight = of_find_backlight_by_node(backlight);
		of_node_put(backlight);

		if (!ctx->backlight)
			return -EPROBE_DEFER;
	}

	ctx->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio)) {
		dev_info(dev, "cannot get reset-gpios %ld\n",
			PTR_ERR(ctx->reset_gpio));
		return PTR_ERR(ctx->reset_gpio);
	}
	devm_gpiod_put(dev, ctx->reset_gpio);
//prize-add gpio ldo1.8v-pengzhipeng-20220514-start
	ctx->ldo18_gpio = devm_gpiod_get(dev, "ldo18", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->ldo18_gpio)) {
		dev_info(dev, "cannot get ldo18-gpios %ld\n",
			PTR_ERR(ctx->ldo18_gpio));
		return PTR_ERR(ctx->ldo18_gpio);
	}
	devm_gpiod_put(dev, ctx->ldo18_gpio);
//prize-add gpio ldo1.8v-pengzhipeng-20220514-end
	
	ctx->bias_pos = devm_gpiod_get_index(dev, "bias", 0, GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->bias_pos)) {
		dev_err(dev, "%s: cannot get bias-pos 0 %ld\n",
			__func__, PTR_ERR(ctx->bias_pos));
		return PTR_ERR(ctx->bias_pos);
	}
	devm_gpiod_put(dev, ctx->bias_pos);

	ctx->bias_neg = devm_gpiod_get_index(dev, "bias", 1, GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->bias_neg)) {
		dev_err(dev, "%s: cannot get bias-neg 1 %ld\n",
			__func__, PTR_ERR(ctx->bias_neg));
		return PTR_ERR(ctx->bias_neg);
	}
	devm_gpiod_put(dev, ctx->bias_neg);
	ctx->prepared = true;
	ctx->enabled = true;

	drm_panel_init(&ctx->panel);
	ctx->panel.dev = dev;
	ctx->panel.funcs = &lcm_drm_funcs;

	ret = drm_panel_add(&ctx->panel);
	if (ret < 0)
		return ret;

	ret = mipi_dsi_attach(dsi);
	if (ret < 0)
		drm_panel_remove(&ctx->panel);

#if defined(CONFIG_MTK_PANEL_EXT)
	mtk_panel_tch_handle_reg(&ctx->panel);
	ret = mtk_panel_ext_create(dev, &ili_ext_params_60, &ili_ext_funcs, &ctx->panel);
	if (ret < 0)
		return ret;
#endif
	ili_check_is_need_fake_resolution(dev);
	printk("%s-\n", __func__);
	//prize add by anhengxuan for lcd hardware info 20220102 start
#if defined(CONFIG_PRIZE_HARDWARE_INFO)
    strcpy(current_lcm_info.chip,"ili7807s");
    strcpy(current_lcm_info.vendor,"ilitek");
    sprintf(current_lcm_info.id,"0x%02x",0x02);
    strcpy(current_lcm_info.more,"1080*2412");
#endif
//prize add by anhengxuan for lcd hardware info 20220402 end
	return ret;
}

static int ili_lcm_remove(struct mipi_dsi_device *dsi)
{
	struct lcm *ctx = mipi_dsi_get_drvdata(dsi);

	mipi_dsi_detach(dsi);
	drm_panel_remove(&ctx->panel);

	return 0;
}

static const struct of_device_id ili_lcm_of_match[] = {
	{ .compatible = "truly,ili7807s,vdo", },
	{ }
};

MODULE_DEVICE_TABLE(of, ili_lcm_of_match);

static struct mipi_dsi_driver ili_lcm_driver = {
	.probe = ili_lcm_probe,
	.remove = ili_lcm_remove,
	.driver = {
		.name = "panel-truly-ili7807s-vdo",
		.owner = THIS_MODULE,
		.of_match_table = ili_lcm_of_match,
	},
};

module_mipi_dsi_driver(ili_lcm_driver);

MODULE_AUTHOR("MEDIATEK");
MODULE_DESCRIPTION("sc ili7807s VDO LCD Panel Driver");
MODULE_LICENSE("GPL v2");
