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
	lcm_dcs_write_seq_static(ctx,0xFF,0x55,0xAA,0x66); 
	lcm_dcs_write_seq_static(ctx,0xFF,0x10); 
	lcm_dcs_write_seq_static(ctx,0xFB,0x00); 
	lcm_dcs_write_seq_static(ctx,0xFF,0x20); 
	lcm_dcs_write_seq_static(ctx,0xFB,0x00); 
	lcm_dcs_write_seq_static(ctx,0xFF,0x21); 
	lcm_dcs_write_seq_static(ctx,0xFB,0x00); 
	lcm_dcs_write_seq_static(ctx,0xFF,0x22); 
	lcm_dcs_write_seq_static(ctx,0xFB,0x00); 
	lcm_dcs_write_seq_static(ctx,0xFF,0x23); 
	lcm_dcs_write_seq_static(ctx,0xFB,0x00); 
	lcm_dcs_write_seq_static(ctx,0xFF,0x24); 
	lcm_dcs_write_seq_static(ctx,0xFB,0x00); 
	lcm_dcs_write_seq_static(ctx,0xFF,0x25); 
	lcm_dcs_write_seq_static(ctx,0xFB,0x00); 
	lcm_dcs_write_seq_static(ctx,0xFF,0x28);
	lcm_dcs_write_seq_static(ctx,0xFB,0x00);
	lcm_dcs_write_seq_static(ctx,0xFF,0x2A);
	lcm_dcs_write_seq_static(ctx,0xFB,0x00);
	lcm_dcs_write_seq_static(ctx,0xFF,0xA3);
	lcm_dcs_write_seq_static(ctx,0xFB,0x00);
	lcm_dcs_write_seq_static(ctx,0xFF,0xB3);
	lcm_dcs_write_seq_static(ctx,0xFB,0x00);
	lcm_dcs_write_seq_static(ctx,0xFF,0x2A); 
	lcm_dcs_write_seq_static(ctx,0x01,0x31); 
	lcm_dcs_write_seq_static(ctx,0x02,0x31);  
	lcm_dcs_write_seq_static(ctx,0x03,0x31); 
	lcm_dcs_write_seq_static(ctx,0x04,0x31); 
	lcm_dcs_write_seq_static(ctx,0x05,0x29); 
	lcm_dcs_write_seq_static(ctx,0x06,0x2F); 
	lcm_dcs_write_seq_static(ctx,0x07,0x28); 
	lcm_dcs_write_seq_static(ctx,0x08,0x28); 
	lcm_dcs_write_seq_static(ctx,0x09,0x26); 
	lcm_dcs_write_seq_static(ctx,0x0A,0x26); 
	lcm_dcs_write_seq_static(ctx,0x0B,0x1A); 
	lcm_dcs_write_seq_static(ctx,0x0C,0x19); 
	lcm_dcs_write_seq_static(ctx,0x0D,0x18); 
	lcm_dcs_write_seq_static(ctx,0x0E,0x09); 
	lcm_dcs_write_seq_static(ctx,0x0F,0x08); 
	lcm_dcs_write_seq_static(ctx,0x10,0x0D); 
	lcm_dcs_write_seq_static(ctx,0x11,0x0C); 
	lcm_dcs_write_seq_static(ctx,0x12,0x02); 
	lcm_dcs_write_seq_static(ctx,0x13,0x27); 
	lcm_dcs_write_seq_static(ctx,0x14,0x27); 
	lcm_dcs_write_seq_static(ctx,0x15,0x24); 
	lcm_dcs_write_seq_static(ctx,0x16,0x25); 
	lcm_dcs_write_seq_static(ctx,0x17,0x01); 
	lcm_dcs_write_seq_static(ctx,0x18,0x00); 
	lcm_dcs_write_seq_static(ctx,0x19,0x31); 
	lcm_dcs_write_seq_static(ctx,0x1A,0x31); 
	lcm_dcs_write_seq_static(ctx,0x1B,0x31); 
	lcm_dcs_write_seq_static(ctx,0x1C,0x31); 
	lcm_dcs_write_seq_static(ctx,0x1D,0x29); 
	lcm_dcs_write_seq_static(ctx,0x1E,0x2F); 
	lcm_dcs_write_seq_static(ctx,0x1F,0x28); 
	lcm_dcs_write_seq_static(ctx,0x20,0x28); 
	lcm_dcs_write_seq_static(ctx,0x21,0x26); 
	lcm_dcs_write_seq_static(ctx,0x22,0x26); 
	lcm_dcs_write_seq_static(ctx,0x23,0x1A); 
	lcm_dcs_write_seq_static(ctx,0x24,0x19); 
	lcm_dcs_write_seq_static(ctx,0x25,0x18); 
	lcm_dcs_write_seq_static(ctx,0x26,0x09); 
	lcm_dcs_write_seq_static(ctx,0x27,0x08); 
	lcm_dcs_write_seq_static(ctx,0x28,0x0D); 
	lcm_dcs_write_seq_static(ctx,0x29,0x0C); 
	lcm_dcs_write_seq_static(ctx,0x2A,0x02); 
	lcm_dcs_write_seq_static(ctx,0x2B,0x27); 
	lcm_dcs_write_seq_static(ctx,0x2D,0x27); 
	lcm_dcs_write_seq_static(ctx,0x2E,0x24); 
	lcm_dcs_write_seq_static(ctx,0x2F,0x25); 
	lcm_dcs_write_seq_static(ctx,0x30,0x01); 
	lcm_dcs_write_seq_static(ctx,0x31,0x00); 
	lcm_dcs_write_seq_static(ctx,0x3D,0x20);  
	lcm_dcs_write_seq_static(ctx,0xFF,0x21); 
	lcm_dcs_write_seq_static(ctx,0x3A,0x00); 
	lcm_dcs_write_seq_static(ctx,0x3B,0x00); 
	lcm_dcs_write_seq_static(ctx,0x88,0x00);   
	lcm_dcs_write_seq_static(ctx,0x89,0x00); 
	lcm_dcs_write_seq_static(ctx,0x8A,0x00); 
	lcm_dcs_write_seq_static(ctx,0x94,0x00);  
	lcm_dcs_write_seq_static(ctx,0x95,0x00);  
	lcm_dcs_write_seq_static(ctx,0x96,0x00); 
	lcm_dcs_write_seq_static(ctx,0x45,0x07);  
	lcm_dcs_write_seq_static(ctx,0x46,0x61);  
	lcm_dcs_write_seq_static(ctx,0x47,0x02);      
	lcm_dcs_write_seq_static(ctx,0x48,0x02);    
	lcm_dcs_write_seq_static(ctx,0x4C,0x61);  
	lcm_dcs_write_seq_static(ctx,0x4D,0x01);      
	lcm_dcs_write_seq_static(ctx,0x4E,0x01);      
	lcm_dcs_write_seq_static(ctx,0x52,0x64); 
	lcm_dcs_write_seq_static(ctx,0x53,0x0f);  
	lcm_dcs_write_seq_static(ctx,0x54,0x0f);  
	lcm_dcs_write_seq_static(ctx,0x76,0x44); 
	lcm_dcs_write_seq_static(ctx,0x77,0x44);  
	lcm_dcs_write_seq_static(ctx,0x78,0x46); 
	lcm_dcs_write_seq_static(ctx,0x7E,0x03);  
	lcm_dcs_write_seq_static(ctx,0x7F,0x00);  
	lcm_dcs_write_seq_static(ctx,0x80,0x0a);      
	lcm_dcs_write_seq_static(ctx,0x81,0x0a);     
	lcm_dcs_write_seq_static(ctx,0x82,0x60);      
	lcm_dcs_write_seq_static(ctx,0x83,0x03);      
	lcm_dcs_write_seq_static(ctx,0x87,0x0a);     
	lcm_dcs_write_seq_static(ctx,0x8B,0x00);  
	lcm_dcs_write_seq_static(ctx,0x8C,0x08);      
	lcm_dcs_write_seq_static(ctx,0x8D,0x08);      
	lcm_dcs_write_seq_static(ctx,0x8E,0x60);     
	lcm_dcs_write_seq_static(ctx,0x8F,0x03);      
	lcm_dcs_write_seq_static(ctx,0x93,0x0a);    
	lcm_dcs_write_seq_static(ctx,0xAF,0x43); 
	lcm_dcs_write_seq_static(ctx,0xB0,0x43);
	lcm_dcs_write_seq_static(ctx,0x3F,0x00);
	lcm_dcs_write_seq_static(ctx,0xBE,0x03);
	lcm_dcs_write_seq_static(ctx,0xBF,0x66); 
	lcm_dcs_write_seq_static(ctx,0xC0,0x46);
	lcm_dcs_write_seq_static(ctx,0xC1,0x42);
	lcm_dcs_write_seq_static(ctx,0xC2,0x84);
	lcm_dcs_write_seq_static(ctx,0xC3,0x86);
	lcm_dcs_write_seq_static(ctx,0xC4,0x86);
	lcm_dcs_write_seq_static(ctx,0xC6,0x46);
	lcm_dcs_write_seq_static(ctx,0xC7,0x43); 
	lcm_dcs_write_seq_static(ctx,0xC8,0x40); 
	                         
	lcm_dcs_write_seq_static(ctx,0xC5,0x86); 
	lcm_dcs_write_seq_static(ctx,0xC9,0x56); 
	                         
	lcm_dcs_write_seq_static(ctx,0x49,0x02); 
	lcm_dcs_write_seq_static(ctx,0x4A,0x57);
	lcm_dcs_write_seq_static(ctx,0x4F,0x02); 
	lcm_dcs_write_seq_static(ctx,0x50,0x57);
	                         
	lcm_dcs_write_seq_static(ctx,0x84,0x02); 
	lcm_dcs_write_seq_static(ctx,0x85,0x57); 
	lcm_dcs_write_seq_static(ctx,0x90,0x02); 
	lcm_dcs_write_seq_static(ctx,0x91,0x57); 
	         
	lcm_dcs_write_seq_static(ctx,0xFF,0x20);
	lcm_dcs_write_seq_static(ctx,0x24,0x89);
	lcm_dcs_write_seq_static(ctx,0xCC,0x5F);
	lcm_dcs_write_seq_static(ctx,0xBC,0x16);
	lcm_dcs_write_seq_static(ctx,0xD5,0x35);
	lcm_dcs_write_seq_static(ctx,0x3D,0x25);
	lcm_dcs_write_seq_static(ctx,0x6F,0xC1);
	lcm_dcs_write_seq_static(ctx,0x12,0xDB);
	lcm_dcs_write_seq_static(ctx,0x02,0x4b);
	lcm_dcs_write_seq_static(ctx,0x03,0x4b);
	lcm_dcs_write_seq_static(ctx,0x0A,0x20);
	lcm_dcs_write_seq_static(ctx,0x0B,0x20);
	                         
	lcm_dcs_write_seq_static(ctx,0x04,0x2e);
	lcm_dcs_write_seq_static(ctx,0x05,0x2e);
	lcm_dcs_write_seq_static(ctx,0x0C,0x2A);
	lcm_dcs_write_seq_static(ctx,0x0D,0x2A);
	                         
	lcm_dcs_write_seq_static(ctx,0x1C,0x1e);
	lcm_dcs_write_seq_static(ctx,0x1D,0x1e);
	lcm_dcs_write_seq_static(ctx,0x1E,0x0D);
	lcm_dcs_write_seq_static(ctx,0x1F,0x0D);
	lcm_dcs_write_seq_static(ctx,0x20,0x0D); 
	lcm_dcs_write_seq_static(ctx,0x21,0x0D); 
	lcm_dcs_write_seq_static(ctx,0x77,0xD0); 
	lcm_dcs_write_seq_static(ctx,0x78,0xD5); 
	lcm_dcs_write_seq_static(ctx,0x7E,0x31);  //31
	lcm_dcs_write_seq_static(ctx,0x80,0x44);  
	lcm_dcs_write_seq_static(ctx,0x82,0x00);  
	lcm_dcs_write_seq_static(ctx,0x83,0x64);
	
	lcm_dcs_write_seq_static(ctx,0x85,0x2C);//2c
	lcm_dcs_write_seq_static(ctx,0x86,0x20);//20

	
	lcm_dcs_write_seq_static(ctx,0x8A,0x0F);
	lcm_dcs_write_seq_static(ctx,0x68,0xAA); 
	lcm_dcs_write_seq_static(ctx,0x2E,0x8F); //EN
	lcm_dcs_write_seq_static(ctx,0x2F,0x24); 
	lcm_dcs_write_seq_static(ctx,0x30,0x42); 
	lcm_dcs_write_seq_static(ctx,0x90,0x13); 
	lcm_dcs_write_seq_static(ctx,0x17,0x51);  
	lcm_dcs_write_seq_static(ctx,0x15,0x1D);  
	lcm_dcs_write_seq_static(ctx,0x3E,0x05); 
	lcm_dcs_write_seq_static(ctx,0xFF,0x25); 
	lcm_dcs_write_seq_static(ctx,0x73,0x01); 
	lcm_dcs_write_seq_static(ctx,0x78,0x1D);  
	lcm_dcs_write_seq_static(ctx,0x15,0x01); 
	lcm_dcs_write_seq_static(ctx,0x16,0x09); 
	lcm_dcs_write_seq_static(ctx,0x17,0x17); 
	lcm_dcs_write_seq_static(ctx,0x18,0x17);
	lcm_dcs_write_seq_static(ctx,0x19,0x17);
	lcm_dcs_write_seq_static(ctx,0x1D,0x05);
	lcm_dcs_write_seq_static(ctx,0x1E,0x05);
	lcm_dcs_write_seq_static(ctx,0x1F,0x00);  
	lcm_dcs_write_seq_static(ctx,0x22,0x02); 
	lcm_dcs_write_seq_static(ctx,0x23,0x21);  
	lcm_dcs_write_seq_static(ctx,0x24,0x03); 
	lcm_dcs_write_seq_static(ctx,0x25,0x00); 
	lcm_dcs_write_seq_static(ctx,0x26,0x23);  
	lcm_dcs_write_seq_static(ctx,0x27,0x01);  
	lcm_dcs_write_seq_static(ctx,0x28,0x00); 
	lcm_dcs_write_seq_static(ctx,0x29,0x00);  
	lcm_dcs_write_seq_static(ctx,0x2A,0x00);  
	lcm_dcs_write_seq_static(ctx,0x2B,0x00);  
	lcm_dcs_write_seq_static(ctx,0x2D,0x67);
	lcm_dcs_write_seq_static(ctx,0x2E,0x01); 
	lcm_dcs_write_seq_static(ctx,0x2F,0x01);
	lcm_dcs_write_seq_static(ctx,0x30,0x00);  
	lcm_dcs_write_seq_static(ctx,0x34,0x44);  
	lcm_dcs_write_seq_static(ctx,0x35,0x10);  
	lcm_dcs_write_seq_static(ctx,0x36,0x02); 
	lcm_dcs_write_seq_static(ctx,0x37,0x02);  
	lcm_dcs_write_seq_static(ctx,0xFF,0xA3);
	lcm_dcs_write_seq_static(ctx,0x58,0x8A);
	lcm_dcs_write_seq_static(ctx,0x39,0x07);
	        
	lcm_dcs_write_seq_static(ctx,0xFF,0x22);
	lcm_dcs_write_seq_static(ctx,0x01,0x09);
	lcm_dcs_write_seq_static(ctx,0x02,0x6C);
	lcm_dcs_write_seq_static(ctx,0x03,0x00);
	lcm_dcs_write_seq_static(ctx,0x24,0x18);
	lcm_dcs_write_seq_static(ctx,0x6F,0XC6);//C4
	lcm_dcs_write_seq_static(ctx,0x70,0x00);
	lcm_dcs_write_seq_static(ctx,0x71,0x10); 
	lcm_dcs_write_seq_static(ctx,0x72,0x93); 
	lcm_dcs_write_seq_static(ctx,0x73,0x00); 
	lcm_dcs_write_seq_static(ctx,0x74,0x02); 
	lcm_dcs_write_seq_static(ctx,0x75,0x02); 
	lcm_dcs_write_seq_static(ctx,0xEA,0x11); 
	lcm_dcs_write_seq_static(ctx,0x76,0x02); 
	lcm_dcs_write_seq_static(ctx,0x77,0x20);//0C 
	lcm_dcs_write_seq_static(ctx,0x05,0x00);
	lcm_dcs_write_seq_static(ctx,0x08,0x02);
	lcm_dcs_write_seq_static(ctx,0x0B,0x2A);
	lcm_dcs_write_seq_static(ctx,0x0C,0x00);

	//lcm_dcs_write_seq_static(ctx,0xDC,0xCC);//TPS IC_VSYNC
	//lcm_dcs_write_seq_static(ctx,0xE0,0x04);
	
	lcm_dcs_write_seq_static(ctx,0xE4,0x02);
	lcm_dcs_write_seq_static(ctx,0x1F,0x8E);  
	lcm_dcs_write_seq_static(ctx,0xEB,0x3D);  
	lcm_dcs_write_seq_static(ctx,0xFF,0x10); 
	lcm_dcs_write_seq_static(ctx,0xc0,0x80,0x0c,0x00,0xc9,0x02,0x00,0x02,0x0e,0x01,0x1f,0x00,0x07,0x08,0xbb,0x08,0x7A); 
	lcm_dcs_write_seq_static(ctx,0xC1,0x10,0xF0); 
	lcm_dcs_write_seq_static(ctx,0xFF,0x28);   
	lcm_dcs_write_seq_static(ctx,0x02,0x04);   
	lcm_dcs_write_seq_static(ctx,0x04,0x3C);  
	lcm_dcs_write_seq_static(ctx,0x05,0x20);  
	lcm_dcs_write_seq_static(ctx,0x06,0x0C);  
	lcm_dcs_write_seq_static(ctx,0x07,0x18);  
	lcm_dcs_write_seq_static(ctx,0x08,0x00);  
	lcm_dcs_write_seq_static(ctx,0x0B,0x22);  
	lcm_dcs_write_seq_static(ctx,0x0C,0x1C);  
	lcm_dcs_write_seq_static(ctx,0x0D,0x1C);  
	lcm_dcs_write_seq_static(ctx,0x0E,0xBB);  
	lcm_dcs_write_seq_static(ctx,0x0F,0x06);  
	lcm_dcs_write_seq_static(ctx,0x10,0x20);  
	lcm_dcs_write_seq_static(ctx,0x11,0x00);  
	lcm_dcs_write_seq_static(ctx,0x12,0x0E);  
	lcm_dcs_write_seq_static(ctx,0x13,0x1C);  
	lcm_dcs_write_seq_static(ctx,0x14,0x2A);  
	lcm_dcs_write_seq_static(ctx,0x15,0x38);  
	lcm_dcs_write_seq_static(ctx,0x16,0x46);  
	lcm_dcs_write_seq_static(ctx,0x17,0x54);  
	lcm_dcs_write_seq_static(ctx,0x18,0x62);  
	lcm_dcs_write_seq_static(ctx,0x19,0x69);  
	lcm_dcs_write_seq_static(ctx,0x1A,0x70);  
	lcm_dcs_write_seq_static(ctx,0x1B,0x77);  
	lcm_dcs_write_seq_static(ctx,0x1C,0x79);  
	lcm_dcs_write_seq_static(ctx,0x1D,0x7B);  
	lcm_dcs_write_seq_static(ctx,0x1E,0x7D);  
	lcm_dcs_write_seq_static(ctx,0x1F,0x7E);  
	lcm_dcs_write_seq_static(ctx,0x2E,0x01);  
	lcm_dcs_write_seq_static(ctx,0x2F,0x00);  
	lcm_dcs_write_seq_static(ctx,0x30,0x11);  
	lcm_dcs_write_seq_static(ctx,0x31,0x33);  
	lcm_dcs_write_seq_static(ctx,0x32,0x33);  
	lcm_dcs_write_seq_static(ctx,0x33,0x33);  
	lcm_dcs_write_seq_static(ctx,0x34,0x55);  
	lcm_dcs_write_seq_static(ctx,0x35,0x75);  //95
	lcm_dcs_write_seq_static(ctx,0x36,0x44);  
	lcm_dcs_write_seq_static(ctx,0x37,0x65);  
	lcm_dcs_write_seq_static(ctx,0x38,0x77);  
	lcm_dcs_write_seq_static(ctx,0x39,0x87);  
	lcm_dcs_write_seq_static(ctx,0x3A,0xA9);  
	lcm_dcs_write_seq_static(ctx,0x3B,0xCB);  //BA
	lcm_dcs_write_seq_static(ctx,0x3D,0xDD);  //
	lcm_dcs_write_seq_static(ctx,0x3E,0x0F);  //
	lcm_dcs_write_seq_static(ctx,0x3F,0x02);  
	lcm_dcs_write_seq_static(ctx,0x40,0x00);  
	lcm_dcs_write_seq_static(ctx,0x41,0x00);  
	lcm_dcs_write_seq_static(ctx,0x42,0x3E);  
	lcm_dcs_write_seq_static(ctx,0x43,0x3C);  
	lcm_dcs_write_seq_static(ctx,0x44,0x3A);  
	lcm_dcs_write_seq_static(ctx,0x45,0x38);  
	lcm_dcs_write_seq_static(ctx,0x46,0x38);  
	lcm_dcs_write_seq_static(ctx,0x47,0x38);  
	lcm_dcs_write_seq_static(ctx,0x48,0x36);  
	lcm_dcs_write_seq_static(ctx,0x49,0x36);  
	lcm_dcs_write_seq_static(ctx,0x4A,0x34);  
	lcm_dcs_write_seq_static(ctx,0x4B,0x34);  
	lcm_dcs_write_seq_static(ctx,0x4C,0x34);  
	lcm_dcs_write_seq_static(ctx,0x4D,0x33);  
	lcm_dcs_write_seq_static(ctx,0x4E,0x11);  
	lcm_dcs_write_seq_static(ctx,0xFF,0x20);  
	lcm_dcs_write_seq_static(ctx,0xF2,0x01);   //00             
	lcm_dcs_write_seq_static(ctx,0xFF,0x23);
	lcm_dcs_write_seq_static(ctx,0x01,0x00,0x00,0x00,0x08,0x00,0x43,0x00,0x62,0x00,0x77,0x00,0x8B,0x00,0x9B,0x00,0xA9);
	lcm_dcs_write_seq_static(ctx,0x02,0x00,0xB7,0x00,0xE2,0x01,0x05,0x01,0x39,0x01,0x63,0x01,0xA8,0x01,0xE2,0x01,0xE4);
	lcm_dcs_write_seq_static(ctx,0x03,0x02,0x23,0x02,0x70,0x02,0x9D,0x02,0xDE,0x03,0x0F,0x03,0x36,0x03,0x48,0x03,0x5A);
	lcm_dcs_write_seq_static(ctx,0x04,0x03,0x6D,0x03,0x82,0x03,0xA0,0x03,0xBE,0x03,0xE9,0x03,0xFF);
	lcm_dcs_write_seq_static(ctx,0x05,0x00,0x00,0x00,0x08,0x00,0x43,0x00,0x62,0x00,0x77,0x00,0x8B,0x00,0x9B,0x00,0xA9);
	                         
	lcm_dcs_write_seq_static(ctx,0x06,0x00,0xB7,0x00,0xE2,0x01,0x05,0x01,0x39,0x01,0x63,0x01,0xA8,0x01,0xE2,0x01,0xE4);
	lcm_dcs_write_seq_static(ctx,0x07,0x02,0x23,0x02,0x70,0x02,0x9D,0x02,0xDE,0x03,0x0F,0x03,0x36,0x03,0x48,0x03,0x5A);
	lcm_dcs_write_seq_static(ctx,0x08,0x03,0x6D,0x03,0x82,0x03,0xA0,0x03,0xBE,0x03,0xE9,0x03,0xFF);
	lcm_dcs_write_seq_static(ctx,0x09,0x00,0x00,0x00,0x08,0x00,0x43,0x00,0x62,0x00,0x77,0x00,0x8B,0x00,0x9B,0x00,0xA9);
	lcm_dcs_write_seq_static(ctx,0x0A,0x00,0xB7,0x00,0xE2,0x01,0x05,0x01,0x39,0x01,0x63,0x01,0xA8,0x01,0xE2,0x01,0xE4);
	lcm_dcs_write_seq_static(ctx,0x0B,0x02,0x23,0x02,0x70,0x02,0x9D,0x02,0xDE,0x03,0x0F,0x03,0x36,0x03,0x48,0x03,0x5A);
	lcm_dcs_write_seq_static(ctx,0x0C,0x03,0x6D,0x03,0x82,0x03,0xA0,0x03,0xBE,0x03,0xE9,0x03,0xFF);
	lcm_dcs_write_seq_static(ctx,0x0D,0x00,0x00,0x00,0x08,0x00,0x43,0x00,0x62,0x00,0x77,0x00,0x8B,0x00,0x9B,0x00,0xA9);
	                         
	lcm_dcs_write_seq_static(ctx,0x0E,0x00,0xB7,0x00,0xE2,0x01,0x05,0x01,0x39,0x01,0x63,0x01,0xA8,0x01,0xE2,0x01,0xE4);
	lcm_dcs_write_seq_static(ctx,0x0F,0x02,0x23,0x02,0x70,0x02,0x9D,0x02,0xDE,0x03,0x0F,0x03,0x36,0x03,0x48,0x03,0x5A);
	lcm_dcs_write_seq_static(ctx,0x10,0x03,0x6D,0x03,0x82,0x03,0xA0,0x03,0xBE,0x03,0xE9,0x03,0xFF);
	lcm_dcs_write_seq_static(ctx,0x11,0x00,0x00,0x00,0x08,0x00,0x43,0x00,0x62,0x00,0x77,0x00,0x8B,0x00,0x9B,0x00,0xA9);
	lcm_dcs_write_seq_static(ctx,0x12,0x00,0xB7,0x00,0xE2,0x01,0x05,0x01,0x39,0x01,0x63,0x01,0xA8,0x01,0xE2,0x01,0xE4);
	lcm_dcs_write_seq_static(ctx,0x13,0x02,0x23,0x02,0x70,0x02,0x9E,0x02,0xDE,0x03,0x0F,0x03,0x36,0x03,0x48,0x03,0x5A);
	lcm_dcs_write_seq_static(ctx,0x14,0x03,0x6D,0x03,0x82,0x03,0xA0,0x03,0xBE,0x03,0xE9,0x03,0xFF);
	lcm_dcs_write_seq_static(ctx,0x15,0x00,0x00,0x00,0x08,0x00,0x43,0x00,0x62,0x00,0x77,0x00,0x8B,0x00,0x9B,0x00,0xA9);
	lcm_dcs_write_seq_static(ctx,0x16,0x00,0xB7,0x00,0xE2,0x01,0x05,0x01,0x39,0x01,0x63,0x01,0xA8,0x01,0xE2,0x01,0xE4);
	lcm_dcs_write_seq_static(ctx,0x17,0x02,0x23,0x02,0x70,0x02,0x9E,0x02,0xDE,0x03,0x0F,0x03,0x36,0x03,0x48,0x03,0x5A);
	lcm_dcs_write_seq_static(ctx,0x18,0x03,0x6D,0x03,0x82,0x03,0xA0,0x03,0xBE,0x03,0xE9,0x03,0xFF);
                         
	lcm_dcs_write_seq_static(ctx,0xFF,0x22);
	lcm_dcs_write_seq_static(ctx,0x05,0x00);
	lcm_dcs_write_seq_static(ctx,0x08,0x04);
	lcm_dcs_write_seq_static(ctx,0xFF,0x10); 
	lcm_dcs_write_seq_static(ctx,0xBF,0x01);   //vesa 01 on 00 off
	lcm_dcs_write_seq_static(ctx,0x36,0x02);    
	lcm_dcs_write_seq_static(ctx,0x35,0x00); 
	lcm_dcs_write_seq_static(ctx,0xBA,0x03); 
	lcm_dcs_write_seq_static(ctx,0x69,0x00);    
	lcm_dcs_write_seq_static(ctx,0x70,0x17);  

	lcm_dcs_write_seq_static(ctx,0xFF,0x24);
	lcm_dcs_write_seq_static(ctx,0xFA,0x3A); 
	lcm_dcs_write_seq_static(ctx,0xFF,0x66,0x99,0x55);
	
	lcm_dcs_write_seq_static(ctx,0xFF,0x10); 
	lcm_dcs_write_seq_static(ctx, 0x11);
	mdelay(120);
	lcm_dcs_write_seq_static(ctx, 0x29);
	mdelay(20);
}

static int lcm_disable(struct drm_panel *panel)
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

static int lcm_unprepare(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);

	if (!ctx->prepared)
		return 0;
	pr_info("%s\n", __func__);
	lcm_dcs_write_seq_static(ctx,0xFF,0x55,0xAA,0x66);
	lcm_dcs_write_seq_static(ctx,0xFF,0x20);
	lcm_dcs_write_seq_static(ctx,0x4A,0x01);
	lcm_dcs_write_seq_static(ctx,0x48,0x10);
	lcm_dcs_write_seq_static(ctx,0x49,0x00);
	lcm_dcs_write_seq_static(ctx,0xFF,0x10);
	lcm_dcs_write_seq_static(ctx,0x28);
	msleep(50);
	lcm_dcs_write_seq_static(ctx,0x10);
	msleep(120);
	lcm_dcs_write_seq_static(ctx,0xFF,0xB3);
	lcm_dcs_write_seq_static(ctx,0x35,0xA8);
	lcm_dcs_write_seq_static(ctx,0x36,0x0A);
	lcm_dcs_write_seq_static(ctx,0x35,0xAA);
	lcm_dcs_write_seq_static(ctx,0xFF,0x26);
	lcm_dcs_write_seq_static(ctx,0x1D,0xA0,0xAA);
	lcm_dcs_write_seq_static(ctx,0x1F,0x01);

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

static int lcm_prepare(struct drm_panel *panel)
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
		lcm_unprepare(panel);

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

static int lcm_enable(struct drm_panel *panel)
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
#define HFP (24)
#define HSA (4)
#define HBP (16)
#define VFP (50)
#define VSA (1)
#define VBP (12)

static u32 fake_heigh = 2412;
static u32 fake_width = 1080;
static bool need_fake_resolution;

static struct drm_display_mode performance_mode_120 = {
	.clock = 840000,
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
	.clock = 840000,
	.hdisplay = 1080,
	.hsync_start = 1080 + HFP,
	.hsync_end = 1080 + HFP + HSA,
	.htotal = 1080 + HFP + HSA + HBP,//1136 //1133
	.vdisplay = 2412,
	.vsync_start = 2412 + 885,
	.vsync_end = 2412 + 885 + VSA,
	.vtotal = 2412 + 885 + VSA + VBP,//3732
	.vrefresh = 90,
};

static struct drm_display_mode default_mode = {
	.clock = 840000,
	.hdisplay = 1080,
	.hsync_start = 1080 + HFP,
	.hsync_end = 1080 + HFP + HSA,
	.htotal = 1080 + HFP + HSA + HBP,//1136 //1133
	.vdisplay = 2412,
	.vsync_start = 2412 + 2540,
	.vsync_end = 2412 + 2540 + VSA,
	.vtotal = 2412 + 2540 + VSA + VBP,//3732
	.vrefresh = 60,
};

#if defined(CONFIG_MTK_PANEL_EXT)
static struct mtk_panel_params ext_params_120 = {
	//.vfp_low_power = 2540,//60hz
	.pll_clk = 420,
	.cust_esd_check = 1,
	.esd_check_enable = 1,
	.lcm_esd_check_table[0] = {
		.cmd = 0x0A, .count = 1, .para_list[0] = 0x9C,
	},
	.data_rate = 840,
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
		.xmit_delay = 512,
		.dec_delay = 526,
		.scale_value = 32,
		.increment_interval = 287,
		.decrement_interval = 7,
		.line_bpg_offset = 12,
		.nfl_bpg_offset = 2235,
		.slice_bpg_offset = 2170,
		.initial_offset = 6144,
		.final_offset = 4336,
		.flatness_minqp = 3,
		.flatness_maxqp = 12,
		.rc_model_size = 8192,
		.rc_edge_factor = 6,
		.rc_quant_incr_limit0 = 11,
		.rc_quant_incr_limit1 = 11,
		.rc_tgt_offset_hi = 3,
		.rc_tgt_offset_lo = 3,
		},
	.dyn = {
		.switch_en = 1,
		.data_rate = 840,
		.hfp = 24,
		.vfp = 50,
	},
	.lfr_enable = 1,
	.lfr_minimum_fps = 60,
};
static struct mtk_panel_params ext_params_90 = {
	//.vfp_low_power = 2540,//60hz
	.pll_clk = 420,
	.cust_esd_check = 1,
	.esd_check_enable = 1,
	.lcm_esd_check_table[0] = {
		.cmd = 0x0A, .count = 1, .para_list[0] = 0x9C,
	},
	.data_rate = 840,
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
		.xmit_delay = 512,
		.dec_delay = 526,
		.scale_value = 32,
		.increment_interval = 287,
		.decrement_interval = 7,
		.line_bpg_offset = 12,
		.nfl_bpg_offset = 2235,
		.slice_bpg_offset = 2170,
		.initial_offset = 6144,
		.final_offset = 4336,
		.flatness_minqp = 3,
		.flatness_maxqp = 12,
		.rc_model_size = 8192,
		.rc_edge_factor = 6,
		.rc_quant_incr_limit0 = 11,
		.rc_quant_incr_limit1 = 11,
		.rc_tgt_offset_hi = 3,
		.rc_tgt_offset_lo = 3,
		},
	.dyn = {
		.switch_en = 1,
		.data_rate = 840,
		.hfp = 24,
		.vfp = 885,
	},
	.lfr_enable = 1,
	.lfr_minimum_fps = 60,
};
static struct mtk_panel_params ext_params_60 = {
	//.vfp_low_power = 2540,//60hz
	.pll_clk = 420,
	.cust_esd_check = 1,
	.esd_check_enable = 1,
	.lcm_esd_check_table[0] = {
		.cmd = 0x0A, .count = 1, .para_list[0] = 0x9C,
	},
	.data_rate = 840,
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
		.xmit_delay = 512,
		.dec_delay = 526,
		.scale_value = 32,
		.increment_interval = 287,
		.decrement_interval = 7,
		.line_bpg_offset = 12,
		.nfl_bpg_offset = 2235,
		.slice_bpg_offset = 2170,
		.initial_offset = 6144,
		.final_offset = 4336,
		.flatness_minqp = 3,
		.flatness_maxqp = 12,
		.rc_model_size = 8192,
		.rc_edge_factor = 6,
		.rc_quant_incr_limit0 = 11,
		.rc_quant_incr_limit1 = 11,
		.rc_tgt_offset_hi = 3,
		.rc_tgt_offset_lo = 3,
		},
	.dyn = {
		.switch_en = 1,
		.data_rate = 840,
		.hfp = 24,
		.vfp = 2412,
	},
	.lfr_enable = 1,
	.lfr_minimum_fps = 60,
};

struct drm_display_mode *get_mode_by_id(struct drm_panel *panel,
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

static int mtk_panel_ext_param_set(struct drm_panel *panel,
			 unsigned int mode)
{
	struct mtk_panel_ext *ext = find_panel_ext(panel);
	int ret = 0;
	struct drm_display_mode *m = get_mode_by_id(panel, mode);


	if (m->vrefresh == 60)
		ext->params = &ext_params_60;
	else if (m->vrefresh == 90)
		ext->params = &ext_params_90;
	else if (m->vrefresh == 120)
		ext->params = &ext_params_120;
	else
		ret = 1;
	if (!ret)
		current_fps = m->vrefresh;
	return ret;
}

static int panel_ext_reset(struct drm_panel *panel, int on)
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

static int panel_ata_check(struct drm_panel *panel)
{
	pr_info("%s success\n", __func__);
	return 1;
}

static int lcm_setbacklight_cmdq(void *dsi, dcs_write_gce cb,
	void *handle, unsigned int level)
{
	char bl_tb0[] = {0x51, 0xFF};

	bl_tb0[1] = level;

	if (!cb)
		return -1;

	cb(dsi, handle, bl_tb0, ARRAY_SIZE(bl_tb0));

	return 0;
}

//static int lcm_get_virtual_heigh(void)
//{
//	return VAC;
//}

//static int lcm_get_virtual_width(void)
//{
//	return HAC;
//}

static struct mtk_panel_funcs ext_funcs = {
	.reset = panel_ext_reset,
	.set_backlight_cmdq = lcm_setbacklight_cmdq,
	.ext_param_set = mtk_panel_ext_param_set,
	.ata_check = panel_ata_check,
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

static void change_drm_disp_mode_params(struct drm_display_mode *mode)
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

static int lcm_get_modes(struct drm_panel *panel)
{
	struct drm_display_mode *mode_60;
	struct drm_display_mode *mode_90;
	struct drm_display_mode *mode_120;
	if (need_fake_resolution)
		change_drm_disp_mode_params(&default_mode);

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
	panel->connector->display_info.width_mm = 64;
	panel->connector->display_info.height_mm = 129;

	return 1;
}

static const struct drm_panel_funcs lcm_drm_funcs = {
	.disable = lcm_disable,
	.unprepare = lcm_unprepare,
	.prepare = lcm_prepare,
	.enable = lcm_enable,
	.get_modes = lcm_get_modes,
};

static void check_is_need_fake_resolution(struct device *dev)
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

static int lcm_probe(struct mipi_dsi_device *dsi)
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
	ret = mtk_panel_ext_create(dev, &ext_params_60, &ext_funcs, &ctx->panel);
	if (ret < 0)
		return ret;
#endif
	check_is_need_fake_resolution(dev);
	printk("%s-\n", __func__);
	//prize add by anhengxuan for lcd hardware info 20220102 start
#if defined(CONFIG_PRIZE_HARDWARE_INFO)
    strcpy(current_lcm_info.chip,"Gc7302");
    strcpy(current_lcm_info.vendor,"galaxycore");
    sprintf(current_lcm_info.id,"0x%02x",0x02);
    strcpy(current_lcm_info.more,"1080*2412");
#endif
//prize add by anhengxuan for lcd hardware info 20220402 end
	return ret;
}

static int lcm_remove(struct mipi_dsi_device *dsi)
{
	struct lcm *ctx = mipi_dsi_get_drvdata(dsi);

	mipi_dsi_detach(dsi);
	drm_panel_remove(&ctx->panel);

	return 0;
}

static const struct of_device_id lcm_of_match[] = {
	{ .compatible = "truly,gc7302,vdo", },
	{ }
};

MODULE_DEVICE_TABLE(of, lcm_of_match);

static struct mipi_dsi_driver lcm_driver = {
	.probe = lcm_probe,
	.remove = lcm_remove,
	.driver = {
		.name = "panel-truly-gc7302-vdo",
		.owner = THIS_MODULE,
		.of_match_table = lcm_of_match,
	},
};

module_mipi_dsi_driver(lcm_driver);

MODULE_AUTHOR("MEDIATEK");
MODULE_DESCRIPTION("sc gc7302 VDO LCD Panel Driver");
MODULE_LICENSE("GPL v2");
