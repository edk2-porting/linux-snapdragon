// SPDX-License-Identifier: GPL-2.0-only
/* Copyright (c) 2022 BigfootACA <bigfoot@classfun.cn>
 * Generated with linux-mdss-dsi-panel-driver-generator from vendor device tree:
 * Copyright (c) 2022, The Linux Foundation. All rights reserved.
 */

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of.h>

#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>

struct nt51021_boe_wuxga {
	struct drm_panel panel;
	struct mipi_dsi_device *dsi;
	struct gpio_desc *reset_gpio;
	bool prepared;
};

static inline
struct nt51021_boe_wuxga *to_nt51021_boe_wuxga(struct drm_panel *panel)
{
	return container_of(panel, struct nt51021_boe_wuxga, panel);
}

#define dsi_generic_write_seq(dsi, seq...) do {				\
		static const u8 d[] = { seq };				\
		int ret;						\
		ret = mipi_dsi_generic_write(dsi, d, ARRAY_SIZE(d));	\
		if (ret < 0)						\
			return ret;					\
	} while (0)

static void nt51021_boe_wuxga_reset(struct nt51021_boe_wuxga *ctx)
{
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	usleep_range(10000, 11000);
}

static int nt51021_boe_wuxga_on(struct nt51021_boe_wuxga *ctx)
{
	struct mipi_dsi_device *dsi = ctx->dsi;
	struct device *dev = &dsi->dev;
	int ret;

	dsi_generic_write_seq(dsi, 0x8f, 0xa5);
	usleep_range(1000, 2000);
	dsi_generic_write_seq(dsi, 0x01, 0x00);
	msleep(20);
	dsi_generic_write_seq(dsi, 0x8f, 0xa5);
	usleep_range(1000, 2000);
	dsi_generic_write_seq(dsi, 0x83, 0x00);
	dsi_generic_write_seq(dsi, 0x84, 0x00);
	dsi_generic_write_seq(dsi, 0x8c, 0x8e);
	dsi_generic_write_seq(dsi, 0xcd, 0x6c);
	dsi_generic_write_seq(dsi, 0xc0, 0x8b);
	dsi_generic_write_seq(dsi, 0xc8, 0xf0);
	dsi_generic_write_seq(dsi, 0x8b, 0x10);
	dsi_generic_write_seq(dsi, 0xa9, 0x20);
	dsi_generic_write_seq(dsi, 0x97, 0x00);
	dsi_generic_write_seq(dsi, 0xfa, 0x15);
	dsi_generic_write_seq(dsi, 0xfd, 0x1b);
	dsi_generic_write_seq(dsi, 0x83, 0xaa);
	dsi_generic_write_seq(dsi, 0x84, 0x11);
	dsi_generic_write_seq(dsi, 0xa9, 0x4b);
	dsi_generic_write_seq(dsi, 0x85, 0x04);
	dsi_generic_write_seq(dsi, 0x86, 0x08);
	dsi_generic_write_seq(dsi, 0x98, 0xc1);
	dsi_generic_write_seq(dsi, 0x83, 0xaa);
	dsi_generic_write_seq(dsi, 0x84, 0x11);
	dsi_generic_write_seq(dsi, 0xc0, 0x05);
	dsi_generic_write_seq(dsi, 0xc1, 0x0c);
	dsi_generic_write_seq(dsi, 0xc2, 0x22);
	dsi_generic_write_seq(dsi, 0xc3, 0x34);
	dsi_generic_write_seq(dsi, 0xc4, 0x44);
	dsi_generic_write_seq(dsi, 0xc5, 0x52);
	dsi_generic_write_seq(dsi, 0xc6, 0x5d);
	dsi_generic_write_seq(dsi, 0xc7, 0x66);
	dsi_generic_write_seq(dsi, 0xc8, 0x70);
	dsi_generic_write_seq(dsi, 0xc9, 0x00);
	dsi_generic_write_seq(dsi, 0xca, 0x11);
	dsi_generic_write_seq(dsi, 0xcb, 0x3b);
	dsi_generic_write_seq(dsi, 0xcc, 0x53);
	dsi_generic_write_seq(dsi, 0xcd, 0x69);
	dsi_generic_write_seq(dsi, 0xce, 0x6a);
	dsi_generic_write_seq(dsi, 0xcf, 0x6c);
	dsi_generic_write_seq(dsi, 0xd0, 0x6b);
	dsi_generic_write_seq(dsi, 0xd1, 0x75);
	dsi_generic_write_seq(dsi, 0xd2, 0x83);
	dsi_generic_write_seq(dsi, 0xd3, 0x99);
	dsi_generic_write_seq(dsi, 0xd4, 0x9a);
	dsi_generic_write_seq(dsi, 0xd5, 0xd0);
	dsi_generic_write_seq(dsi, 0xd6, 0xd6);
	dsi_generic_write_seq(dsi, 0xd7, 0xdb);
	dsi_generic_write_seq(dsi, 0xd8, 0xe1);
	dsi_generic_write_seq(dsi, 0xd9, 0xe7);
	dsi_generic_write_seq(dsi, 0xda, 0xed);
	dsi_generic_write_seq(dsi, 0xdb, 0xf4);
	dsi_generic_write_seq(dsi, 0xdc, 0xfb);
	dsi_generic_write_seq(dsi, 0xdd, 0xff);
	dsi_generic_write_seq(dsi, 0xde, 0xff);
	dsi_generic_write_seq(dsi, 0xdf, 0x0f);
	dsi_generic_write_seq(dsi, 0xe0, 0x15);
	dsi_generic_write_seq(dsi, 0xe1, 0x1c);
	dsi_generic_write_seq(dsi, 0xe2, 0x32);
	dsi_generic_write_seq(dsi, 0xe3, 0x44);
	dsi_generic_write_seq(dsi, 0xe4, 0x52);
	dsi_generic_write_seq(dsi, 0xe5, 0x60);
	dsi_generic_write_seq(dsi, 0xe6, 0x6b);
	dsi_generic_write_seq(dsi, 0xe7, 0x74);
	dsi_generic_write_seq(dsi, 0xe8, 0x7e);
	dsi_generic_write_seq(dsi, 0xe9, 0x14);
	dsi_generic_write_seq(dsi, 0xea, 0x25);
	dsi_generic_write_seq(dsi, 0xeb, 0x4d);
	dsi_generic_write_seq(dsi, 0xec, 0x63);
	dsi_generic_write_seq(dsi, 0xed, 0x71);
	dsi_generic_write_seq(dsi, 0xee, 0x6a);
	dsi_generic_write_seq(dsi, 0xef, 0x6c);
	dsi_generic_write_seq(dsi, 0xf0, 0x6b);
	dsi_generic_write_seq(dsi, 0xf1, 0x75);
	dsi_generic_write_seq(dsi, 0xf2, 0x83);
	dsi_generic_write_seq(dsi, 0xf3, 0x99);
	dsi_generic_write_seq(dsi, 0xf4, 0x9a);
	dsi_generic_write_seq(dsi, 0xf5, 0xd0);
	dsi_generic_write_seq(dsi, 0xf6, 0xd6);
	dsi_generic_write_seq(dsi, 0xf7, 0xdb);
	dsi_generic_write_seq(dsi, 0xf8, 0xe1);
	dsi_generic_write_seq(dsi, 0xf9, 0xe7);
	dsi_generic_write_seq(dsi, 0xfa, 0xed);
	dsi_generic_write_seq(dsi, 0xfb, 0xf4);
	dsi_generic_write_seq(dsi, 0xfc, 0xfb);
	dsi_generic_write_seq(dsi, 0xfd, 0xff);
	dsi_generic_write_seq(dsi, 0xfe, 0xff);
	dsi_generic_write_seq(dsi, 0xff, 0x0f);
	dsi_generic_write_seq(dsi, 0x83, 0xbb);
	dsi_generic_write_seq(dsi, 0x84, 0x22);
	dsi_generic_write_seq(dsi, 0xa1, 0xff);
	dsi_generic_write_seq(dsi, 0xa2, 0xfe);
	dsi_generic_write_seq(dsi, 0xa3, 0xfa);
	dsi_generic_write_seq(dsi, 0xa4, 0xf7);
	dsi_generic_write_seq(dsi, 0xa5, 0xf3);
	dsi_generic_write_seq(dsi, 0xa6, 0xf1);
	dsi_generic_write_seq(dsi, 0xa7, 0xed);
	dsi_generic_write_seq(dsi, 0xa8, 0xeb);
	dsi_generic_write_seq(dsi, 0xa9, 0xe9);
	dsi_generic_write_seq(dsi, 0xaa, 0xe6);
	dsi_generic_write_seq(dsi, 0xaf, 0x00);
	dsi_generic_write_seq(dsi, 0xb0, 0x35);
	dsi_generic_write_seq(dsi, 0xb1, 0x89);
	dsi_generic_write_seq(dsi, 0xb2, 0x99);
	dsi_generic_write_seq(dsi, 0xb3, 0x99);
	dsi_generic_write_seq(dsi, 0xb4, 0x0d);
	dsi_generic_write_seq(dsi, 0xb5, 0x1a);
	dsi_generic_write_seq(dsi, 0xb6, 0x16);
	dsi_generic_write_seq(dsi, 0x9a, 0x10);
	dsi_generic_write_seq(dsi, 0x9b, 0x00);
	dsi_generic_write_seq(dsi, 0x96, 0xe6);
	dsi_generic_write_seq(dsi, 0x99, 0x06);
	dsi_generic_write_seq(dsi, 0xc0, 0x05);
	dsi_generic_write_seq(dsi, 0xc1, 0x0c);
	dsi_generic_write_seq(dsi, 0xc2, 0x22);
	dsi_generic_write_seq(dsi, 0xc3, 0x34);
	dsi_generic_write_seq(dsi, 0xc4, 0x44);
	dsi_generic_write_seq(dsi, 0xc5, 0x52);
	dsi_generic_write_seq(dsi, 0xc6, 0x5d);
	dsi_generic_write_seq(dsi, 0xc7, 0x66);
	dsi_generic_write_seq(dsi, 0xc8, 0x70);
	dsi_generic_write_seq(dsi, 0xc9, 0x00);
	dsi_generic_write_seq(dsi, 0xca, 0x11);
	dsi_generic_write_seq(dsi, 0xcb, 0x3b);
	dsi_generic_write_seq(dsi, 0xcc, 0x53);
	dsi_generic_write_seq(dsi, 0xcd, 0x69);
	dsi_generic_write_seq(dsi, 0xce, 0x6a);
	dsi_generic_write_seq(dsi, 0xcf, 0x6c);
	dsi_generic_write_seq(dsi, 0xd0, 0x6b);
	dsi_generic_write_seq(dsi, 0xd1, 0x75);
	dsi_generic_write_seq(dsi, 0xd2, 0x83);
	dsi_generic_write_seq(dsi, 0xd3, 0x99);
	dsi_generic_write_seq(dsi, 0xd4, 0x9a);
	dsi_generic_write_seq(dsi, 0xd5, 0xd0);
	dsi_generic_write_seq(dsi, 0xd6, 0xd6);
	dsi_generic_write_seq(dsi, 0xd7, 0xdb);
	dsi_generic_write_seq(dsi, 0xd8, 0xe1);
	dsi_generic_write_seq(dsi, 0xd9, 0xe7);
	dsi_generic_write_seq(dsi, 0xda, 0xed);
	dsi_generic_write_seq(dsi, 0xdb, 0xf4);
	dsi_generic_write_seq(dsi, 0xdc, 0xfb);
	dsi_generic_write_seq(dsi, 0xdd, 0xff);
	dsi_generic_write_seq(dsi, 0xde, 0xff);
	dsi_generic_write_seq(dsi, 0xdf, 0x0f);
	dsi_generic_write_seq(dsi, 0xe0, 0x15);
	dsi_generic_write_seq(dsi, 0xe1, 0x1c);
	dsi_generic_write_seq(dsi, 0xe2, 0x32);
	dsi_generic_write_seq(dsi, 0xe3, 0x44);
	dsi_generic_write_seq(dsi, 0xe4, 0x52);
	dsi_generic_write_seq(dsi, 0xe5, 0x60);
	dsi_generic_write_seq(dsi, 0xe6, 0x6b);
	dsi_generic_write_seq(dsi, 0xe7, 0x74);
	dsi_generic_write_seq(dsi, 0xe8, 0x7e);
	dsi_generic_write_seq(dsi, 0xe9, 0x14);
	dsi_generic_write_seq(dsi, 0xea, 0x25);
	dsi_generic_write_seq(dsi, 0xeb, 0x4d);
	dsi_generic_write_seq(dsi, 0xec, 0x63);
	dsi_generic_write_seq(dsi, 0xed, 0x71);
	dsi_generic_write_seq(dsi, 0xee, 0x6a);
	dsi_generic_write_seq(dsi, 0xef, 0x6c);
	dsi_generic_write_seq(dsi, 0xf0, 0x6b);
	dsi_generic_write_seq(dsi, 0xf1, 0x75);
	dsi_generic_write_seq(dsi, 0xf2, 0x83);
	dsi_generic_write_seq(dsi, 0xf3, 0x99);
	dsi_generic_write_seq(dsi, 0xf4, 0x9a);
	dsi_generic_write_seq(dsi, 0xf5, 0xd0);
	dsi_generic_write_seq(dsi, 0xf6, 0xd6);
	dsi_generic_write_seq(dsi, 0xf7, 0xdb);
	dsi_generic_write_seq(dsi, 0xf8, 0xe1);
	dsi_generic_write_seq(dsi, 0xf9, 0xe7);
	dsi_generic_write_seq(dsi, 0xfa, 0xed);
	dsi_generic_write_seq(dsi, 0xfb, 0xf4);
	dsi_generic_write_seq(dsi, 0xfc, 0xfb);
	dsi_generic_write_seq(dsi, 0xfd, 0xff);
	dsi_generic_write_seq(dsi, 0xfe, 0xff);
	dsi_generic_write_seq(dsi, 0xff, 0x0f);
	dsi_generic_write_seq(dsi, 0x83, 0xcc);
	dsi_generic_write_seq(dsi, 0x84, 0x33);
	dsi_generic_write_seq(dsi, 0xc0, 0x05);
	dsi_generic_write_seq(dsi, 0xc1, 0x0c);
	dsi_generic_write_seq(dsi, 0xc2, 0x22);
	dsi_generic_write_seq(dsi, 0xc3, 0x34);
	dsi_generic_write_seq(dsi, 0xc4, 0x44);
	dsi_generic_write_seq(dsi, 0xc5, 0x52);
	dsi_generic_write_seq(dsi, 0xc6, 0x5d);
	dsi_generic_write_seq(dsi, 0xc7, 0x66);
	dsi_generic_write_seq(dsi, 0xc8, 0x70);
	dsi_generic_write_seq(dsi, 0xc9, 0x00);
	dsi_generic_write_seq(dsi, 0xca, 0x11);
	dsi_generic_write_seq(dsi, 0xcb, 0x3b);
	dsi_generic_write_seq(dsi, 0xcc, 0x53);
	dsi_generic_write_seq(dsi, 0xcd, 0x69);
	dsi_generic_write_seq(dsi, 0xce, 0x6a);
	dsi_generic_write_seq(dsi, 0xcf, 0x6c);
	dsi_generic_write_seq(dsi, 0xd0, 0x6b);
	dsi_generic_write_seq(dsi, 0xd1, 0x75);
	dsi_generic_write_seq(dsi, 0xd2, 0x83);
	dsi_generic_write_seq(dsi, 0xd3, 0x99);
	dsi_generic_write_seq(dsi, 0xd4, 0x9a);
	dsi_generic_write_seq(dsi, 0xd5, 0xd0);
	dsi_generic_write_seq(dsi, 0xd6, 0xd6);
	dsi_generic_write_seq(dsi, 0xd7, 0xdb);
	dsi_generic_write_seq(dsi, 0xd8, 0xe1);
	dsi_generic_write_seq(dsi, 0xd9, 0xe7);
	dsi_generic_write_seq(dsi, 0xda, 0xed);
	dsi_generic_write_seq(dsi, 0xdb, 0xf4);
	dsi_generic_write_seq(dsi, 0xdc, 0xfb);
	dsi_generic_write_seq(dsi, 0xdd, 0xff);
	dsi_generic_write_seq(dsi, 0xde, 0xff);
	dsi_generic_write_seq(dsi, 0xdf, 0x0f);
	dsi_generic_write_seq(dsi, 0xe0, 0x15);
	dsi_generic_write_seq(dsi, 0xe1, 0x1c);
	dsi_generic_write_seq(dsi, 0xe2, 0x32);
	dsi_generic_write_seq(dsi, 0xe3, 0x44);
	dsi_generic_write_seq(dsi, 0xe4, 0x52);
	dsi_generic_write_seq(dsi, 0xe5, 0x60);
	dsi_generic_write_seq(dsi, 0xe6, 0x6b);
	dsi_generic_write_seq(dsi, 0xe7, 0x74);
	dsi_generic_write_seq(dsi, 0xe8, 0x7e);
	dsi_generic_write_seq(dsi, 0xe9, 0x14);
	dsi_generic_write_seq(dsi, 0xea, 0x25);
	dsi_generic_write_seq(dsi, 0xeb, 0x4d);
	dsi_generic_write_seq(dsi, 0xec, 0x63);
	dsi_generic_write_seq(dsi, 0xed, 0x71);
	dsi_generic_write_seq(dsi, 0xee, 0x6a);
	dsi_generic_write_seq(dsi, 0xef, 0x6c);
	dsi_generic_write_seq(dsi, 0xf0, 0x6b);
	dsi_generic_write_seq(dsi, 0xf1, 0x75);
	dsi_generic_write_seq(dsi, 0xf2, 0x83);
	dsi_generic_write_seq(dsi, 0xf3, 0x99);
	dsi_generic_write_seq(dsi, 0xf4, 0x9a);
	dsi_generic_write_seq(dsi, 0xf5, 0xd0);
	dsi_generic_write_seq(dsi, 0xf6, 0xd6);
	dsi_generic_write_seq(dsi, 0xf7, 0xdb);
	dsi_generic_write_seq(dsi, 0xf8, 0xe1);
	dsi_generic_write_seq(dsi, 0xf9, 0xe7);
	dsi_generic_write_seq(dsi, 0xfa, 0xed);
	dsi_generic_write_seq(dsi, 0xfb, 0xf4);
	dsi_generic_write_seq(dsi, 0xfc, 0xfb);
	dsi_generic_write_seq(dsi, 0xfd, 0xff);
	dsi_generic_write_seq(dsi, 0xfe, 0xff);
	dsi_generic_write_seq(dsi, 0xff, 0x0f);
	dsi_generic_write_seq(dsi, 0x83, 0x00);
	dsi_generic_write_seq(dsi, 0x84, 0x00);
	dsi_generic_write_seq(dsi, 0x9f, 0xff);
	dsi_generic_write_seq(dsi, 0x83, 0xbb);
	dsi_generic_write_seq(dsi, 0x84, 0x22);
	dsi_generic_write_seq(dsi, 0x94, 0xba);

	ret = mipi_dsi_dcs_exit_sleep_mode(dsi);
	if (ret < 0) {
		dev_err(dev, "Failed to exit sleep mode: %d\n", ret);
		return ret;
	}
	usleep_range(5000, 6000);

	dsi_generic_write_seq(dsi, 0x8f, 0x00);
	usleep_range(1000, 2000);

	return 0;
}

static int nt51021_boe_wuxga_off(struct nt51021_boe_wuxga *ctx)
{
	struct mipi_dsi_device *dsi = ctx->dsi;
	struct device *dev = &dsi->dev;
	int ret;

	dsi_generic_write_seq(dsi, 0x8f, 0xa5);
	msleep(20);

	ret = mipi_dsi_dcs_set_display_off(dsi);
	if (ret < 0) {
		dev_err(dev, "Failed to set display off: %d\n", ret);
		return ret;
	}
	msleep(130);

	ret = mipi_dsi_dcs_enter_sleep_mode(dsi);
	if (ret < 0) {
		dev_err(dev, "Failed to enter sleep mode: %d\n", ret);
		return ret;
	}
	msleep(70);

	dsi_generic_write_seq(dsi, 0x8f, 0x00);
	usleep_range(4000, 5000);

	return 0;
}

static int nt51021_boe_wuxga_prepare(struct drm_panel *panel)
{
	struct nt51021_boe_wuxga *ctx = to_nt51021_boe_wuxga(panel);
	struct device *dev = &ctx->dsi->dev;
	int ret;

	if (ctx->prepared)
		return 0;

	nt51021_boe_wuxga_reset(ctx);

	ret = nt51021_boe_wuxga_on(ctx);
	if (ret < 0) {
		dev_err(dev, "Failed to initialize panel: %d\n", ret);
		gpiod_set_value_cansleep(ctx->reset_gpio, 1);
		return ret;
	}

	ctx->prepared = true;
	return 0;
}

static int nt51021_boe_wuxga_unprepare(struct drm_panel *panel)
{
	struct nt51021_boe_wuxga *ctx = to_nt51021_boe_wuxga(panel);
	struct device *dev = &ctx->dsi->dev;
	int ret;

	if (!ctx->prepared)
		return 0;

	ret = nt51021_boe_wuxga_off(ctx);
	if (ret < 0)
		dev_err(dev, "Failed to un-initialize panel: %d\n", ret);

	gpiod_set_value_cansleep(ctx->reset_gpio, 1);

	ctx->prepared = false;
	return 0;
}

static const struct drm_display_mode nt51021_boe_wuxga_mode = {
	.clock = (1200 + 84 + 4 + 60) * (1920 + 36 + 1 + 25) * 60 / 1000,
	.hdisplay = 1200,
	.hsync_start = 1200 + 84,
	.hsync_end = 1200 + 84 + 4,
	.htotal = 1200 + 84 + 4 + 60,
	.vdisplay = 1920,
	.vsync_start = 1920 + 36,
	.vsync_end = 1920 + 36 + 1,
	.vtotal = 1920 + 36 + 1 + 25,
	.width_mm = 107,
	.height_mm = 172,
};

static int nt51021_boe_wuxga_get_modes(struct drm_panel *panel,
				       struct drm_connector *connector)
{
	struct drm_display_mode *mode;

	mode = drm_mode_duplicate(connector->dev, &nt51021_boe_wuxga_mode);
	if (!mode)
		return -ENOMEM;

	drm_mode_set_name(mode);

	mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	connector->display_info.width_mm = mode->width_mm;
	connector->display_info.height_mm = mode->height_mm;
	drm_mode_probed_add(connector, mode);

	return 1;
}

static const struct drm_panel_funcs nt51021_boe_wuxga_panel_funcs = {
	.prepare = nt51021_boe_wuxga_prepare,
	.unprepare = nt51021_boe_wuxga_unprepare,
	.get_modes = nt51021_boe_wuxga_get_modes,
};

static int nt51021_boe_wuxga_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct nt51021_boe_wuxga *ctx;
	int ret;

	ctx = devm_kzalloc(dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	ctx->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(ctx->reset_gpio),
				     "Failed to get reset-gpios\n");

	ctx->dsi = dsi;
	mipi_dsi_set_drvdata(dsi, ctx);

	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_BURST |
			  MIPI_DSI_MODE_NO_EOT_PACKET | MIPI_DSI_MODE_LPM;

	drm_panel_init(&ctx->panel, dev, &nt51021_boe_wuxga_panel_funcs,
		       DRM_MODE_CONNECTOR_DSI);

	ret = drm_panel_of_backlight(&ctx->panel);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to get backlight\n");

	drm_panel_add(&ctx->panel);

	ret = mipi_dsi_attach(dsi);
	if (ret < 0) {
		dev_err(dev, "Failed to attach to DSI host: %d\n", ret);
		drm_panel_remove(&ctx->panel);
		return ret;
	}

	return 0;
}

static int nt51021_boe_wuxga_remove(struct mipi_dsi_device *dsi)
{
	struct nt51021_boe_wuxga *ctx = mipi_dsi_get_drvdata(dsi);
	int ret;

	ret = mipi_dsi_detach(dsi);
	if (ret < 0)
		dev_err(&dsi->dev, "Failed to detach from DSI host: %d\n", ret);

	drm_panel_remove(&ctx->panel);

	return 0;
}

static const struct of_device_id nt51021_boe_wuxga_of_match[] = {
	{ .compatible = "mdss,nt51021-boe-wuxga" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, nt51021_boe_wuxga_of_match);

static struct mipi_dsi_driver nt51021_boe_wuxga_driver = {
	.probe = nt51021_boe_wuxga_probe,
	.remove = nt51021_boe_wuxga_remove,
	.driver = {
		.name = "panel-nt51021-boe-wuxga",
		.of_match_table = nt51021_boe_wuxga_of_match,
	},
};
module_mipi_dsi_driver(nt51021_boe_wuxga_driver);

MODULE_AUTHOR("BigfootACA <bigfoot@classfun.cn>");
MODULE_DESCRIPTION("DRM driver for NT51021 BOE WUXGA");
MODULE_LICENSE("GPL");
