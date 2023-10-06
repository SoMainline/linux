// SPDX-License-Identifier: GPL-2.0-only
// Copyright (c) 2023 FIXME
// Generated with linux-mdss-dsi-panel-driver-generator from vendor device tree:
//   Copyright (c) 2013, The Linux Foundation. All rights reserved. (FIXME)

#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of.h>

#include <drm/display/drm_dsc.h>
#include <drm/display/drm_dsc_helper.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>

struct panel_sw43408a {
	struct drm_panel panel;
	struct mipi_dsi_device *dsi;
	struct drm_dsc_config dsc;
	struct gpio_desc *reset_gpio;
	bool prepared;
};

static inline
struct panel_sw43408a *to_panel_sw43408a(struct drm_panel *panel)
{
	return container_of(panel, struct panel_sw43408a, panel);
}

static void panel_sw43408a_reset(struct panel_sw43408a *ctx)
{
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	usleep_range(10000, 11000);
	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	msleep(40);
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	usleep_range(10000, 11000);
}

static int panel_sw43408a_on(struct panel_sw43408a *ctx)
{
	struct mipi_dsi_device *dsi = ctx->dsi;
	struct device *dev = &dsi->dev;
	int ret;

	dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	ret = mipi_dsi_compression_mode(dsi, true);
	if (ret < 0) {
		dev_err(dev, "Failed to set compression mode: %d\n", ret);
		return ret;
	}

	mipi_dsi_dcs_write_seq(dsi, 0x11, 0x00);
	mipi_dsi_generic_write_seq(dsi, 0x11,
				   0x00, 0x00, 0x89, 0x30, 0x80, 0x07, 0x08,
				   0x05, 0x46, 0x03, 0x84, 0x02, 0xa3, 0x02,
				   0xa3, 0x02, 0x00, 0x02, 0x51, 0x00, 0x20,
				   0x5f, 0x4b, 0x00, 0x09, 0x00, 0x0c, 0x00,
				   0x1c, 0x00, 0x18, 0x18, 0x00, 0x10, 0xf0,
				   0x03, 0x0c, 0x20, 0x00, 0x06, 0x0b, 0x0b,
				   0x33, 0x0e, 0x1c, 0x2a, 0x38, 0x46, 0x54,
				   0x62, 0x69, 0x70, 0x77, 0x79, 0x7b, 0x7d,
				   0x7e, 0x01, 0x02, 0x01, 0x00, 0x09, 0x40,
				   0x09, 0xbe, 0x19, 0xfc, 0x19, 0xfa, 0x19,
				   0xf8, 0x1a, 0x38, 0x1a, 0x78, 0x1a, 0xb6,
				   0x2a, 0xf6, 0x2b, 0x34, 0x2b, 0x74, 0x3b,
				   0x74, 0x6b, 0xf4, 0x00, 0x00, 0x00, 0x00,
				   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				   0x00);
	mipi_dsi_dcs_write_seq(dsi, 0xb0, 0xac);
	mipi_dsi_dcs_write_seq(dsi, 0x2a, 0x00, 0x00, 0x05, 0x45);
	mipi_dsi_dcs_write_seq(dsi, 0x2b, 0x00, 0x00, 0x07, 0x07);
	mipi_dsi_dcs_write_seq(dsi, 0x26, 0x02);
	mipi_dsi_dcs_write_seq(dsi, 0x44, 0x03, 0x84);
	mipi_dsi_dcs_write_seq(dsi, 0x35);
	mipi_dsi_dcs_write_seq(dsi, 0x51, 0x01, 0x47);
	mipi_dsi_dcs_write_seq(dsi, 0x53, 0x0c, 0x30);
	mipi_dsi_dcs_write_seq(dsi, 0x55, 0x04, 0x70, 0xdb, 0x00, 0x78, 0xdb);
	mipi_dsi_dcs_write_seq(dsi, 0xee, 0x24);
	mipi_dsi_dcs_write_seq(dsi, 0xfb, 0xac);
	mipi_dsi_dcs_write_seq(dsi, 0xb0, 0xca);
	mipi_dsi_dcs_write_seq(dsi, 0x11, 0x00);
	msleep(90);
	mipi_dsi_dcs_write_seq(dsi, 0x29, 0x00);
	msleep(50);
	mipi_dsi_dcs_write_seq(dsi, 0xb0, 0xac);
	mipi_dsi_dcs_write_seq(dsi, 0xe9,
			       0x32, 0x32, 0x55, 0x06, 0x00, 0x1c, 0x00, 0x00,
			       0x55, 0x50, 0x4b);
	mipi_dsi_dcs_write_seq(dsi, 0xd0,
			       0x80, 0x10, 0x81, 0x02, 0x0b, 0x00, 0x00, 0x00,
			       0x00, 0x00, 0x00, 0x00, 0x01, 0x0c, 0x00, 0xd4,
			       0x0b, 0x00, 0x11, 0x00, 0x1c);
	mipi_dsi_dcs_write_seq(dsi, 0xb0, 0xca);
	mipi_dsi_dcs_write_seq(dsi, 0x53, 0x0d, 0x30);

	return 0;
}

static int panel_sw43408a_off(struct panel_sw43408a *ctx)
{
	struct mipi_dsi_device *dsi = ctx->dsi;

	dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;

	mipi_dsi_dcs_write_seq(dsi, 0x28, 0x00);
	usleep_range(1000, 2000);
	mipi_dsi_dcs_write_seq(dsi, 0x10, 0x00);
	msleep(120);

	return 0;
}

static int panel_sw43408a_prepare(struct drm_panel *panel)
{
	struct panel_sw43408a *ctx = to_panel_sw43408a(panel);
	struct device *dev = &ctx->dsi->dev;
	struct drm_dsc_picture_parameter_set pps;
	int ret;

	if (ctx->prepared)
		return 0;

	panel_sw43408a_reset(ctx);

	ret = panel_sw43408a_on(ctx);
	if (ret < 0) {
		dev_err(dev, "Failed to initialize panel: %d\n", ret);
		gpiod_set_value_cansleep(ctx->reset_gpio, 1);
		return ret;
	}

	drm_dsc_pps_payload_pack(&pps, &ctx->dsc);

//	ret = mipi_dsi_picture_parameter_set(ctx->dsi, &pps);
//	if (ret < 0) {
//		dev_err(panel->dev, "failed to transmit PPS: %d\n", ret);
//		return ret;
//	}

	ret = mipi_dsi_compression_mode(ctx->dsi, true);
	if (ret < 0) {
		dev_err(dev, "failed to enable compression mode: %d\n", ret);
		return ret;
	}

	msleep(150); /* TODO: Is this panel-dependent? */

	ctx->prepared = true;
	return 0;
}

static int panel_sw43408a_unprepare(struct drm_panel *panel)
{
	struct panel_sw43408a *ctx = to_panel_sw43408a(panel);
	struct device *dev = &ctx->dsi->dev;
	int ret;

	if (!ctx->prepared)
		return 0;

	ret = panel_sw43408a_off(ctx);
	if (ret < 0)
		dev_err(dev, "Failed to un-initialize panel: %d\n", ret);

	gpiod_set_value_cansleep(ctx->reset_gpio, 1);

	ctx->prepared = false;
	return 0;
}

static const struct drm_display_mode panel_sw43408a_mode = {
	.clock = (1350 + 20 + 32 + 20) * (1800 + 20 + 4 + 20) * 60 / 1000,
	.hdisplay = 1350,
	.hsync_start = 1350 + 20,
	.hsync_end = 1350 + 20 + 32,
	.htotal = 1350 + 20 + 32 + 20,
	.vdisplay = 1800,
	.vsync_start = 1800 + 20,
	.vsync_end = 1800 + 20 + 4,
	.vtotal = 1800 + 20 + 4 + 20,
	.width_mm = 85,
	.height_mm = 114,
};

static int panel_sw43408a_get_modes(struct drm_panel *panel,
					      struct drm_connector *connector)
{
	struct drm_display_mode *mode;

	mode = drm_mode_duplicate(connector->dev, &panel_sw43408a_mode);
	if (!mode)
		return -ENOMEM;

	drm_mode_set_name(mode);

	mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	connector->display_info.width_mm = mode->width_mm;
	connector->display_info.height_mm = mode->height_mm;
	drm_mode_probed_add(connector, mode);

	return 1;
}

static const struct drm_panel_funcs panel_sw43408a_panel_funcs = {
	.prepare = panel_sw43408a_prepare,
	.unprepare = panel_sw43408a_unprepare,
	.get_modes = panel_sw43408a_get_modes,
};

static int panel_sw43408a_bl_update_status(struct backlight_device *bl)
{
	struct mipi_dsi_device *dsi = bl_get_data(bl);
	u16 brightness = backlight_get_brightness(bl);
	int ret;

	dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;

	ret = mipi_dsi_dcs_set_display_brightness_large(dsi, brightness);
	if (ret < 0)
		return ret;

	dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	return 0;
}

// TODO: Check if /sys/class/backlight/.../actual_brightness actually returns
// correct values. If not, remove this function.
static int panel_sw43408a_bl_get_brightness(struct backlight_device *bl)
{
	struct mipi_dsi_device *dsi = bl_get_data(bl);
	u16 brightness;
	int ret;

	dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;

	ret = mipi_dsi_dcs_get_display_brightness_large(dsi, &brightness);
	if (ret < 0)
		return ret;

	dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	return brightness;
}

static const struct backlight_ops panel_sw43408a_bl_ops = {
	.update_status = panel_sw43408a_bl_update_status,
	.get_brightness = panel_sw43408a_bl_get_brightness,
};

static struct backlight_device *
panel_sw43408a_create_backlight(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	const struct backlight_properties props = {
		.type = BACKLIGHT_RAW,
		.brightness = 1023,
		.max_brightness = 1023,
	};

	return devm_backlight_device_register(dev, dev_name(dev), dev, dsi,
					      &panel_sw43408a_bl_ops, &props);
}

static int panel_sw43408a_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct panel_sw43408a *ctx;
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
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO_BURST |
			  MIPI_DSI_MODE_NO_EOT_PACKET |
			  MIPI_DSI_CLOCK_NON_CONTINUOUS;

	drm_panel_init(&ctx->panel, dev, &panel_sw43408a_panel_funcs,
		       DRM_MODE_CONNECTOR_DSI);
	ctx->panel.prepare_prev_first = true;

	ctx->panel.backlight = panel_sw43408a_create_backlight(dsi);
	if (IS_ERR(ctx->panel.backlight))
		return dev_err_probe(dev, PTR_ERR(ctx->panel.backlight),
				     "Failed to create backlight\n");

	drm_panel_add(&ctx->panel);

	/* This panel only supports DSC; unconditionally enable it */
	dsi->dsc = &ctx->dsc;

	ctx->dsc.dsc_version_major = 1;
	ctx->dsc.dsc_version_minor = 1;

	// TODO: Pass slice_per_pkt = 2
	ctx->dsc.slice_height = 900;
	ctx->dsc.slice_width = 675;
	/*
	 * hdisplay should be read from the selected mode once
	 * it is passed back to drm_panel (in prepare?)
	 */
	WARN_ON(1350 % ctx->dsc.slice_width);
	ctx->dsc.slice_count = 1350 / ctx->dsc.slice_width;
	ctx->dsc.bits_per_component = 8;
	ctx->dsc.bits_per_pixel = 8 << 4; /* 4 fractional bits */
	ctx->dsc.block_pred_enable = true;

	ret = mipi_dsi_attach(dsi);
	if (ret < 0) {
		dev_err(dev, "Failed to attach to DSI host: %d\n", ret);
		drm_panel_remove(&ctx->panel);
		return ret;
	}

	return 0;
}

static void panel_sw43408a_remove(struct mipi_dsi_device *dsi)
{
	struct panel_sw43408a *ctx = mipi_dsi_get_drvdata(dsi);
	int ret;

	ret = mipi_dsi_detach(dsi);
	if (ret < 0)
		dev_err(&dsi->dev, "Failed to detach from DSI host: %d\n", ret);

	drm_panel_remove(&ctx->panel);
}

static const struct of_device_id panel_sw43408a_of_match[] = {
	{ .compatible = "microsoft,epsilon-sw43408a-lge-left" }, // FIXME
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, panel_sw43408a_of_match);

static struct mipi_dsi_driver panel_sw43408a_driver = {
	.probe = panel_sw43408a_probe,
	.remove = panel_sw43408a_remove,
	.driver = {
		.name = "panel-panel-lg-v2-amoled-c3-dv",
		.of_match_table = panel_sw43408a_of_match,
	},
};
module_mipi_dsi_driver(panel_sw43408a_driver);

MODULE_AUTHOR("linux-mdss-dsi-panel-driver-generator <fix@me>"); // FIXME
MODULE_DESCRIPTION("DRM driver for dsi panel lg v2 amoled cmd C3 dv");
MODULE_LICENSE("GPL");
