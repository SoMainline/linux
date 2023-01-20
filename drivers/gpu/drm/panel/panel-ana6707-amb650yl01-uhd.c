// SPDX-License-Identifier: GPL-2.0-only
// Copyright (c) 2023 FIXME
// Generated with linux-mdss-dsi-panel-driver-generator from vendor device tree:
//   Copyright (c) 2013, The Linux Foundation. All rights reserved. (FIXME)

#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_graph.h>
#include <linux/regulator/consumer.h>

#include <video/mipi_display.h>

#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>

#define HAS_DUAL_DSI BIT(0)

struct ana6707_amb650yl01 {
	struct drm_panel panel;
	struct mipi_dsi_device *dsi[2];
	struct regulator_bulk_data supplies[6];
	const struct mipi_dsi_device_info dsi_info;
	struct gpio_desc *reset_gpio;
	bool prepared;
};

static inline
struct ana6707_amb650yl01 *to_ana6707_amb650yl01(struct drm_panel *panel)
{
	return container_of(panel, struct ana6707_amb650yl01, panel);
}

static void ana6707_amb650yl01_reset(struct ana6707_amb650yl01 *ctx)
{
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	usleep_range(10000, 11000);
}

static int ana6707_amb650yl01_on(struct ana6707_amb650yl01 *ctx)
{
	struct mipi_dsi_device *dsi = ctx->dsi[0];
	struct device *dev = &ctx->dsi[0]->dev;
	int ret;

	ctx->dsi[0]->mode_flags |= MIPI_DSI_MODE_LPM;
	ctx->dsi[1]->mode_flags |= MIPI_DSI_MODE_LPM;

	ret = mipi_dsi_dcs_exit_sleep_mode(dsi);
	if (ret < 0) {
		dev_err(dev, "Failed to exit sleep mode: %d\n", ret);
		return ret;
	}
	usleep_range(15000, 16000);

	ret = mipi_dsi_compression_mode(dsi, false);
	if (ret < 0) {
		dev_err(dev, "Failed to set compression mode: %d\n", ret);
		return ret;
	}

	mipi_dsi_dcs_write_seq(dsi, 0xf0, 0x5a, 0x5a);
	mipi_dsi_dcs_write_seq(dsi, 0xb0, 0x16);
	mipi_dsi_dcs_write_seq(dsi, 0xe6, 0x44);
	mipi_dsi_dcs_write_seq(dsi, 0xf0, 0xa5, 0xa5);
	mipi_dsi_dcs_write_seq(dsi, 0xf0, 0x5a, 0x5a);
	mipi_dsi_dcs_write_seq(dsi, 0xb0, 0x04);
	mipi_dsi_dcs_write_seq(dsi, 0xbd, 0x00, 0xa0);
	mipi_dsi_dcs_write_seq(dsi, 0xf0, 0xa5, 0xa5);
	mipi_dsi_dcs_write_seq(dsi, 0xf0, 0x5a, 0x5a);
	mipi_dsi_dcs_write_seq(dsi, 0xed, 0x46, 0x00, 0x0e, 0x90);
	mipi_dsi_dcs_write_seq(dsi, 0xf0, 0xa5, 0xa5);
	mipi_dsi_dcs_write_seq(dsi, 0xf0, 0x5a, 0x5a);
	mipi_dsi_dcs_write_seq(dsi, 0x60, 0x00, 0xc0);
	mipi_dsi_dcs_write_seq(dsi, 0xf0, 0xa5, 0xa5);

	ret = mipi_dsi_dcs_set_column_address(dsi, 0x0000, 0x066b);
	if (ret < 0) {
		dev_err(dev, "Failed to set column address: %d\n", ret);
		return ret;
	}

	ret = mipi_dsi_dcs_set_page_address(dsi, 0x0000, 0x0eff);
	if (ret < 0) {
		dev_err(dev, "Failed to set page address: %d\n", ret);
		return ret;
	}

	ret = mipi_dsi_dcs_set_tear_on(dsi, MIPI_DSI_DCS_TEAR_MODE_VBLANK);
	if (ret < 0) {
		dev_err(dev, "Failed to set tear on: %d\n", ret);
		return ret;
	}

	ret = mipi_dsi_dcs_set_tear_scanline(dsi, 0x0000);
	if (ret < 0) {
		dev_err(dev, "Failed to set tear scanline: %d\n", ret);
		return ret;
	}

	mipi_dsi_dcs_write_seq(dsi, 0xf0, 0x5a, 0x5a);
	mipi_dsi_dcs_write_seq(dsi, 0xb0, 0x09);
	mipi_dsi_dcs_write_seq(dsi, 0xb9, 0x09, 0x78);
	mipi_dsi_dcs_write_seq(dsi, 0xf0, 0xa5, 0xa5);
	mipi_dsi_dcs_write_seq(dsi, MIPI_DCS_WRITE_CONTROL_DISPLAY, 0x20);
	mipi_dsi_dcs_write_seq(dsi, 0xf0, 0x5a, 0x5a);
	mipi_dsi_dcs_write_seq(dsi, 0xb0, 0x0b);
	mipi_dsi_dcs_write_seq(dsi, 0x92, 0x27, 0x25);
	mipi_dsi_dcs_write_seq(dsi, 0xf0, 0xa5, 0xa5);
	mipi_dsi_dcs_write_seq(dsi, 0xf0, 0x5a, 0x5a);
	mipi_dsi_dcs_write_seq(dsi, 0xd7, 0x82);
	mipi_dsi_dcs_write_seq(dsi, 0xf0, 0xa5, 0xa5);
	msleep(110);

	return 0;
}

static int ana6707_amb650yl01_off(struct ana6707_amb650yl01 *ctx)
{
	struct mipi_dsi_device *dsi = ctx->dsi[0];
	struct device *dev = &ctx->dsi[0]->dev;
	int ret;

	ctx->dsi[0]->mode_flags &= ~MIPI_DSI_MODE_LPM;
	ctx->dsi[1]->mode_flags &= ~MIPI_DSI_MODE_LPM;

	ret = mipi_dsi_dcs_set_display_off(dsi);
	if (ret < 0) {
		dev_err(dev, "Failed to set display off: %d\n", ret);
		return ret;
	}
	msleep(20);

	ret = mipi_dsi_dcs_enter_sleep_mode(dsi);
	if (ret < 0) {
		dev_err(dev, "Failed to enter sleep mode: %d\n", ret);
		return ret;
	}
	msleep(120);

	return 0;
}

static int ana6707_amb650yl01_prepare(struct drm_panel *panel)
{
	struct ana6707_amb650yl01 *ctx = to_ana6707_amb650yl01(panel);
	struct device *dev = &ctx->dsi[0]->dev;
	int ret;

	if (ctx->prepared)
		return 0;

	ret = regulator_bulk_enable(ARRAY_SIZE(ctx->supplies), ctx->supplies);
	if (ret < 0) {
		dev_err(dev, "Failed to enable regulators: %d\n", ret);
		return ret;
	}

	ana6707_amb650yl01_reset(ctx);

	ret = ana6707_amb650yl01_on(ctx);
	if (ret < 0) {
		dev_err(dev, "Failed to initialize panel: %d\n", ret);
		gpiod_set_value_cansleep(ctx->reset_gpio, 1);
		regulator_bulk_disable(ARRAY_SIZE(ctx->supplies), ctx->supplies);
		return ret;
	}

	ctx->prepared = true;
	return 0;
}

static int ana6707_amb650yl01_unprepare(struct drm_panel *panel)
{
	struct ana6707_amb650yl01 *ctx = to_ana6707_amb650yl01(panel);
	struct device *dev = &ctx->dsi[0]->dev;
	int ret;

	if (!ctx->prepared)
		return 0;

	ret = ana6707_amb650yl01_off(ctx);
	if (ret < 0)
		dev_err(dev, "Failed to un-initialize panel: %d\n", ret);

	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	regulator_bulk_disable(ARRAY_SIZE(ctx->supplies), ctx->supplies);

	ctx->prepared = false;
	return 0;
}

/*
 * All of the values here are x2 of what's specified in the downstream device-tree,
 * as the information there is essentially divided by the number of DSI hosts
 */
/* HACK! the framerate is cut to accomodate for non-DSC bw limits */
static const struct drm_display_mode ana6707_amb650yl01_mode = {
	.clock = (1644 + 72 + 16 + 16) * (3840 + 360 + 16 + 16) * 30 / 1000,
	.hdisplay = 1644,
	.hsync_start = 1644 + 72,
	.hsync_end = 1644 + 72 + 16,
	.htotal = 1644 + 72 + 16 + 16,
	.vdisplay = 3840,
	.vsync_start = 3840 + 360,
	.vsync_end = 3840 + 360 + 16,
	.vtotal = 3840 + 360 + 16 + 16,
	.width_mm = 65,
	.height_mm = 152,
};

static int ana6707_amb650yl01_get_modes(struct drm_panel *panel,
					    struct drm_connector *connector)
{
	struct drm_display_mode *mode;

	mode = drm_mode_duplicate(connector->dev, &ana6707_amb650yl01_mode);
	if (!mode)
		return -ENOMEM;

	drm_mode_set_name(mode);

	mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	connector->display_info.width_mm = mode->width_mm;
	connector->display_info.height_mm = mode->height_mm;
	drm_mode_probed_add(connector, mode);

	return 1;
}

static const struct drm_panel_funcs ana6707_amb650yl01_panel_funcs = {
	.prepare = ana6707_amb650yl01_prepare,
	.unprepare = ana6707_amb650yl01_unprepare,
	.get_modes = ana6707_amb650yl01_get_modes,
};

static int ana6707_amb650yl01_bl_update_status(struct backlight_device *bl)
{
	struct mipi_dsi_device *dsi = bl_get_data(bl);
	struct ana6707_amb650yl01 *ctx = mipi_dsi_get_drvdata(dsi);
	u16 brightness = backlight_get_brightness(bl);
	int ret;

	ctx->dsi[0]->mode_flags &= ~MIPI_DSI_MODE_LPM;
	ctx->dsi[1]->mode_flags &= ~MIPI_DSI_MODE_LPM;

	brightness = cpu_to_be16(brightness);

	ret = mipi_dsi_dcs_set_display_brightness(dsi, brightness);
	if (ret < 0)
		return ret;

	ctx->dsi[0]->mode_flags |= MIPI_DSI_MODE_LPM;
	ctx->dsi[1]->mode_flags |= MIPI_DSI_MODE_LPM;

	return 0;
}

static int ana6707_amb650yl01_bl_get_brightness(struct backlight_device *bl)
{
	struct mipi_dsi_device *dsi = bl_get_data(bl);
	struct ana6707_amb650yl01 *ctx = mipi_dsi_get_drvdata(dsi);
	u16 brightness;
	int ret;

	ctx->dsi[0]->mode_flags &= ~MIPI_DSI_MODE_LPM;
	ctx->dsi[1]->mode_flags &= ~MIPI_DSI_MODE_LPM;

	ret = mipi_dsi_dcs_get_display_brightness(dsi, &brightness);
	if (ret < 0)
		return ret;

	ctx->dsi[0]->mode_flags |= MIPI_DSI_MODE_LPM;
	ctx->dsi[1]->mode_flags |= MIPI_DSI_MODE_LPM;

	return be16_to_cpu(brightness);
}

static const struct backlight_ops ana6707_amb650yl01_bl_ops = {
	.update_status = ana6707_amb650yl01_bl_update_status,
	.get_brightness = ana6707_amb650yl01_bl_get_brightness,
};

static struct backlight_device *
ana6707_amb650yl01_create_backlight(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	const struct backlight_properties props = {
		.type = BACKLIGHT_RAW,
		.brightness = 4095,
		.max_brightness = 4095,
	};

	return devm_backlight_device_register(dev, dev_name(dev), dev, dsi,
					      &ana6707_amb650yl01_bl_ops, &props);
}

static const struct mipi_dsi_device_info info = {
	.type = "AMB650YL01",
	.channel = 0,
	.node = NULL,
};

static int ana6707_amb650yl01_probe(struct mipi_dsi_device *dsi)
{
	struct mipi_dsi_host *dsi_r_host;
	struct ana6707_amb650yl01 *ctx;
	struct device *dev = &dsi->dev;
	struct device_node *dsi_r;
	bool dual_dsi;
	int ret, i;

	ctx = devm_kzalloc(dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	ctx->supplies[0].supply = "vci";
	ctx->supplies[1].supply = "tsvddh";
	ctx->supplies[2].supply = "tsio";
	ctx->supplies[3].supply = "vddio";
	ctx->supplies[4].supply = "ab";
	ctx->supplies[5].supply = "ibb";
	ret = devm_regulator_bulk_get(dev, ARRAY_SIZE(ctx->supplies),
				      ctx->supplies);
	if (ret < 0)
		return dev_err_probe(dev, ret, "Failed to get regulators\n");

	ctx->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_ASIS);
	if (IS_ERR(ctx->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(ctx->reset_gpio), "Failed to get reset-gpios\n");

	dual_dsi = (u64)of_device_get_match_data(dev) & HAS_DUAL_DSI;

	/* If the panel is connected on two DSIs then DSI0 left, DSI1 right */
	if (dual_dsi) {
		dsi_r = of_graph_get_remote_node(dsi->dev.of_node, 1, -1);
		if (!dsi_r) {
			dev_err(dev, "Cannot get secondary DSI node.\n");
			return -ENODEV;
		}

		dsi_r_host = of_find_mipi_dsi_host_by_node(dsi_r);
		of_node_put(dsi_r);
		if (!dsi_r_host) {
			dev_err(dev, "Cannot get secondary DSI host\n");
			return -EPROBE_DEFER;
		}

		ctx->dsi[1] = mipi_dsi_device_register_full(dsi_r_host, &info);
		if (!ctx->dsi[1]) {
			dev_err(dev, "Cannot get secondary DSI node\n");
			return -ENODEV;
		}
	}

	ctx->dsi[0] = dsi;
	mipi_dsi_set_drvdata(dsi, ctx);

	drm_panel_init(&ctx->panel, dev, &ana6707_amb650yl01_panel_funcs,
		       DRM_MODE_CONNECTOR_DSI);

	ctx->panel.backlight = ana6707_amb650yl01_create_backlight(dsi);
	if (IS_ERR(ctx->panel.backlight))
		return dev_err_probe(dev, PTR_ERR(ctx->panel.backlight),
				     "Failed to create backlight\n");

	drm_panel_add(&ctx->panel);

	for (i = 0; i <= dual_dsi; i++) {
		ctx->dsi[i]->lanes = 4;
		ctx->dsi[i]->format = MIPI_DSI_FMT_RGB888;
		ctx->dsi[i]->mode_flags = MIPI_DSI_CLOCK_NON_CONTINUOUS;

		ret = mipi_dsi_attach(ctx->dsi[i]);
		if (ret < 0) {
			dev_err(dev, "Failed to attach to DSI%d: %d\n", i, ret);
			drm_panel_remove(&ctx->panel);
			return ret;
		}
	}

	return 0;
}

static void ana6707_amb650yl01_remove(struct mipi_dsi_device *dsi)
{
	struct ana6707_amb650yl01 *ctx = mipi_dsi_get_drvdata(dsi);
	int ret;

	ret = mipi_dsi_detach(dsi);
	if (ret < 0)
		dev_err(&dsi->dev, "Failed to detach from DSI host: %d\n", ret);

	drm_panel_remove(&ctx->panel);
}

static const struct of_device_id ana6707_amb650yl01_of_match[] = {
	{ .compatible = "sony,ana6707-amb650yl01", .data = (void *)HAS_DUAL_DSI },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, ana6707_amb650yl01_of_match);

static struct mipi_dsi_driver ana6707_amb650yl01_driver = {
	.probe = ana6707_amb650yl01_probe,
	.remove = ana6707_amb650yl01_remove,
	.driver = {
		.name = "panel-ana6707-amb650yl01",
		.of_match_table = ana6707_amb650yl01_of_match,
	},
};
module_mipi_dsi_driver(ana6707_amb650yl01_driver);

MODULE_AUTHOR("linux-mdss-dsi-panel-driver-generator <fix@me>"); // FIXME
MODULE_DESCRIPTION("DRM driver for 9");
MODULE_LICENSE("GPL");
