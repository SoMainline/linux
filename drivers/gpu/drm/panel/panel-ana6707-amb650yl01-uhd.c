// SPDX-License-Identifier: GPL-2.0-only
// Copyright (c) 2023 Marijn Suijten <marijn.suijten@somainline.org>

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
#include <drm/drm_probe_helper.h>
#include <drm/display/drm_dsc.h>
#include <drm/display/drm_dsc_helper.h>

static const bool enable_4k = false, is_120fps = false;

struct ana6707_amb650yl01 {
	struct drm_panel panel;
	struct mipi_dsi_device *dsi[2];
	struct drm_dsc_config dsc;
	struct regulator_bulk_data supplies[7];
	// const struct mipi_dsi_device_info dsi_info;
	struct gpio_desc *reset_gpio;
	bool prepared;
};

static inline struct ana6707_amb650yl01 *
to_ana6707_amb650yl01(struct drm_panel *panel)
{
	return container_of(panel, struct ana6707_amb650yl01, panel);
}

static void ana6707_amb650yl01_reset(struct ana6707_amb650yl01 *ctx)
{
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	usleep_range(10000, 11000);
	// gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	// usleep_range(5000, 6000);
	// gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	// usleep_range(1000, 2000);
	// gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	// usleep_range(10000, 11000);
}

static int ana6707_amb650yl01_on(struct ana6707_amb650yl01 *ctx)
{
	struct mipi_dsi_device *dsi = ctx->dsi[0];
	struct device *dev = &dsi->dev;
	u16 resX, resY, scanline;
	int ret;

	if (enable_4k) {
		scanline = 0;
		resX = 1644 - 1;
		resY = 3840 - 1;
	} else {
		scanline = 0xf00;
		resX = 1096 - 1;
		resY = 2560 - 1;
	}

	ctx->dsi[0]->mode_flags |= MIPI_DSI_MODE_LPM;
	if (ctx->dsi[1])
		ctx->dsi[1]->mode_flags |= MIPI_DSI_MODE_LPM;

	ret = mipi_dsi_dcs_exit_sleep_mode(dsi);
	if (ret < 0) {
		dev_err(dev, "Failed to exit sleep mode: %d\n", ret);
		return ret;
	}
	ret = mipi_dsi_dcs_exit_sleep_mode(ctx->dsi[1]);
	if (ret < 0) {
		dev_err(dev, "Failed to exit sleep mode: %d\n", ret);
		return ret;
	}
	usleep_range(15000, 16000);

	ret = mipi_dsi_compression_mode(dsi, true);
	if (ret < 0) {
		dev_err(dev, "Failed to set compression mode: %d\n", ret);
		return ret;
	}
	if (ctx->dsi[1]) {
		ret = mipi_dsi_compression_mode(ctx->dsi[1], true);
		if (ret < 0) {
			dev_err(dev, "failed to enable compression mode: %d\n",
				ret);
			return ret;
		}
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
	mipi_dsi_dcs_write_seq(dsi, 0x60, 0x04, 0xc0);
	mipi_dsi_dcs_write_seq(dsi, 0xf0, 0xa5, 0xa5);

	ret = mipi_dsi_dcs_set_column_address(dsi, 0, resX);
	if (ret < 0) {
		dev_err(dev, "Failed to set column address: %d\n", ret);
		return ret;
	}

	ret = mipi_dsi_dcs_set_page_address(dsi, 0, resY);
	if (ret < 0) {
		dev_err(dev, "Failed to set page address: %d\n", ret);
		return ret;
	}

	ret = mipi_dsi_dcs_set_tear_on(dsi, MIPI_DSI_DCS_TEAR_MODE_VBLANK);
	if (ret < 0) {
		dev_err(dev, "Failed to set tear on: %d\n", ret);
		return ret;
	}

	ret = mipi_dsi_dcs_set_tear_scanline(dsi, scanline);
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
	struct device *dev = &dsi->dev;
	int ret;

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
	struct drm_dsc_picture_parameter_set pps;
	struct mipi_dsi_device *dsi = ctx->dsi[0];
	struct device *dev = &dsi->dev;
	int ret;

	if (ctx->prepared)
		return 0;

	ret = regulator_bulk_enable(ARRAY_SIZE(ctx->supplies), ctx->supplies);
	if (ret < 0) {
		dev_err(dev, "Failed to enable regulators: %d\n", ret);
		return ret;
	}

	ana6707_amb650yl01_reset(ctx);

	msleep(120);

	ret = ana6707_amb650yl01_on(ctx);
	if (ret < 0) {
		dev_err(dev, "Failed to initialize panel: %d\n", ret);
		goto fail;
	}

	msleep(120);

	drm_dsc_pps_payload_pack(&pps, &ctx->dsc);

	ret = mipi_dsi_picture_parameter_set(dsi, &pps);
	if (ret < 0) {
		dev_err(panel->dev, "failed to transmit PPS: %d\n", ret);
		goto fail;
	}
	if (ctx->dsi[1]) {
		ret = mipi_dsi_picture_parameter_set(ctx->dsi[1], &pps);
		if (ret < 0) {
			dev_err(panel->dev, "failed to transmit PPS: %d\n",
				ret);
			goto fail;
		}
	}

	msleep(28);

	// ret = ana6707_amb650yl01_timing_switch(ctx);
	// if (ret < 0) {
	// 	dev_err(dev, "Failed to execute switch timing cmd: %d\n", ret);
	// 	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	// 	regulator_bulk_disable(ARRAY_SIZE(ctx->supplies), ctx->supplies);
	// 	return ret;
	// }

	return 0;

fail:
	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	regulator_bulk_disable(ARRAY_SIZE(ctx->supplies), ctx->supplies);
	return ret;
}

static int ana6707_amb650yl01_enable(struct drm_panel *panel)
{
	struct ana6707_amb650yl01 *ctx = to_ana6707_amb650yl01(panel);
	struct mipi_dsi_device *dsi = ctx->dsi[0];
	struct device *dev = &dsi->dev;
	int ret;

	ret = mipi_dsi_dcs_set_display_on(dsi);
	if (ret < 0) {
		dev_err(dev, "Failed to set display on: %d\n", ret);
		return ret;
	}
	ret = mipi_dsi_dcs_set_display_on(ctx->dsi[1]);
	if (ret < 0) {
		dev_err(dev, "Failed to set display on: %d\n", ret);
		return ret;
	}
	usleep_range(10000, 11000);

	return ret;
}

static int ana6707_amb650yl01_disable(struct drm_panel *panel)
{
	struct ana6707_amb650yl01 *ctx = to_ana6707_amb650yl01(panel);
	struct device *dev = &ctx->dsi[0]->dev;
	int ret;

	ret = ana6707_amb650yl01_off(ctx);
	if (ret < 0)
		dev_err(dev, "Failed to un-initialize panel: %d\n", ret);

	return ret;
}

static int ana6707_amb650yl01_unprepare(struct drm_panel *panel)
{
	struct ana6707_amb650yl01 *ctx = to_ana6707_amb650yl01(panel);
	struct device *dev = &ctx->dsi[0]->dev;
	int ret;

	ctx->dsi[0]->mode_flags &= ~MIPI_DSI_MODE_LPM;
	if (ctx->dsi[1])
		ctx->dsi[1]->mode_flags &= ~MIPI_DSI_MODE_LPM;

	if (!ctx->prepared)
		return 0;

	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	regulator_bulk_disable(ARRAY_SIZE(ctx->supplies), ctx->supplies);

	ctx->prepared = false;
	return 0;
}

/*
 * All of the horizontal values here are x2 of what's specified in the downstream device-tree,
 * as the information there is essentially divided by the number of DSI hosts
 */
/* HACK! the framerate is cut to accommodate for non-DSC bw limits */
static const struct drm_display_mode ana6707_amb650yl01_mode_4k = {
	.clock = (1644 + 16) * (3840 + 16) * 60 / 1000,
	.hdisplay = 1644,
	.hsync_start = 1644,
	.hsync_end = 1644,
	.htotal = 1644 + 16,
	.vdisplay = 3840,
	.vsync_start = 3840,
	.vsync_end = 3840,
	.vtotal = 3840 + 16,
	.width_mm = 65,
	.height_mm = 152,
};

static const int hdisplay = 1096; //  / 2;
static const int transfer_delay = 1000;

static const struct drm_display_mode ana6707_amb650yl01_mode = {
	.clock = (hdisplay + transfer_delay) * 2560 * 60 / 1000,
	.hdisplay = hdisplay,
	.hsync_start = (hdisplay + transfer_delay),
	.hsync_end = (hdisplay + transfer_delay),
	.htotal = (hdisplay + transfer_delay),
	.vdisplay = 2560,
	.vsync_start = 2560,
	.vsync_end = 2560,
	.vtotal = 2560,
	.width_mm = 65,
	.height_mm = 152,
};

static int ana6707_amb650yl01_get_modes(struct drm_panel *panel,
					struct drm_connector *connector)
{
	struct drm_display_mode *mode;
	if (enable_4k)
		return drm_connector_helper_get_modes_fixed(
			connector, &ana6707_amb650yl01_mode_4k);
	else
		return drm_connector_helper_get_modes_fixed(
			connector, &ana6707_amb650yl01_mode);
}

static const struct drm_panel_funcs ana6707_amb650yl01_panel_funcs = {
	.prepare = ana6707_amb650yl01_prepare,
	.enable = ana6707_amb650yl01_enable,
	.disable = ana6707_amb650yl01_disable,
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
	if (ctx->dsi[1])
		ctx->dsi[1]->mode_flags &= ~MIPI_DSI_MODE_LPM;

	ret = mipi_dsi_dcs_set_display_brightness_large(dsi, brightness);

	ctx->dsi[0]->mode_flags |= MIPI_DSI_MODE_LPM;
	if (ctx->dsi[1])
		ctx->dsi[1]->mode_flags |= MIPI_DSI_MODE_LPM;

	if (ret < 0)
		return ret;

	return 0;
}

static int ana6707_amb650yl01_bl_get_brightness(struct backlight_device *bl)
{
	struct mipi_dsi_device *dsi = bl_get_data(bl);
	struct ana6707_amb650yl01 *ctx = mipi_dsi_get_drvdata(dsi);
	u16 brightness;
	int ret;

	ctx->dsi[0]->mode_flags &= ~MIPI_DSI_MODE_LPM;
	if (ctx->dsi[1])
		ctx->dsi[1]->mode_flags &= ~MIPI_DSI_MODE_LPM;

	ret = mipi_dsi_dcs_get_display_brightness_large(dsi, &brightness);

	ctx->dsi[0]->mode_flags |= MIPI_DSI_MODE_LPM;
	if (ctx->dsi[1])
		ctx->dsi[1]->mode_flags |= MIPI_DSI_MODE_LPM;

	if (ret < 0)
		return ret;

	return brightness;
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
		.brightness = 100,
		.max_brightness = 4095,
	};

	return devm_backlight_device_register(dev, dev_name(dev), dev, dsi,
					      &ana6707_amb650yl01_bl_ops,
					      &props);
}

static int ana6707_amb650yl01_probe(struct mipi_dsi_device *dsi)
{
	const u16 hdisplay = enable_4k ? 1644 : 1096;
	struct mipi_dsi_host *dsi_sec_host;
	struct ana6707_amb650yl01 *ctx;
	struct device *dev = &dsi->dev;
	struct device_node *dsi_sec;
	int ret, i;

	ctx = devm_kzalloc(dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	/*
	 * Downstream order:
	 * vddio
	 * vddr
	 * vci
	 * tsio
	 * tsvddh
	 * no idea when ab/ibb.. maybe before since pmic probes way ahead
	 */
	ctx->supplies[0].supply = "ab";
	ctx->supplies[1].supply = "ibb";
	ctx->supplies[2].supply = "vddio";
	ctx->supplies[3].supply = "vddr";
	ctx->supplies[4].supply = "vci";
	ctx->supplies[5].supply = "tsio";
	ctx->supplies[6].supply = "tsvddh";
	ret = devm_regulator_bulk_get(dev, ARRAY_SIZE(ctx->supplies),
				      ctx->supplies);
	if (ret < 0)
		return dev_err_probe(dev, ret, "Failed to get regulators\n");

	ctx->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(ctx->reset_gpio),
				     "Failed to get reset-gpios\n");

	dsi_sec = of_graph_get_remote_node(dsi->dev.of_node, 1, -1);

	if (dsi_sec) {
		dev_notice(dev, "Using Dual-DSI\n");

		const struct mipi_dsi_device_info info = { "AMB650YL01", 0,
							   dsi_sec };

		dev_notice(dev, "Found second DSI `%s`\n", dsi_sec->name);

		dsi_sec_host = of_find_mipi_dsi_host_by_node(dsi_sec);
		of_node_put(dsi_sec);
		if (!dsi_sec_host) {
			return dev_err_probe(dev, -EPROBE_DEFER,
					     "Cannot get secondary DSI host\n");
		}

		ctx->dsi[1] =
			mipi_dsi_device_register_full(dsi_sec_host, &info);
		if (IS_ERR(ctx->dsi[1])) {
			return dev_err_probe(dev, PTR_ERR(ctx->dsi[1]),
					     "Cannot get secondary DSI node\n");
		}

		dev_notice(dev, "Second DSI name `%s`\n", ctx->dsi[1]->name);
		mipi_dsi_set_drvdata(ctx->dsi[1], ctx);
	} else {
		dev_notice(dev, "Using Single-DSI\n");
	}

	ctx->dsi[0] = dsi;
	mipi_dsi_set_drvdata(dsi, ctx);

	drm_panel_init(&ctx->panel, dev, &ana6707_amb650yl01_panel_funcs,
		       DRM_MODE_CONNECTOR_DSI);
	ctx->panel.prepare_prev_first = true;

	ctx->panel.backlight = ana6707_amb650yl01_create_backlight(dsi);
	if (IS_ERR(ctx->panel.backlight))
		return dev_err_probe(dev, PTR_ERR(ctx->panel.backlight),
				     "Failed to create backlight\n");

	drm_panel_add(&ctx->panel);

	ctx->dsc.dsc_version_major = 1;
	ctx->dsc.dsc_version_minor = 1; /* TODO: Could this be ver 2? */

	ctx->dsc.slice_height = 32;
	/* Downstream sets this while parsing DT */
	ctx->dsc.slice_count = 1;
	/*
	 * hdisplay should be read from the selected mode once
	 * it is passed back to drm_panel (in prepare?)
	 */
	// WARN_ON(1096 % ctx->dsc.slice_count);
	ctx->dsc.slice_width = hdisplay / 2; //1096 / ctx->dsc.slice_count;
	ctx->dsc.bits_per_component = 8;
	ctx->dsc.bits_per_pixel = 8 << 4; /* 4 fractional bits */
	ctx->dsc.block_pred_enable = true;


	for (i = 0; i < ARRAY_SIZE(ctx->dsi); i++) {
		if (!ctx->dsi[i])
			continue;

		/* This panel only supports DSC; unconditionally enable it */
		ctx->dsi[i]->dsc = &ctx->dsc;

		dev_notice(&ctx->dsi[i]->dev, "Binding DSI %d\n", i);

		// TODO: KP when calling mipi_dsi_attach below?
		// if (i > 0)
		// 	continue;

		ctx->dsi[i]->lanes = 4;
		ctx->dsi[i]->format = MIPI_DSI_FMT_RGB888;
		ctx->dsi[i]->mode_flags = MIPI_DSI_CLOCK_NON_CONTINUOUS;

		ret = mipi_dsi_attach(ctx->dsi[i]);
		if (ret < 0) {
			drm_panel_remove(&ctx->panel);
			return dev_err_probe(dev, ret,
					     "Failed to attach to DSI%d\n", i);
		}
	}

	dev_notice(dev, "FINALLY PROBED SUCCESSFULLY!\n");

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
	{ .compatible = "sony,ana6707-amb650yl01" },
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

MODULE_AUTHOR("Marijn Suijten <marijn.suijten@somainline.org>");
MODULE_DESCRIPTION("DRM panel driver for the Samsung ANA6707 Driver-IC");
MODULE_LICENSE("GPL");
