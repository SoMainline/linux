// SPDX-License-Identifier: GPL-2.0-only
// Copyright (c) 2023 Marijn Suijten <marijn.suijten@somainline.org>

#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regulator/consumer.h>

#include <video/mipi_display.h>

#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>
#include <drm/drm_probe_helper.h>
#include <drm/display/drm_dsc.h>
#include <drm/display/drm_dsc_helper.h>

#define WRITE_CONTROL_DISPLAY_BACKLIGHT BIT(5)

static const bool enable_4k = true;

struct samsung_souxp00 {
	struct drm_panel panel;
	struct mipi_dsi_device *dsi;
	struct drm_dsc_config dsc;
	struct gpio_desc *reset_gpio;
	struct regulator *vddio, *vci;
};

static inline struct samsung_souxp00 *
to_samsung_souxp00(struct drm_panel *panel)
{
	return container_of(panel, struct samsung_souxp00, panel);
}

static int samsung_souxp00_on(struct samsung_souxp00 *ctx)
{
	const u16 hdisplay = enable_4k ? 1644 : 1096;
	const u16 vdisplay = enable_4k ? 3840 : 2560;
	struct mipi_dsi_device *dsi = ctx->dsi;
	struct device *dev = &dsi->dev;
	int ret;

	dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	ret = mipi_dsi_dcs_exit_sleep_mode(dsi);
	if (ret < 0) {
		dev_err(dev, "Failed to exit sleep mode: %d\n", ret);
		return ret;
	}
	usleep_range(10000, 11000);

	// if (panel == 07) {
	// 	mipi_dsi_dcs_write_seq(dsi, 0xf0, 0x5a, 0x5a);
	// 	mipi_dsi_dcs_write_seq(dsi, 0xdd, 0x00, 0x80, 0x00, 0x00);
	// 	mipi_dsi_dcs_write_seq(dsi, 0xf0, 0xa5, 0xa5);
	// }

	ret = mipi_dsi_dcs_set_tear_on(dsi, MIPI_DSI_DCS_TEAR_MODE_VBLANK);
	if (ret < 0) {
		dev_err(dev, "Failed to set tear on: %d\n", ret);
		return ret;
	}

	ret = mipi_dsi_compression_mode(dsi, true);
	if (ret < 0) {
		dev_err(dev, "Failed to set compression mode: %d\n", ret);
		return ret;
	}

	mipi_dsi_dcs_write_seq(dsi, 0xf0, 0x5a, 0x5a);
	mipi_dsi_dcs_write_seq(dsi, 0xb0, 0x05);
	mipi_dsi_dcs_write_seq(dsi, 0xd7, 0x07);
	mipi_dsi_dcs_write_seq(dsi, 0xf0, 0xa5, 0xa5);
	mipi_dsi_dcs_write_seq(dsi, 0xf0, 0x5a, 0x5a);
	mipi_dsi_dcs_write_seq(dsi, 0xe2, enable_4k ? 0 : 1);
	mipi_dsi_dcs_write_seq(dsi, 0xf0, 0xa5, 0xa5);

	ret = mipi_dsi_dcs_set_column_address(dsi, 0, hdisplay - 1);
	if (ret < 0) {
		dev_err(dev, "Failed to set column address: %d\n", ret);
		return ret;
	}

	ret = mipi_dsi_dcs_set_page_address(dsi, 0, vdisplay - 1);
	if (ret < 0) {
		dev_err(dev, "Failed to set page address: %d\n", ret);
		return ret;
	}

	// if (panel == 01) {
	// 	mipi_dsi_dcs_write_seq(dsi, 0xf0, 0x5a, 0x5a);
	// 	mipi_dsi_dcs_write_seq(dsi, 0xb0, 0x70);
	// 	mipi_dsi_dcs_write_seq(dsi, 0xb9, 0x00, 0x60);
	// 	mipi_dsi_dcs_write_seq(dsi, 0xf0, 0xa5, 0xa5);
	// }

	/* Enable backlight control */
	mipi_dsi_dcs_write_seq(dsi, MIPI_DCS_WRITE_CONTROL_DISPLAY,
			       WRITE_CONTROL_DISPLAY_BACKLIGHT);
	mipi_dsi_dcs_write_seq(dsi, 0xf0, 0x5a, 0x5a);
	mipi_dsi_dcs_write_seq(dsi, 0xc5, 0x2e, 0x21);
	mipi_dsi_dcs_write_seq(dsi, 0xf0, 0xa5, 0xa5);
	msleep(110);

	return 0;
}

static int samsung_souxp00_prepare(struct drm_panel *panel)
{
	struct samsung_souxp00 *ctx = to_samsung_souxp00(panel);
	struct drm_dsc_picture_parameter_set pps;
	struct device *dev = &ctx->dsi->dev;
	int ret;

	dev_err(dev, "%s\n", __func__);

	ret = regulator_enable(ctx->vddio);
	if (ret < 0) {
		dev_err(dev, "Failed to enable vddio regulator: %d\n", ret);
		return ret;
	}

	ret = regulator_enable(ctx->vci);
	if (ret < 0) {
		dev_err(dev, "Failed to enable vci regulator: %d\n", ret);
		regulator_disable(ctx->vddio);
		return ret;
	}

	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	usleep_range(10000, 11000);

	ret = samsung_souxp00_on(ctx);
	if (ret < 0) {
		dev_err(dev, "Failed to send on command sequence: %d\n", ret);
		goto fail;
	}

	drm_dsc_pps_payload_pack(&pps, &ctx->dsc);

	ret = mipi_dsi_picture_parameter_set(ctx->dsi, &pps);
	if (ret < 0) {
		dev_err(dev, "failed to transmit PPS: %d\n", ret);
		goto fail;
	}

	msleep(28);

	return 0;

fail:
	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	regulator_disable(ctx->vci);
	regulator_disable(ctx->vddio);
	return ret;
}

static int samsung_souxp00_enable(struct drm_panel *panel)
{
	struct samsung_souxp00 *ctx = to_samsung_souxp00(panel);
	struct mipi_dsi_device *dsi = ctx->dsi;
	struct device *dev = &dsi->dev;
	int ret;

	dev_err(dev, "%s\n", __func__);

	ret = mipi_dsi_dcs_set_display_on(dsi);
	if (ret < 0) {
		dev_err(dev, "Failed to turn display on: %d\n", ret);
		return ret;
	}

	return 0;
}

static int samsung_souxp00_disable(struct drm_panel *panel)
{
	struct samsung_souxp00 *ctx = to_samsung_souxp00(panel);
	struct mipi_dsi_device *dsi = ctx->dsi;
	struct device *dev = &dsi->dev;
	int ret;

	dev_err(dev, "%s\n", __func__);

	dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;

	ret = mipi_dsi_dcs_set_display_off(dsi);
	if (ret < 0) {
		dev_err(dev, "Failed to turn display off: %d\n", ret);
		return ret;
	}
	msleep(20);

	mipi_dsi_dcs_write_seq(dsi, MIPI_DCS_WRITE_CONTROL_DISPLAY,
			       WRITE_CONTROL_DISPLAY_BACKLIGHT);
	usleep_range(17000, 18000);

	ret = mipi_dsi_dcs_enter_sleep_mode(dsi);
	if (ret < 0) {
		dev_err(dev, "Failed to enter sleep mode: %d\n", ret);
		return ret;
	}
	msleep(100);

	return 0;
}

static int samsung_souxp00_unprepare(struct drm_panel *panel)
{
	struct samsung_souxp00 *ctx = to_samsung_souxp00(panel);
	struct device *dev = &ctx->dsi->dev;

	dev_err(dev, "%s\n", __func__);

	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	regulator_disable(ctx->vddio);
	regulator_disable(ctx->vci);

	return 0;
}

/*
 * Small fake porch to force the DSI pclk/byteclk
 * high enough to have a smooth panel at 60Hz.
 *
 * 60 for 4k, or 40 for 2.5k is fine for Griffin, but not PDX203.
 * Even at 84 there's still the occasional hiccup.
*/
static const int fake_porch = 98;

static const struct drm_display_mode samsung_souxp00_2_5k_mode = {
	.clock = (1096 + fake_porch) * 2560 * 60 / 1000,
	.hdisplay = 1096,
	.hsync_start = 1096 + fake_porch,
	.hsync_end = 1096 + fake_porch,
	.htotal = 1096 + fake_porch,
	.vdisplay = 2560,
	.vsync_start = 2560,
	.vsync_end = 2560,
	.vtotal = 2560,
	.width_mm = 65,
	.height_mm = 152,
};

static const struct drm_display_mode samsung_souxp00_4k_mode = {
	.clock = (1644 + fake_porch) * 3840 * 60 / 1000,
	.hdisplay = 1644,
	.hsync_start = 1644 + fake_porch,
	.hsync_end = 1644 + fake_porch,
	.htotal = 1644 + fake_porch,
	.vdisplay = 3840,
	.vsync_start = 3840,
	.vsync_end = 3840,
	.vtotal = 3840,
	.width_mm = 65,
	.height_mm = 152,
};

static int samsung_souxp00_get_modes(struct drm_panel *panel,
				     struct drm_connector *connector)
{
	if (enable_4k)
		return drm_connector_helper_get_modes_fixed(
			connector, &samsung_souxp00_4k_mode);
	else
		return drm_connector_helper_get_modes_fixed(
			connector, &samsung_souxp00_2_5k_mode);
}

static const struct drm_panel_funcs samsung_souxp00_panel_funcs = {
	.prepare = samsung_souxp00_prepare,
	.enable = samsung_souxp00_enable,
	.disable = samsung_souxp00_disable,
	.unprepare = samsung_souxp00_unprepare,
	.get_modes = samsung_souxp00_get_modes,
};

static int samsung_souxp00_bl_update_status(struct backlight_device *bl)
{
	struct mipi_dsi_device *dsi = bl_get_data(bl);
	u16 brightness = backlight_get_brightness(bl);
	int ret;

	dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;

	ret = mipi_dsi_dcs_set_display_brightness_large(dsi, brightness);

	dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	return ret;
}

static int samsung_souxp00_bl_get_brightness(struct backlight_device *bl)
{
	struct mipi_dsi_device *dsi = bl_get_data(bl);
	u16 brightness;
	int ret;

	dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;

	ret = mipi_dsi_dcs_get_display_brightness_large(dsi, &brightness);

	dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	if (ret < 0)
		return ret;

	return brightness;
}

static const struct backlight_ops samsung_souxp00_bl_ops = {
	.update_status = samsung_souxp00_bl_update_status,
	.get_brightness = samsung_souxp00_bl_get_brightness,
};

static struct backlight_device *
samsung_souxp00_create_backlight(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	const struct backlight_properties props = {
		.type = BACKLIGHT_RAW,
		.brightness = 400,
		.max_brightness = 4095,
	};

	return devm_backlight_device_register(dev, dev_name(dev), dev, dsi,
					      &samsung_souxp00_bl_ops, &props);
}

static int samsung_souxp00_probe(struct mipi_dsi_device *dsi)
{
	const u16 hdisplay = enable_4k ? 1644 : 1096;
	struct device *dev = &dsi->dev;
	struct samsung_souxp00 *ctx;
	int ret;

	ctx = devm_kzalloc(dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	ctx->vddio = devm_regulator_get(dev, "vddio");
	if (IS_ERR(ctx->vddio))
		return dev_err_probe(dev, PTR_ERR(ctx->vddio),
				     "Failed to get vddio regulator\n");

	ctx->vci = devm_regulator_get(dev, "vci");
	if (IS_ERR(ctx->vci))
		return dev_err_probe(dev, PTR_ERR(ctx->vci),
				     "Failed to get vci regulator\n");

	ctx->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(ctx->reset_gpio),
				     "Failed to get reset-gpios\n");

	ctx->dsi = dsi;
	mipi_dsi_set_drvdata(dsi, ctx);

	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_CLOCK_NON_CONTINUOUS;

	drm_panel_init(&ctx->panel, dev, &samsung_souxp00_panel_funcs,
		       DRM_MODE_CONNECTOR_DSI);
	ctx->panel.prepare_prev_first = true;

	ctx->panel.backlight = samsung_souxp00_create_backlight(dsi);
	if (IS_ERR(ctx->panel.backlight))
		return dev_err_probe(dev, PTR_ERR(ctx->panel.backlight),
				     "Failed to create backlight\n");

	drm_panel_add(&ctx->panel);

	/* This panel only supports DSC; unconditionally enable it */
	dsi->dsc = &ctx->dsc;

	ctx->dsc.dsc_version_major = 1;
	ctx->dsc.dsc_version_minor = 1;

	ctx->dsc.slice_height = 32;
	ctx->dsc.slice_count = 2;
	/*
	 * hdisplay should be read from the selected mode once
	 * it is passed back to drm_panel (in prepare?)
	 */
	WARN_ON(hdisplay % ctx->dsc.slice_count);
	ctx->dsc.slice_width = hdisplay / ctx->dsc.slice_count;
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

static void samsung_souxp00_remove(struct mipi_dsi_device *dsi)
{
	struct samsung_souxp00 *ctx = mipi_dsi_get_drvdata(dsi);
	int ret;

	ret = mipi_dsi_detach(dsi);
	if (ret < 0)
		dev_err(&dsi->dev, "Failed to detach from DSI host: %d\n", ret);

	drm_panel_remove(&ctx->panel);
}

static const struct of_device_id samsung_souxp00_of_match[] = {
	/* Sony Xperia 1 */
	{ .compatible = "samsung,souxp00_a-amb650wh01" },
	/* Sony Xperia 1 II */
	{ .compatible = "samsung,souxp00_a-amb650wh07" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, samsung_souxp00_of_match);

static struct mipi_dsi_driver samsung_souxp00_driver = {
	.probe = samsung_souxp00_probe,
	.remove = samsung_souxp00_remove,
	.driver = {
		.name = "panel-samsung-souxp",
		.of_match_table = samsung_souxp00_of_match,
	},
};
module_mipi_dsi_driver(samsung_souxp00_driver);

MODULE_AUTHOR("Marijn Suijten <marijn.suijten@somainline.org>");
MODULE_DESCRIPTION("DRM panel driver for the Samsung SOUXP00 Driver-IC");
MODULE_LICENSE("GPL");
