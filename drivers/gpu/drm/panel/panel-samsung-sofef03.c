// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2022 Konrad Dybcio <konrad.dybcio@linaro.org>
 * Copyright (c) 2023 Marijn Suijten <marijn.suijten@somainline.org>
 */

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

static const bool enable_120hz = true;

struct samsung_sofef03_m {
	struct drm_panel panel;
	struct mipi_dsi_device *dsi;
	struct drm_dsc_config dsc;
	struct regulator *vddio, *vci;
	struct gpio_desc *reset_gpio;
};

static inline struct samsung_sofef03_m *
to_samsung_sofef03_m(struct drm_panel *panel)
{
	return container_of(panel, struct samsung_sofef03_m, panel);
}

static void samsung_sofef03_m_reset(struct samsung_sofef03_m *ctx)
{
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	usleep_range(10000, 11000);
}

static int samsung_sofef03_m_on(struct samsung_sofef03_m *ctx)
{
	struct mipi_dsi_device *dsi = ctx->dsi;
	struct device *dev = &dsi->dev;
	int ret;

	dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	mipi_dsi_dcs_write_seq(dsi, 0x9d, 0x01);

	ret = mipi_dsi_dcs_exit_sleep_mode(dsi);
	if (ret < 0) {
		dev_err(dev, "Failed to exit sleep mode: %d\n", ret);
		return ret;
	}
	usleep_range(10000, 11000);

	mipi_dsi_dcs_write_seq(dsi, 0xf0, 0x5a, 0x5a);
	mipi_dsi_dcs_write_seq(dsi, 0xb0, 0x09);
	mipi_dsi_dcs_write_seq(dsi, 0xd5, 0x00, 0x00, 0x00);
	mipi_dsi_dcs_write_seq(dsi, 0xb0, 0x08);
	mipi_dsi_dcs_write_seq(dsi, 0xee, 0x00, 0x00);
	mipi_dsi_dcs_write_seq(dsi, 0xf0, 0xa5, 0xa5);

	ret = mipi_dsi_dcs_set_tear_on(dsi, MIPI_DSI_DCS_TEAR_MODE_VBLANK);
	if (ret < 0) {
		dev_err(dev, "Failed to set tear on: %d\n", ret);
		return ret;
	}

	ret = mipi_dsi_dcs_set_column_address(dsi, 0, 1080 - 1);
	if (ret < 0) {
		dev_err(dev, "Failed to set column address: %d\n", ret);
		return ret;
	}

	ret = mipi_dsi_dcs_set_page_address(dsi, 0, 2520 - 1);
	if (ret < 0) {
		dev_err(dev, "Failed to set page address: %d\n", ret);
		return ret;
	}

	ret = mipi_dsi_dcs_set_display_brightness_large(dsi, 100);
	if (ret < 0) {
		dev_err(dev, "Failed to set display brightness: %d\n", ret);
		return ret;
	}

	mipi_dsi_dcs_write_seq(dsi, 0xf0, 0x5a, 0x5a);
	mipi_dsi_dcs_write_seq(dsi, 0xdf, 0x83);
	mipi_dsi_dcs_write_seq(dsi, 0xb0, 0x01);
	mipi_dsi_dcs_write_seq(dsi, 0xe6, 0x01);
	mipi_dsi_dcs_write_seq(dsi, 0xb0, 0x02);
	mipi_dsi_dcs_write_seq(dsi, 0xec, 0x02, 0x00, 0x1c, 0x1c);
	mipi_dsi_dcs_write_seq(dsi, 0xb0, 0x0c);
	mipi_dsi_dcs_write_seq(dsi, 0xec, 0x01, 0x19);
	mipi_dsi_dcs_write_seq(dsi, 0xf0, 0xa5, 0xa5);
	mipi_dsi_dcs_write_seq(dsi, MIPI_DCS_WRITE_CONTROL_DISPLAY,
			       WRITE_CONTROL_DISPLAY_BACKLIGHT);
	mipi_dsi_dcs_write_seq(dsi, 0xf0, 0x5a, 0x5a);
	mipi_dsi_dcs_write_seq(dsi, 0xc2, 0x2d, 0x27);
	mipi_dsi_dcs_write_seq(dsi, 0x60, enable_120hz ? 0x10 : 0x00);
	mipi_dsi_dcs_write_seq(dsi, 0xf0, 0xa5, 0xa5);
	msleep(110);

	return 0;
}

static int samsung_sofef03_m_prepare(struct drm_panel *panel)
{
	struct samsung_sofef03_m *ctx = to_samsung_sofef03_m(panel);
	struct drm_dsc_picture_parameter_set pps;
	struct device *dev = &ctx->dsi->dev;
	int ret;

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

	samsung_sofef03_m_reset(ctx);

	ret = samsung_sofef03_m_on(ctx);
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

	ret = mipi_dsi_compression_mode(ctx->dsi, true);
	if (ret < 0) {
		dev_err(dev, "Failed to enable compression mode: %d\n", ret);
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

static int samsung_sofef03_m_enable(struct drm_panel *panel)
{
	struct samsung_sofef03_m *ctx = to_samsung_sofef03_m(panel);
	struct mipi_dsi_device *dsi = ctx->dsi;
	struct device *dev = &dsi->dev;
	int ret;

	ret = mipi_dsi_dcs_set_display_on(dsi);
	if (ret < 0) {
		dev_err(dev, "Failed to turn display on: %d\n", ret);
		return ret;
	}

	return 0;
}

static int samsung_sofef03_m_disable(struct drm_panel *panel)
{
	struct samsung_sofef03_m *ctx = to_samsung_sofef03_m(panel);
	struct mipi_dsi_device *dsi = ctx->dsi;
	struct device *dev = &dsi->dev;
	int ret;

	dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;

	ret = mipi_dsi_dcs_set_display_off(dsi);
	if (ret < 0) {
		dev_err(dev, "Failed to turn display off: %d\n", ret);
		return ret;
	}
	msleep(20);

	ret = mipi_dsi_dcs_enter_sleep_mode(dsi);
	if (ret < 0) {
		dev_err(dev, "Failed to enter sleep mode: %d\n", ret);
		return ret;
	}
	msleep(100);

	return 0;
}

static int samsung_sofef03_m_unprepare(struct drm_panel *panel)
{
	struct samsung_sofef03_m *ctx = to_samsung_sofef03_m(panel);

	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	regulator_disable(ctx->vci);
	regulator_disable(ctx->vddio);

	return 0;
}

static const struct drm_display_mode samsung_sofef03_m_60hz_mode = {
	.clock = (1080 + 156 + 8 + 8) * (2520 + 2393 + 8 + 8) * 60 / 1000,
	.hdisplay = 1080,
	.hsync_start = 1080 + 156,
	.hsync_end = 1080 + 156 + 8,
	.htotal = 1080 + 156 + 8 + 8,
	.vdisplay = 2520,
	.vsync_start = 2520 + 2393,
	.vsync_end = 2520 + 2393 + 8,
	.vtotal = 2520 + 2393 + 8 + 8,
	.width_mm = 61,
	.height_mm = 142,
};

static const struct drm_display_mode samsung_sofef03_m_120hz_mode = {
	.clock = (1080 + 56 + 8 + 8) * (2520 + 499 + 8 + 8) * 120 / 1000,
	.hdisplay = 1080,
	.hsync_start = 1080 + 56,
	.hsync_end = 1080 + 56 + 8,
	.htotal = 1080 + 56 + 8 + 8,
	.vdisplay = 2520,
	.vsync_start = 2520 + 499,
	.vsync_end = 2520 + 499 + 8,
	.vtotal = 2520 + 499 + 8 + 8,
	.width_mm = 61,
	.height_mm = 142,
};

static int samsung_sofef03_m_get_modes(struct drm_panel *panel,
				       struct drm_connector *connector)
{
	if (enable_120hz)
		return drm_connector_helper_get_modes_fixed(
			connector, &samsung_sofef03_m_120hz_mode);
	else
		return drm_connector_helper_get_modes_fixed(
			connector, &samsung_sofef03_m_60hz_mode);
}

static const struct drm_panel_funcs samsung_sofef03_m_panel_funcs = {
	.prepare = samsung_sofef03_m_prepare,
	.enable = samsung_sofef03_m_enable,
	.disable = samsung_sofef03_m_disable,
	.unprepare = samsung_sofef03_m_unprepare,
	.get_modes = samsung_sofef03_m_get_modes,
};

static int samsung_sofef03_m_bl_update_status(struct backlight_device *bl)
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

static int samsung_sofef03_m_bl_get_brightness(struct backlight_device *bl)
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

static const struct backlight_ops samsung_sofef03_m_bl_ops = {
	.update_status = samsung_sofef03_m_bl_update_status,
	.get_brightness = samsung_sofef03_m_bl_get_brightness,
};

static struct backlight_device *
samsung_sofef03_m_create_backlight(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	const struct backlight_properties props = {
		.type = BACKLIGHT_RAW,
		.brightness = 100,
		.max_brightness = 1023,
	};

	return devm_backlight_device_register(dev, dev_name(dev), dev, dsi,
					      &samsung_sofef03_m_bl_ops,
					      &props);
}

static int samsung_sofef03_m_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct samsung_sofef03_m *ctx;
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

	drm_panel_init(&ctx->panel, dev, &samsung_sofef03_m_panel_funcs,
		       DRM_MODE_CONNECTOR_DSI);
	ctx->panel.prepare_prev_first = true;

	ctx->panel.backlight = samsung_sofef03_m_create_backlight(dsi);
	if (IS_ERR(ctx->panel.backlight))
		return dev_err_probe(dev, PTR_ERR(ctx->panel.backlight),
				     "Failed to create backlight\n");

	drm_panel_add(&ctx->panel);

	/* This panel only supports DSC; unconditionally enable it */
	dsi->dsc = &ctx->dsc;

	ctx->dsc.dsc_version_major = 1;
	ctx->dsc.dsc_version_minor = 1;

	ctx->dsc.slice_height = 30;
	ctx->dsc.slice_width = 540;
	ctx->dsc.slice_count = 2;
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

static void samsung_sofef03_m_remove(struct mipi_dsi_device *dsi)
{
	struct samsung_sofef03_m *ctx = mipi_dsi_get_drvdata(dsi);
	int ret;

	ret = mipi_dsi_detach(dsi);
	if (ret < 0)
		dev_err(&dsi->dev, "Failed to detach from DSI host: %d\n", ret);

	drm_panel_remove(&ctx->panel);
}

static const struct of_device_id samsung_sofef03_m_of_match[] = {
	/* Sony Xperia 5 II, 5 III */
	{ .compatible = "samsung,sofef03-m-amb609vp01" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, samsung_sofef03_m_of_match);

static struct mipi_dsi_driver samsung_sofef03_m_driver = {
	.probe = samsung_sofef03_m_probe,
	.remove = samsung_sofef03_m_remove,
	.driver = {
		.name = "panel-samsung-sofef03-m",
		.of_match_table = samsung_sofef03_m_of_match,
	},
};
module_mipi_dsi_driver(samsung_sofef03_m_driver);

MODULE_AUTHOR("Konrad Dybcio <konrad.dybcio@linaro.org>");
MODULE_AUTHOR("Marijn Suijten <marijn.suijten@somainline.org>");
MODULE_DESCRIPTION("DRM panel driver for Samsung SOFEF03-M Driver-IC panels");
MODULE_LICENSE("GPL");
