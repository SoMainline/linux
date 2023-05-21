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

#define WRITE_CONTROL_DISPLAY_AOD_LOW BIT(0)
#define WRITE_CONTROL_DISPLAY_AOD_ON BIT(1)
#define WRITE_CONTROL_DISPLAY_DIMMING BIT(3)
#define WRITE_CONTROL_DISPLAY_LOCAL_HBM BIT(4)
#define WRITE_CONTROL_DISPLAY_BACKLIGHT BIT(5)
#define WRITE_CONTROL_DISPLAY_HBM GENMASK(6, 7)

struct samsung_sofef01_m {
	struct drm_panel panel;
	struct mipi_dsi_device *dsi;
	struct regulator *vddio, *vci;
	struct gpio_desc *reset_gpio;
	const struct drm_display_mode *mode;
};

static inline struct samsung_sofef01_m *
to_samsung_sofef01_m(struct drm_panel *panel)
{
	return container_of(panel, struct samsung_sofef01_m, panel);
}

static int samsung_sofef01_m_on(struct samsung_sofef01_m *ctx)
{
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

	ret = mipi_dsi_dcs_set_tear_on(dsi, MIPI_DSI_DCS_TEAR_MODE_VBLANK);
	if (ret < 0) {
		dev_err(dev, "Failed to set tear on: %d\n", ret);
		return ret;
	}

	mipi_dsi_dcs_write_seq(dsi, 0xf0, 0x5a, 0x5a);
	mipi_dsi_dcs_write_seq(dsi, 0xdf, 0x03);
	mipi_dsi_dcs_write_seq(dsi, 0xe0, 0x01);
	mipi_dsi_dcs_write_seq(dsi, 0xf0, 0xa5, 0xa5);

	// PDX213:
	mipi_dsi_dcs_write_seq(dsi, 0xf0, 0x5a, 0x5a);
	mipi_dsi_dcs_write_seq(dsi, 0xfc, 0x5a, 0x5a);
	mipi_dsi_dcs_write_seq(dsi, 0xE1, 0x00, 0x00, 0x02, 0x00, 0x1C, 0x1C,
			       0x00, 0x00, 0x20, 0x00, 0x00, 0x01, 0x19);
	mipi_dsi_dcs_write_seq(dsi, 0xfc, 0xa5, 0xa5);
	mipi_dsi_dcs_write_seq(dsi, 0xf0, 0xa5, 0xa5);

	// PDX225 stock:
	// https://github.com/sonyxperiadev/kernel-copyleft-dts/blob/65.1.A.4.xxx/qcom/dsi-panel-samsung-amoled-fhd-cmd.dtsi#L61
	// mipi_dsi_dcs_write_seq(dsi, 0xf0, 0x5a, 0x5a);
	// mipi_dsi_dcs_write_seq(dsi, 0xb0, 0x27, 0xf2);
	// mipi_dsi_dcs_write_seq(dsi, 0xb0, 0xf2, 0x80);
	// mipi_dsi_dcs_write_seq(dsi, 0xb0, 0xf7, 0x07);
	// mipi_dsi_dcs_write_seq(dsi, 0xf0, 0xa5, 0xa5);

	// mipi_dsi_dcs_write_seq(dsi, 0xf0, 0x5a, 0x5a);
	// mipi_dsi_dcs_write_seq(dsi, 0xb0, 0x02, 0x8f);
	// mipi_dsi_dcs_write_seq(dsi, 0x8f, 0x27, 0x05);
	// mipi_dsi_dcs_write_seq(dsi, 0xf0, 0xa5, 0xa5);

	// mipi_dsi_dcs_write_seq(dsi, 0xf0, 0x5a, 0x5a);
	// mipi_dsi_dcs_write_seq(dsi, 0xb0, 0x92, 0x63);
	// mipi_dsi_dcs_write_seq(dsi, 0x63, 0x05);
	// mipi_dsi_dcs_write_seq(dsi, 0xf0, 0xa5, 0xa5);

	// TODO: This wasn't previously called but makes sense to be called now
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

	// PDX213:
	mipi_dsi_dcs_set_display_brightness_large(dsi, 0x119);

	mipi_dsi_dcs_write_seq(dsi, MIPI_DCS_WRITE_CONTROL_DISPLAY,
			       WRITE_CONTROL_DISPLAY_BACKLIGHT);
	mipi_dsi_dcs_write_seq(dsi, MIPI_DCS_WRITE_POWER_SAVE, 0x00);
	mipi_dsi_dcs_write_seq(dsi, 0xf0, 0x5a, 0x5a);
	// mipi_dsi_dcs_write_seq(dsi, 0xbe, 0x92, 0x29);
	// PDX213:
	mipi_dsi_dcs_write_seq(dsi, 0xbe, 0x92, 0x09);
	// mipi_dsi_dcs_write_seq(dsi, 0xf0, 0xa5, 0xa5);
	// mipi_dsi_dcs_write_seq(dsi, 0xf0, 0x5a, 0x5a);
	mipi_dsi_dcs_write_seq(dsi, 0xb0, 0x06);
	mipi_dsi_dcs_write_seq(dsi, 0xb6, 0x90);
	mipi_dsi_dcs_write_seq(dsi, 0xf0, 0xa5, 0xa5);
	msleep(110);

	return 0;
}

static int samsung_sofef01_m_prepare(struct drm_panel *panel)
{
	struct samsung_sofef01_m *ctx = to_samsung_sofef01_m(panel);
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

	ret = samsung_sofef01_m_on(ctx);
	if (ret < 0) {
		dev_err(dev, "Failed to send on command sequence: %d\n", ret);
		gpiod_set_value_cansleep(ctx->reset_gpio, 1);
		regulator_disable(ctx->vci);
		regulator_disable(ctx->vddio);
		return ret;
	}

	return 0;
}

static int samsung_sofef01_m_enable(struct drm_panel *panel)
{
	struct samsung_sofef01_m *ctx = to_samsung_sofef01_m(panel);
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

static int samsung_sofef01_m_disable(struct drm_panel *panel)
{
	struct samsung_sofef01_m *ctx = to_samsung_sofef01_m(panel);
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

	ret = mipi_dsi_dcs_enter_sleep_mode(dsi);
	if (ret < 0) {
		dev_err(dev, "Failed to enter sleep mode: %d\n", ret);
		return ret;
	}
	msleep(120);

	return 0;
}

static int samsung_sofef01_m_unprepare(struct drm_panel *panel)
{
	struct samsung_sofef01_m *ctx = to_samsung_sofef01_m(panel);

	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	regulator_disable(ctx->vci);
	regulator_disable(ctx->vddio);

	return 0;
}

static int samsung_sofef01_m_get_modes(struct drm_panel *panel,
				       struct drm_connector *connector)
{
	struct samsung_sofef01_m *ctx = to_samsung_sofef01_m(panel);

	return drm_connector_helper_get_modes_fixed(connector, ctx->mode);
}

static const struct drm_panel_funcs samsung_sofef01_m_panel_funcs = {
	.prepare = samsung_sofef01_m_prepare,
	.enable = samsung_sofef01_m_enable,
	.disable = samsung_sofef01_m_disable,
	.unprepare = samsung_sofef01_m_unprepare,
	.get_modes = samsung_sofef01_m_get_modes,
};

static int samsung_sofef01_m_bl_update_status(struct backlight_device *bl)
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

static int samsung_sofef01_m_bl_get_brightness(struct backlight_device *bl)
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

static const struct backlight_ops samsung_sofef01_m_bl_ops = {
	.update_status = samsung_sofef01_m_bl_update_status,
	.get_brightness = samsung_sofef01_m_bl_get_brightness,
};

static struct backlight_device *
samsung_sofef01_m_create_backlight(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	const struct backlight_properties props = {
		.type = BACKLIGHT_RAW,
		.brightness = 100,
		.max_brightness = 1023,
	};

	return devm_backlight_device_register(dev, dev_name(dev), dev, dsi,
					      &samsung_sofef01_m_bl_ops,
					      &props);
}

static int samsung_sofef01_m_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct samsung_sofef01_m *ctx;
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
	ctx->mode = of_device_get_match_data(dev);
	mipi_dsi_set_drvdata(dsi, ctx);

	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_CLOCK_NON_CONTINUOUS;

	ctx->panel.prepare_prev_first = true;

	drm_panel_init(&ctx->panel, dev, &samsung_sofef01_m_panel_funcs,
		       DRM_MODE_CONNECTOR_DSI);

	ctx->panel.backlight = samsung_sofef01_m_create_backlight(dsi);
	if (IS_ERR(ctx->panel.backlight))
		return dev_err_probe(dev, PTR_ERR(ctx->panel.backlight),
				     "Failed to create backlight\n");

	drm_panel_add(&ctx->panel);

	ret = mipi_dsi_attach(dsi);
	if (ret < 0) {
		dev_err(dev, "Failed to attach to DSI host: %d\n", ret);
		drm_panel_remove(&ctx->panel);
		return ret;
	}

	return 0;
}

static void samsung_sofef01_m_remove(struct mipi_dsi_device *dsi)
{
	struct samsung_sofef01_m *ctx = mipi_dsi_get_drvdata(dsi);
	int ret;

	ret = mipi_dsi_detach(dsi);
	if (ret < 0)
		dev_err(&dsi->dev, "Failed to detach from DSI host: %d\n", ret);

	drm_panel_remove(&ctx->panel);
}

/*
 * drm/msm's DSI code does not calculate transfer time but instead relies on
 * fake porch values (which are not a thing in CMD mode) to represent the
 * transfer time.
 *
 * Use the following expressions based on qcom,mdss-dsi-panel-clockrate from
 * downstream DT to artificially bump the mode's clock
 */
static const unsigned dsi_lanes = 4;
static const unsigned bpp = 24;
/* qcom,mdss-dsi-panel-clockrate from downstream DT */
static const unsigned long bitclk_hz = 1132293600;
static const unsigned long stable_clockrate = bitclk_hz * dsi_lanes / bpp;
static const unsigned long fake_porch = stable_clockrate / (2520 * 60) - 1080;

/* 61x142mm variant, Sony Xperia 5 */
static const struct drm_display_mode samsung_sofef01_m_61_142_mode = {
	.clock = (1080 + fake_porch) * 2520 * 60 / 1000,
	.hdisplay = 1080,
	.hsync_start = 1080 + fake_porch,
	.hsync_end = 1080 + fake_porch,
	.htotal = 1080 + fake_porch,
	.vdisplay = 2520,
	.vsync_start = 2520,
	.vsync_end = 2520,
	.vtotal = 2520,
	.width_mm = 61,
	.height_mm = 142,
};

/* 60x139mm variant, Sony Xperia 10 II, 10 III and 10 IV */
static const struct drm_display_mode samsung_sofef01_m_60_139_mode = {
	.clock = (1080 + fake_porch) * 2520 * 60 / 1000,
	.hdisplay = 1080,
	.hsync_start = 1080 + fake_porch,
	.hsync_end = 1080 + fake_porch,
	.htotal = 1080 + fake_porch,
	.vdisplay = 2520,
	.vsync_start = 2520,
	.vsync_end = 2520,
	.vtotal = 2520,
	.width_mm = 60,
	.height_mm = 139,
};

static const struct of_device_id samsung_sofef01_m_of_match[] = {
	/* Sony Xperia 5 */
	{ .compatible = "samsung,sofef01-m-amb609tc01",
	  .data = &samsung_sofef01_m_61_142_mode },
	/* Sony Xperia 10 II */
	{ .compatible = "samsung,sofef01-m-ams597ut01",
	  .data = &samsung_sofef01_m_60_139_mode },
	/* Sony Xperia 10 III */
	{ .compatible = "samsung,sofef01-m-ams597ut04",
	  .data = &samsung_sofef01_m_60_139_mode },
	/* Sony Xperia 10 IV */
	{ .compatible = "samsung,sofef01-m-ams597ut05",
	  .data = &samsung_sofef01_m_60_139_mode },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, samsung_sofef01_m_of_match);

static struct mipi_dsi_driver samsung_sofef01_m_driver = {
	.probe = samsung_sofef01_m_probe,
	.remove = samsung_sofef01_m_remove,
	.driver = {
		.name = "panel-samsung-sofef01-m",
		.of_match_table = samsung_sofef01_m_of_match,
	},
};
module_mipi_dsi_driver(samsung_sofef01_m_driver);

MODULE_AUTHOR("Marijn Suijten <marijn.suijten@somainline.org>");
MODULE_DESCRIPTION("DRM panel driver for Samsung SOFEF01-M Driver-IC panels");
MODULE_LICENSE("GPL");
