// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2022 Linaro Ltd.
 */
#include <linux/bitfield.h>
#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/pm.h>
#include <linux/pm_runtime.h>
#include <linux/gpio/consumer.h>

#define REG_REV_MAJ			0x00

#define REG_REV_MIN			0x01 // ?

#define REG_SCMS_15			0x15
 #define REG_SCMS_15_FN_STICKY			BIT(0) // write val of 0x51[4] here
 #define REG_SCMS_15_VAL		GENMASK(2, 1)

#define REG_51				0x51
 #define REG_51_FN_STICKY_STATE		BIT(0)
 #define REG_51_FN_STICKY		BIT(4)

#define REG_53				0x53
 #define REG_53_0			BIT(0)
 #define REG_SOME_KEY_TRISTATE		GENMASK(3, 2)
  #define SOME_KEY_OFF			0
  #define SOME_KEY_L1			1 // unused?
  #define SOME_KEY_L2			2
  /* 3 - res / ignore */
 #define REG_AUDIO_MUTE_KEY		BIT(4)

#define REG_SUS_CTL			0x80
 #define REG_SUS_CTL_S0IX_ENTER		0x55
 #define REG_SUS_CTL_SUS_ENTER		0x66
 #define REG_SUS_CTL_S0IX_EXIT		0xAA
 #define REG_SUS_CTL_SUS_EXIT		0xBB

#define REG_KBD				0xC0
 #define REG_KBD_MICMUTE		BIT(0)
 #define REG_KBD_BKL_LVL		GENMASK(5, 4)
  #define KBD_BKL_LVL_OFF		0
  #define KBD_BKL_LVL_LO		1
  #define KBD_BKL_LVL_HI		2
  /* 3 - unknown/reserved? */

#define REG_C5				0xC5
 #define REG_C5_0			BIT(0)
 #define REG_C5_7			BIT(7)

#define REG_C6				0xC6 //convertible button
 #define REG_C6_CONERTIBLE_STATE	GENMASK(3, 0)

#define REG_CA				0xCA
 #define REG_CA_WDOG_STH		BIT(1) // if true DYTC(0x001f1001) else DYTC(0x000f1001)
 #define REG_EN_KBD_CTL			BIT(2) // some enable
 #define REG_EN_KB_BKL			BIT(3) // some optional value to be used after setting BIT(2)
 #define REG_CA_KBD_LIGHT_PRESENT_MAYBE	BIT(7)


 #define REG_CA_0			BIT(0) # _Q3C query, DYTC (0x001F1001) if true, else DYTC (0x000F1001)
 #define REG_CA_6			BIT(6) # HKEY.MHKQ (0x1012) _Q1F, SCMS(0x0e), SCMS(0x20) afterwards
 #define REG_CA_12			BIT(12) # KBBL, keyboard backlight? on/off swapped?

#define REG_CF				0xCF
 #define REG_CF_0			BIT(0)

#define REG_QUICK_EVENT			0xf0
enum ec_quick_events {
	EC_QEV_SLEEP = 0x13, /* Fn + 4 */
	EC_QEV_MONITOR_KEY = 0x19, /* Fn + F7 */
	EC_QEV_KEYBOARD_LIGHT = 0x1f, /* Fn + Space */
	EC_QEV_MICMUTE = 0x28, /* Fn + F4 */ // won't work due to 0x8 == HPDD & 0x08 == 0x08?
	EC_QEV_AIRPLANE = 0x2a, /* Fn + F8 */
	EC_QEV_THERMAL_ALERT = 0x36, //TZ70, acpi sets it to 127deg briefly
	EC_QEV_WDOG_MAYBE = 0x3c, /* Pops up every 90s or more, wdog/heartbeat? */
	EC_QEV_CHG_CONNECT = 0x50, /* only works when power actually flows, LED blinks 3 times */
	EC_QEV_CHG_DISCONECT = 0x51,
	EC_QEV_LID_OPEN = 0x52,
	EC_QEV_LID_SHUT = 0x53,
	EC_QEV_PRINTSCREEN = 0x62, /* Fn + PrtScr */
	EC_QEV_CALL_KEY = 0x6c, /* Fn + F10 */
	EC_QEV_HANGUP_KEY = 0x6d, /* Fn + F11 */
	EC_QEV_FAVORITE_KEY = 0x6e, /* Fn + F12 */
	EC_QEV_FNLOCK = 0x75, /* Fn + ESC */
	EC_QEV_CONVERTIBLE_BTN = 0x77,
};

#define REG_TEMP_DECICELCIUS		0x2020 /* ECMR */

struct x13s_ec {
	struct i2c_client *client;
	struct regmap *regmap;
	struct gpio_desc *reset_gpio;
	bool suspended;
	//int last_convertible_mode;
};

static irqreturn_t x13s_ec_intr(int irq, void *data)
{
	struct x13s_ec *ec = data;
	unsigned int event;

	msleep(30);
	regmap_read(ec->regmap, 0xf0, &event);

	if (event)
		pr_err("got ev = 0x%x\n", event);

	return IRQ_HANDLED;
}

static int x13s_ec_suspend(struct device *dev)
{
	struct x13s_ec *ec = dev_get_drvdata(dev);

	if (ec->suspended)
		return 0;

	//regmap_write(ec->regmap, REG_SUS_CTL, REG_SUS_CTL_SUS_ENTER);
	msleep(10); // 30?

	gpiod_set_value(ec->reset_gpio, 1);

	ec->suspended = true;

	return 0;
}

static int x13s_ec_resume(struct device *dev)
{
	struct x13s_ec *ec = dev_get_drvdata(dev);
	int i;

	if (!ec->suspended)
		return 0;

	gpiod_set_value(ec->reset_gpio, 0);
	msleep(10);

	/* The EC or I2C host can be grumpy when waking up */
	for (i = 0; i < 3; i++) {
		//regmap_write(ec->regmap, REG_SUS_CTL, REG_SUS_CTL_SUS_EXIT);
		msleep(10);
	}

	ec->suspended = false;

	return 0;
}

static const struct regmap_config regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0xff, // ??
};

static int x13s_ec_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct x13s_ec *ec;
	int ret;
	u32 val;

	ec = devm_kzalloc(dev, sizeof(*ec), GFP_KERNEL);
	if (!ec)
		return -ENOMEM;

	dev_set_drvdata(dev, ec);
	ec->client = client;

	ec->regmap = devm_regmap_init_i2c(client, &regmap_config);
	if (IS_ERR(ec->regmap))
		return dev_err_probe(dev, PTR_ERR(ec->regmap), "couldn't init regmap\n");

	ret = devm_request_threaded_irq(dev, client->irq,
					NULL, x13s_ec_intr,
					IRQF_ONESHOT, "x13s_ec", ec);
	if (ret < 0)
		return dev_err_probe(dev, ret, "unable to request irq\n");

	ec->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ec->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(ec->reset_gpio), "couldn't get reset GPIO\n");

	devm_pm_runtime_enable(dev);

	if (!device_link_add(dev, &client->adapter->dev, DL_FLAG_PM_RUNTIME | DL_FLAG_RPM_ACTIVE | DL_FLAG_STATELESS))
		return -ENODEV;

	regmap_read(ec->regmap, REG_REV_MAJ, &val);
	if (val)
		dev_info(dev, "Found Lenovo EC v%u\n", val);

	msleep(50);
	regmap_write(ec->regmap, REG_CA, 0xff);
	msleep(5000);
	regmap_write(ec->regmap, 0xc0, BIT(5) | BIT(0));

	return 0;
}

static const struct of_device_id x13s_ec_of_match[] = {
	{ .compatible = "lenovo,thinkpad-ec-x13s" },
	{}
};
MODULE_DEVICE_TABLE(of, x13s_ec_of_match);

static const struct i2c_device_id x13s_ec_i2c_id_table[] = {
	{ "thinkpad-x13s-ec", },
	{}
};
MODULE_DEVICE_TABLE(i2c, x13s_ec_i2c_id_table);

static const struct dev_pm_ops x13s_ec_pm_ops = {
	NOIRQ_SYSTEM_SLEEP_PM_OPS(x13s_ec_suspend, x13s_ec_resume)
};

static struct i2c_driver x13s_ec_i2c_driver = {
	.driver = {
		.name = "thinkpad-x13s-ec",
		.of_match_table = x13s_ec_of_match,
		.pm = &x13s_ec_pm_ops,
	},
	.probe = x13s_ec_probe,
	.id_table = x13s_ec_i2c_id_table,
};
module_i2c_driver(x13s_ec_i2c_driver);

MODULE_DESCRIPTION("Lenovo Thinkpad X13s EC driver");
MODULE_LICENSE("GPL");
