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

#define EC_REG_REV_MAJ			0x00
#define EC_REG_REV_MIN			0x01 // ?
#define EC_REG_51			0x51
 #define EC_REG_51_FN_STICKY		BIT(0)
#define EC_REG_53			0x53
#define  EC_REG_53_0			BIT(0)
#define  EC_REG_53_2			BIT(2)
#define  EC_REG_53_3			BIT(3)
#define  EC_REG_53_4			BIT(4)
#define EC_REG_C5			0xC5
#define  EC_REG_C5_7			BIT(7)
#define EC_REG_CA			0xCA
#define  EC_REG_CA_0			BIT(0) # _Q3C query, DYTC (0x001F1001) if true, else DYTC (0x000F1001)
#define  EC_REG_CA_6			BIT(6) # HKEY.MHKQ (0x1012) _Q1F, SCMS(0x0e), SCMS(0x20) afterwards
#define  EC_REG_CA_12			BIT(12) # KBBL, keyboard backlight? on/off swapped?
#define EC_REG_CF			0xCF
#define  EC_REG_CF_0			BIT(0)

#define EC_REG_QUICK_EVENT		0xf0
enum ec_quick_events {
	EC_QEV_SLEEP = 0x13, /* Fn + 4 */
	EC_QEV_MONITOR_KEY = 0x19, /* Fn + F7 */
	EC_QEV_KEYBOARD_LIGHT = 0x1f, /* Fn + Space */
	EC_QEV_MICMUTE = 0x28, /* Fn + F4 */
	EC_QEV_AIRPLANE = 0x2a, /* Fn + F8 */
	EC_QEV_WDOG_MAYBE = 0x3c, /* Pops up every 90s or more, wdog/heartbeat? */
	EC_QEV_CHG_CONNECT = 0x50, /* only works when power actually flows, LED blinks 3 times */
	EC_QEV_CHG_DISCONECT = 0x51,
	EC_QEV_LID_OPEN = 0x52, /* uncertain, hard to measure separately */
	EC_QEV_LID_SHUT = 0x53, /* uncertain, hard to measure separately */
	EC_QEV_PRINTSCREEN = 0x62, /* Fn + PrtScr */
	EC_QEV_CALL_KEY = 0x6c, /* Fn + F10 */
	EC_QEV_HANGUP_KEY = 0x6d, /* Fn + F11 */
	EC_QEV_FAVORITE_KEY = 0x6e, /* Fn + F12 */
	EC_QEV_FNLOCK = 0x75, /* Fn + ESC */
};

struct thinkpad_x13s_ec {
	struct i2c_client *client;
	struct regmap *regmap;
};

static irqreturn_t thinkpad_x13s_ec_intr(int irq, void *data)
{
	struct thinkpad_x13s_ec *ec = data;
	unsigned int event;
	int ret;

	msleep(10);
	ret = regmap_read(ec->regmap, 0xf0, &event);
	if (ret < 0) {
		pr_err("AAAAAAAAAAAAA\n");
		return IRQ_HANDLED;
	}

	if (event)
		pr_err("got ev = 0x%x\n", event);

	return IRQ_HANDLED;
}

static const struct regmap_config regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0xff, // ??
};

static int thinkpad_x13s_ec_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct thinkpad_x13s_ec *ec;
	int ret;
	u32 val;

	ec = devm_kzalloc(dev, sizeof(*ec), GFP_KERNEL);
	if (!ec)
		return -ENOMEM;

	ec->client = client;

	ec->regmap = devm_regmap_init_i2c(client, &regmap_config);
	if (IS_ERR(ec->regmap))
		return dev_err_probe(dev, PTR_ERR(ec->regmap), "couldn't init regmap\n");

	ret = devm_request_threaded_irq(dev, client->irq,
					NULL, thinkpad_x13s_ec_intr,
					IRQF_ONESHOT, "thinkpad_x13s_ec", ec);
	if (ret < 0)
		return dev_err_probe(dev, ret, "unable to request irq\n");

	regmap_read(ec->regmap, EC_REG_REV_MAJ, &val);
	if (val)
		dev_info(dev, "Found Lenovo EC v%u\n", val);

	msleep(50);
	regmap_write(ec->regmap, 0xcb, 0xff);
	msleep(5000);
	regmap_write(ec->regmap, 0xcb, 0);

	return 0;
}

static const struct of_device_id thinkpad_x13s_ec_of_match[] = {
	{ .compatible = "lenovo,thinkpad-ec-x13s" },
	{}
};
MODULE_DEVICE_TABLE(of, thinkpad_x13s_ec_of_match);

static const struct i2c_device_id thinkpad_x13s_ec_i2c_id_table[] = {
	{ "thinkpad-x13s-ec", },
	{}
};
MODULE_DEVICE_TABLE(i2c, thinkpad_x13s_ec_i2c_id_table);

static struct i2c_driver thinkpad_x13s_ec_i2c_driver = {
	.driver = {
		.name = "thinkpad-x13s-ec",
		.of_match_table = thinkpad_x13s_ec_of_match,
	},
	.probe = thinkpad_x13s_ec_probe,
	.id_table = thinkpad_x13s_ec_i2c_id_table,
};
module_i2c_driver(thinkpad_x13s_ec_i2c_driver);

MODULE_DESCRIPTION("Lenovo Thinkpad X13s EC driver");
MODULE_LICENSE("GPL");
