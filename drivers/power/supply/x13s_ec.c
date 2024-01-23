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

struct x13s_ec {
	struct i2c_client *client;
	struct regmap *regmap;
	struct gpio_desc *reset_gpio;
	bool suspended;
	struct mutex lock;
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

	mutex_init(&ec->lock);

	dev_set_drvdata(dev, ec);
	ec->client = client;

	ec->regmap = devm_regmap_init_i2c(client, &regmap_config);
	if (IS_ERR(ec->regmap))
		return dev_err_probe(dev, PTR_ERR(ec->regmap), "couldn't init regmap\n");

	devm_pm_runtime_enable(dev);

	if (!device_link_add(dev, &client->adapter->dev, DL_FLAG_PM_RUNTIME | DL_FLAG_RPM_ACTIVE | DL_FLAG_STATELESS))
		return -ENODEV;

	regmap_read(ec->regmap, 0, &val);
	if (val)
		dev_info(dev, "Found Lenovo EC v%u\n", val);

	guard(mutex)(&ec->lock);

	msleep(100);
	u8 buf[1] = {0};

	int i;
	for (i = 0; i <= 0xff; i++) {
		i2c_smbus_write_byte_data(client, 0x02, (u8)i);
		i2c_smbus_read_i2c_block_data(client, 0x02, 1, buf);
		pr_err("reg[0x%x] = 0x%x\n", i, buf[0]);
		msleep(150);
	}

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

static struct i2c_driver x13s_ec_i2c_driver = {
	.driver = {
		.name = "thinkpad-x13s-ec",
		.of_match_table = x13s_ec_of_match,
	},
	.probe = x13s_ec_probe,
	.id_table = x13s_ec_i2c_id_table,
};
module_i2c_driver(x13s_ec_i2c_driver);

MODULE_DESCRIPTION("Lenovo Thinkpad X13s EC driver");
MODULE_LICENSE("GPL");
