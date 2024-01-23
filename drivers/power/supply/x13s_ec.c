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

#define REG_SMBUS_MULTI_READ		0x00
#define REG_SMBUS_MULTI_WRITE		0x01
#define REG_SMBUS_REQUEST		0x02
#define REG_SMBUS_RESPONSE		0x03

#define REG_REV_MAJ			0x00

#define REG_REV_MIN			0x01 // ?

#define REG_SCMS_15			0x15
 #define REG_SCMS_15_FN_STICKY			BIT(0) // write val of 0x51[4] here
 #define REG_SCMS_15_VAL		GENMASK(2, 1) // TODO: QEVs 0x73, 0x70, 0x72 fire on press
 // one of the modes: juggle between fn normal, for 1 press and sticky

#define REG_51				0x51
 #define REG_51_FN_STICKY_STATE		BIT(0) // wr here to turn off
 #define REG_51_FN_STICKY		BIT(4)

#define REG_53				0x53
 #define REG_53_0			BIT(0)
 #define REG_SOME_KEY_TRISTATE		GENMASK(3, 2)
  #define SOME_KEY_OFF			0
  #define SOME_KEY_L1			1 // unused?
  #define SOME_KEY_L2			2
  /* 3 - res / ignore */
 #define REG_AUDIO_MUTE_KEY		BIT(4)

#define REG_73				0x73
 

#define REG_7C				0x7C

#define REG_7D				0x7D

#define REG_SUS_CTL			0x80
 #define REG_SUS_CTL_SUS_ENTER		0x55
 #define REG_SUS_CTL_S0IX_OFF		0x66 //also called on disp off notif
 #define REG_SUS_CTL_SUS_EXIT		0xAA
 #define REG_SUS_CTL_S0IX_EXIT		0xBB //also called on disp on notif

#define REG_A8				0xA8
 #define A8_0				BIT(0)

#define REG_A9				0xA9
 #define A9_X				GENMASK(4, 0)
 #define A9_5				BIT(5)

#define REG_KBD				0xC0
 #define REG_KBD_MICMUTE		BIT(0)
 // BIT(1) is also micmute!?
 #define REG_KBD_BKL_LVL		GENMASK(5, 4) //brightness only
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
 #define CA_0				BIT(0) // ????
 #define REG_CA_WDOG_STH		BIT(1) // if true DYTC(0x001f1001) else DYTC(0x000f1001)
 #define REG_EN_KBD_CTL			BIT(2) // some enable
 #define REG_EN_KB_BKL			BIT(3) // some optional value to be used after setting BIT(2)
 #define REG_EN_SOMETHING_WAKEUP	BIT(6)
 #define REG_CA_KBD_LIGHT_PRESENT_MAYBE	BIT(7)


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
	EC_QEV_FN_NEXT = 0x70, /* "Fn is active for the next keypress" */
	EC_QEV_FN_VERY_STICKY = 0x72, /* FnLock is active for ALL keys */
	EC_QEV_FN_DONE = 0x73, /* EC_QEV_FN_NEXT has been consumed */
	EC_QEV_FNLOCK = 0x75, /* Fn + ESC */
	EC_QEV_CONVERTIBLE_BTN = 0x77,
};

#define REG_TEMP_DECICELCIUS		0x2020 /* ECMR */

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

static int x13s_ec_write(struct x13s_ec *ec, u16 reg, u8 val)
{
	int ret;

	struct i2c_msg i2c_msg[] = {
		{
			.addr = ec->client->addr,
			.flags = ec->client->flags,
			.len = 1,
			.buf = (u8[]){ 3, reg & 0xff, reg >> 8 },
		}, {
			.addr = ec->client->addr,
			.len = 1,
			.buf = (u8[]){ val },
		},
	};

	WARN_ON(!mutex_is_locked(&ec->lock));

//	ret = i2c_transfer(ec->client->adapter, i2c_msg, 2);
//	if (ret)
//		pr_err("ec: failed to i2c tx: %d\n", ret);

	return ret;
}

static int x13s_ec_read(struct x13s_ec *ec, u16 reg, u8 *buf, u8 buf_len)
{
	int ret = 0;
	u8 __free(kfree) *rx_buffer;

	rx_buffer = kzalloc(buf_len, GFP_KERNEL);
	if (!rx_buffer)
		return -ENOMEM;

	struct i2c_msg msg[] = {
		{
			.addr = ec->client->addr,
			.flags = ec->client->flags,
			.len = 1,
			.buf = (u8[]) { 0x02 },
		}, {
			.addr = ec->client->addr,
			.flags = ec->client->flags | I2C_M_RD,
			.len = buf_len,
			.buf = rx_buffer,
		},
	};

	ret = i2c_transfer(ec->client->adapter, msg, ARRAY_SIZE(msg));
	if (ret)
		pr_err("ec: failed to i2c rx: %d\n", ret);
	else
		memcpy(buf, rx_buffer, buf_len);

	return ret;
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

	mutex_init(&ec->lock);

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

	guard(mutex)(&ec->lock);

	msleep(100);
	u8 buf[1] = {0};
//	ret = x13s_ec_read(ec, REG_CA, buf, 1);
	i2c_smbus_write_byte_data(client, 0x02, REG_CA);
	i2c_smbus_read_i2c_block_data(client, 0x02, 1, buf);
	pr_err("buf[0] = 0x%x\n", buf[0]);

	msleep(100);

//	i2c_smbus_write_i2c_block_data(client, 3, 2, (u8 []){ REG_CA, 0x60 | BIT(7) });
//	msleep(100);

	int i;
#if 0
	for (i = 0; i <= 0xff; i++) {
		i2c_smbus_write_byte_data(client, 0x02, (u8)i);
		i2c_smbus_read_i2c_block_data(client, 0x02, 1, buf);
		pr_err("reg[%u] = 0x%x\n", i, buf[0]);
		msleep(150);
	}
#endif
//	for (i = 0; i <= 0xff; i++) {
//		pr_err("WRITING 0xFF TO REG 0x%x\n", i);
		i2c_smbus_write_i2c_block_data(client, 3, 2, (u8 []){ 192, 0x21 });
		msleep(100);
//		gpiod_set_value(ec->reset_gpio, 1);
//	}

	msleep(100);
	ret = x13s_ec_write(ec, REG_CA, 0xff);
	msleep(50);
	ret = x13s_ec_write(ec, REG_KBD, 0xa8 | BIT(0));
	if (ret)
		pr_err("1 failed with %d\n", ret);
	msleep(100);

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
//		.pm = &x13s_ec_pm_ops,
	},
	.probe = x13s_ec_probe,
	.id_table = x13s_ec_i2c_id_table,
};
module_i2c_driver(x13s_ec_i2c_driver);

MODULE_DESCRIPTION("Lenovo Thinkpad X13s EC driver");
MODULE_LICENSE("GPL");
