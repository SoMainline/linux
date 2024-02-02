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

/* I2C commands */
#define REG_SMBUS_MULTI_READ		0x00
#define REG_SMBUS_MULTI_WRITE		0x01
#define REG_SMBUS_READ			0x02
#define REG_SMBUS_WRITE			0x03

/* SMBUS_R/W registers */
#define REG_01				0x01
 // writing to this triggers a zero/reload of like a bajillion registers:
 /*
  [ 0x2, 0x8, 0xb, 0xf, 0x10, 0x12, 0x16, 0x17, 0x19, 0x1a, 0x1b, 0x1c, 0x1d,
   0x1e, 0x1f, 0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28,
   0x29, 0x2a, 0x2b, 0x2c, 0x2d, 0x2e, 0x2f, 0x30, 0x31, 0x32, 0x33,
   0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3a, 0x3b, 0x3c, 0x47, 0x4a,
   0x4f, 0x50, 0x52, 0x54, 0x55, 0x56, 0x57, 0x5d, 0x5e, 0x5f, 0x60,
   0x61, 0x62, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69, 0x6a, 0x6b,
   0x6c, 0x6d, 0x6e, 0x6f, 0x70, 0x71, 0x72, 0x73, 0x74, 0x75, 0x77,
   0x79, 0x7a, 0x81, 0x82, 0x84, 0x85, 0x8a, 0x8c, 0x8d, 0x8e, 0x8f,
   0x91, 0x93, 0x94, 0x99, 0x9e, 0x9f, 0xa0, 0xa1, 0xa2, 0xa3, 0xa4,
   0xa5, 0xa6, 0xa7, 0xb0, 0xb1, 0xb2, 0xbd, 0xbe, 0xbf, 0xc0, 0xc1,
   0xc3, 0xc5, 0xc9, 0xca, 0xcc, 0xcd, 0xce, 0xcf, 0xd0, 0xd1, 0xd2,
   0xd3, 0xd4, 0xd5, 0xd6, 0xd7, 0xd8, 0xd9, 0xf0, 0xf1, 0xf2, 0xf3,
   0xf4, 0xf5, 0xf6, 0xf7, 0xf8, 0xf9, 0xfa, 0xfb, 0xfc, 0xfd, 0xfe ]
  */
//bit(0) is settable

#define REG_02				0x02
//BIT(1) by default, 0 setable

#define REG_03				0x03
 //BIT(2) settable?

#define REG_05				0x05
 //0x0 vs 0x18 vs 0x01

#define REG_06				0x06
 //0x0, 0x80

#define REG_08				0x08
 // 0xb3(def) vs 0x1e

#define REG_LED_STS			0x0a
 #define REG_LED_STS_X			BIT(0)
 #define REG_LED_STS_FNLOCK		BIT(4)
 #define REG_LED_STS_Y			BIT(5)

#define REG_STH				0x0b
 #define REG_STH_4			BIT(4)
 #define REG_STH_PLUGGEDIN		BIT(6)

#define REG_LED_STS2			0x0c // R/O
 #define REG_LED_STS2_AUDIO_MUTE	BIT(0)
 #define REG_LED_STS2_CAPSLOCK		BIT(2)
 #define REG_LED_STS2_X			BIT(4)
 #define REG_LED_STS2_Y			BIT(5)

#define REG_0F				0x0F
 //0x30(def) vs 0x1

#define REG_10				0x10
 //0x80, 0xa0

#define REG_11				0x11
  //0x0 vs 0xb1

#define REG_12				0x12
  //0x7 vs 0x57

#define REG_SCMS_15			0x15 // fn stupid mode?
 #define REG_SCMS_15_FN_STICKY			BIT(0) // write val of 0x51[4] here?
 #define REG_SCMS_15_VAL		GENMASK(2, 1) // TODO: QEVs 0x73, 0x70, 0x72 fire on press
  #define REG_SCMS_15_VAL_XXX 2 //also \/ ???
  #define REG_SCMS_15_VAL_CYCLE	1 // one of the modes: juggle between fn normal, for 1 press and sticky

#define REG_23				0x23
//0x80

#define REG_27				0x27
//0x80

#define REG_4A				0x4A
// 0x0default - 0x0f

#define REG_53_0			0x50 // 0x1 vs vs 0x04(def) vs 0x24 (bmsk?)

#define REG_FNLOCK			0x51
 #define REG_FNLOCK_DISABLED		BIT(0) // r(status)/w(disable only)
 //maybe persistent setting storage??
 #define REG_FNLOCK_EN			BIT(4) // write => next keypress has fn (under what circumstances?)
 #define REG_FNLOCK_UNK			BIT(7)

#define REG_52				0x52
 // 0x84 (def) vs 0x5

#define REG_53				0x53
 #define REG_53_FNKEY_TRISTATE		GENMASK(1, 0)
  #define SOME_KEY_OFF			0 /* Fn is not active */
  #define SOME_KEY_ONESHOT		1 /* Tap Fn once, then press anoher key */
  #define SOME_KEY_PERMA		2 /* Double-tap Fn for FnLock (with no LED) */
  /* 3 - unused */
 #define REG_53_FNKEY_TRISTATE2		GENMASK(3, 2)
  /* 0 - res / ignore */
  #define SOME_KEY_L0			1
  #define SOME_KEY_L1			2
  #define SOME_KEY_L2			3
 #define REG_AUDIO_MUTE_LED		BIT(4) // set/clr on/off/get

/* 0x54..0x55 - 1-bit */

#define REG_57				0x57
 // 0x40(def) vs 0x1

#define REG_5B				0x5B
 // 0x18 (def) vs 0x0

/* 0x60..0x6f - 1bit regs */

#define REG_70				0x70
 #define REG_70_LID_OPEN		BIT(1) //set if open
 #define REG_70_PLUGGEDIN		BIT(5)

// 0x71 - 1bit

#define REG_73				0x73

#define REG_74				0x74
// Reg 0x74 changed from 0x1 to 0x8

#define REG_75				0x75
 #define REG_75_0			BIT(0)
 #define REG_75_2			BIT(2)
 #define REG_75_LCD_BL_OFF		BIT(5)
 #define REG_75_6			BIT(6)

#define REG_77				0x77
 #define REG_77_FN_HELD_DOWN		BIT(2) // ???
 #define REG_77_4			BIT(4) //bryan has it on
 #define REG_77_LCD_BL_ON		BIT(5)

#define REG_7A				0x7A
//0x1 -> 0x10

/* these two consume QEVs */
#define REG_QEV_X			0x7C
#define REG_QEV_Y			0x7D

#define REG_STH2			0x7f
 #define REG_STH2_PLUGGEDIN		BIT(6)

#define REG_SUS_CTL			0x80
 #define REG_SUS_CTL_SUS_ENTER		0x55
 #define REG_SUS_CTL_S0IX_OFF		0x66 //also called on disp off notif
 #define REG_SUS_CTL_SUS_EXIT		0xAA
 #define REG_SUS_CTL_S0IX_EXIT		0xBB //also called on disp on notif

#define REG_81				0x81
// 0 def vs 0x12

#define REG_82				0x82
// 0xa def vs 0x1e

#define REG_84				0x84
// 0x2(def) vs 0x3

#define REG_8A				0x8A
 #define REG_8A_MAYBE_BATDRAINING	BIT(0)

#define REG_8B				0x8B
 //keeps pulsin between 0x1 and 0x7 on readback

#define REG_8C				0x8C
 //bit1

#define REG_8D				0x8D
 //bit1

#define REG_8E				0x8E
 //bit1

#define REG_8F				0x8F
 //bit1

// 0x91 1bit? */

#define REG_94				0x94
 // REG 0x94 changed from 0x1 to 0xff

#define REG_97				0x97
//bit0

#define REG_A8				0xA8
 #define A8_0				BIT(0) //keeps pulsing randomly

#define REG_A9				0xA9
 #define A9_X				GENMASK(4, 0) //bit 0 pulses randomly
 #define A9_5				BIT(5) // stays on

#define REG_AA				0xAA
 //bti 0 keeps pulsing
 // 0x22-0x23

#define REG_AB				0xAB
 //bits (012) and 3 pulse randomly
 //bit 5 stays on

#define REG_AC				0xAC
 #define REG_AC_0			BIT(0) //keeps pulsing randomly
 #define REG_AC_1			BIT(1)
 #define REG_AC_2			BIT(2)
 #define REG_AC_5			BIT(5)

#define REG_AD				0xAD
 //see reg_ab

#define REG_BD				0xbd
 //0x0 (def) vs 0x1

#define REG_KBD				0xC0
 #define REG_KBD_MICMUTE		BIT(0) // wr here to turn on
 #define REG_KBD_SCMS_0X20		GENMASK(2, 1) // SCMS() gets this value and writes to 0x15[2:1]/0xc0[5:4]
  #define SOMETHING_X			0
  #define SOMETHING_Y			1
  #define SOMETHING_Z			2
 #define REG_KBD_BKL_LVL		GENMASK(5, 4) //brightness only
  #define KBD_BKL_LVL_OFF		0
  #define KBD_BKL_LVL_LO		1
  #define KBD_BKL_LVL_HI		2
 #define REG_KBD_6			BIT(6) //unk, on at boot
  /* 3 - unknown/reserved? */

#define REG_C3				0xC3
 //0x1 (def)vs 0x0?

#define REG_C5				0xC5
 #define REG_C5_0			BIT(0) // INGO(0x03) & 0xCF, if ~BIT(0), set it, unset otherwise
 #define REG_C5_7			BIT(7) // INGO(0x03, 0x1) & ~0xCF[7], set it, if INGO(0x03, 0x1) & 0xCF[7] clr it

#define REG_C6				0xC6 //convertible button
 #define REG_C6_CONERTIBLE_STATE	GENMASK(3, 0)

#define REG_C9				0xC9
 // 0x4(def) vs 0x1

#define REG_CA				0xCA
 #define CA_0				BIT(0) // ????
 #define REG_CA_SUS_ALLOWED		BIT(1) // leds blink if true, if true DYTC(0x001f1001) else DYTC(0x000f1001)
 #define REG_EN_KBD_CTL			BIT(2) // some enable
 #define REG_EN_KB_BKL			BIT(3) // some optional value to be used after setting BIT(2)
 #define REG_CA_5			BIT(5) // on by default
 #define REG_EN_SOMETHING_WAKEUP	BIT(6)
 #define REG_CA_KBD_LIGHT_PRESENT_MAYBE	BIT(7)

#define REG_CD				0xCD
 //bit 0|1 -> bit0

#define REG_CE				0xCE
 //0x0 (def) vs 0x1

#define REG_CF				0xCF
 #define REG_CF_0			BIT(0) // INGO(=0x04, 0x1), if BIT(0), flip it
 #define REG_CF_2			BIT(2)
 #define REG_CF_5			BIT(5)
 #define REG_CF_7			BIT(7) // INGO(=0x03, 0x0), if ~BIT(7), set it, unset otherwise

/* d0..d9 may read 0x7e */

#define REG_E1				0xE1
 //bit0

#define REG_EF				0xEF
 //1bit

#define REG_F6				0xF6
// 0x37(def) vs 0x39

#define REG_FC				0xFC
  //0x31(def) vs 0x33

#define REG_FE				0xFE
 //0x7e, may be duplicating ff?

#define REG_FF				0xFF
 //bit0, may be correlated w/ 0x22
 //0x7e

// polling period: 10 ms
// passive <=95C, critical 99C
/* SMBUS_MULTI_R/W regs */
#define REG_TEMP0_VAL			0x2020 // TZ70, deci C
#define NUM_THERMAL_SENSORS		6

/* Simple readback regs */
#define REG_REV_MAJ			0x00
#define REG_REV_MIN			0x01 // ?
#define REG_QUICK_EVENT			0xf0
enum ec_quick_events {
	EC_QEV_SLEEP = 0x13, /* Fn + 4 */
	EC_QEV_MONITOR_KEY = 0x19, /* Fn + F7 */
	EC_QEV_KEYBOARD_LIGHT = 0x1f, /* Fn + Space */
	EC_QEV_MICMUTE = 0x28, /* Fn + F4 */ // won't work due to 0x8 == HPDD & 0x08 == 0x08?
	EC_QEV_AIRPLANE = 0x2a, /* Fn + F8 */
	EC_QEV_THERMAL_ALERT = 0x36, //TZ70, acpi sets it to 127deg briefly
	EC_QEV_PERF_MODE_MAYBE = 0x3c, /* Maybe some LPM/idle entry posible notif? */
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

struct x13s_ec {
	struct i2c_client *client;
	struct regmap *regmap;
	struct gpio_desc *reset_gpio;
	bool suspended;
	struct mutex lock;
	//int last_convertible_mode;
};

static int last_reg_state[0xff] = { 0 };
static int x13s_ec_read(struct x13s_ec *ec, u8 reg, u8 *buf);
static irqreturn_t x13s_ec_intr(int irq, void *data)
{
	struct x13s_ec *ec = data;
	unsigned int event;

	guard(mutex)(&ec->lock);

	regmap_read(ec->regmap, 0xf0, &event);
	if (event)
		pr_err("got ev = 0x%x\n", event);

	int i;
	for (i = 0; i <= 0xff; i++) {
		if (i == REG_QEV_X || i == REG_QEV_Y || i == 0xa8 || i == 0xa9 || i == 0xaa || i == 0xab || i == 0xac || i == 0xad || i == 0x8b)
			continue;
		u8 buf;
		x13s_ec_read(ec, i, &buf);
		if ((i == 0x77 && ((last_reg_state[i] ^ buf) == 0x04)))
			continue;

		if (buf != last_reg_state[i]) {
			pr_err(" REG 0x%x changed from 0x%x to 0x%x!\n", i, last_reg_state[i], buf);
			last_reg_state[i] =buf;
		}
	}

	return IRQ_HANDLED;
}

static int x13s_ec_multi_write(struct x13s_ec *ec, u16 reg, u8 val)
{
	return i2c_smbus_write_i2c_block_data(ec->client,
					      REG_SMBUS_MULTI_WRITE,
					      3,
					      (u8 []){ reg & 0xff, reg >> 8, val });
}

static int x13s_ec_write(struct x13s_ec *ec, u8 reg, u8 val)
{
	return i2c_smbus_write_i2c_block_data(ec->client,
					      REG_SMBUS_WRITE,
					      2,
					      (u8 []){ reg, val });
}

static int x13s_ec_multi_read(struct x13s_ec *ec, u16 reg, u8 *buf)
{
	int ret;

	ret = i2c_smbus_write_word_data(ec->client,
					REG_SMBUS_MULTI_READ,
					reg);
	if (ret)
		return ret;

	return i2c_smbus_read_i2c_block_data(ec->client,
					     REG_SMBUS_MULTI_READ,
					     1, buf);
}

static int x13s_ec_read(struct x13s_ec *ec, u8 reg, u8 *buf)
{
	int ret;

	// TODO: figure out MULTI_READ
	ret = i2c_smbus_write_byte_data(ec->client, REG_SMBUS_READ, reg);
	if (ret)
		return ret;

	return i2c_smbus_read_i2c_block_data(ec->client,
					     REG_SMBUS_READ,
					     1, buf);
}

static int x13s_ec_suspend(struct device *dev)
{
	struct x13s_ec *ec = dev_get_drvdata(dev);
	int i;

	if (ec->suspended)
		return 0;

	for (i = 0; i < 3; i++) {
		x13s_ec_write(ec, REG_SUS_CTL, REG_SUS_CTL_SUS_ENTER);
		msleep(10);
	}

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
		x13s_ec_write(ec, REG_SUS_CTL, REG_SUS_CTL_SUS_EXIT);
		msleep(10);
	}

	ec->suspended = false;

	return 0;
}

// static int x13s_ec_get_temp(struct thermal_zone_device *tz, int *temp)
// {
	// TODO: check endianness :/
	//return x13s_ec_multi_read(thermal_zone_device_priv(tz), temp)
// }

static const struct regmap_config regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0xff,
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

	//guard(mutex)(&ec->lock);

	msleep(100);
	u8 buf[1] = {0};

	// devm_thermal_of_zone_register(bgp->dev, id, &data[id],&k3_of_thermal_ops);

	gpiod_set_value(ec->reset_gpio, 0);
	msleep(150);

	u8 longbuf[1] = { 0 };
	int i;
	for (i = 0; i < 6; i++) {
		x13s_ec_multi_read(ec, 0x2020 + i, longbuf);
		pr_err("TZ %u reports %u degC", i, longbuf[0]);
	}
#if 0
	for (i =0; i <= 0xff; i++) {
		x13s_ec_read(ec, i, buf);
		pr_err("reg[0x%x] = 0x%x\n", i, buf[0]);
	}
#endif
	// x13s_ec_read(ec, 0x0a, buf);
	// pr_err("EC [0x0a] = 0x%x\n", buf[0]);
	x13s_ec_write(ec, 0x2, 0x2);
	// x13s_ec_read(ec, 0x0a, buf);
	// pr_err("EC [0x0a] = 0x%x\n", buf[0]);

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
