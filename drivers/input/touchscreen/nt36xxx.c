// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2010 - 2018 Novatek, Inc.
 * Copyright (c) 2024, Linaro Ltd.
 */

#include <linux/cleanup.h>
#include <linux/gpio.h>
#include <linux/input/mt.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>

#include <drm/drm_panel.h>

#include "nt36xxx.h"

// TODO: BOOT_UPDATE_FIRMWARE should prob be a module parameter
// obv true for flashless chips, default to false for flash-equipped ones

enum nt36xxx_gestures {
	GESTURE_FIRST = 12,
	GESTURE_WORD_C = GESTURE_FIRST,
	GESTURE_WORD_W = 13,
	GESTURE_WORD_V = 14,
	GESTURE_DOUBLE_CLICK = 15,
	GESTURE_WORD_Z = 16,
	GESTURE_WORD_M = 17,
	GESTURE_WORD_O = 18,
	GESTURE_WORD_e = 19,
	GESTURE_WORD_S = 20,
	GESTURE_SLIDE_UP = 21,
	GESTURE_SLIDE_DOWN = 22,
	GESTURE_SLIDE_LEFT = 23,
	GESTURE_SLIDE_RIGHT = 24,
	GESTURE_MAX
};

static const u16 gesture_default_keycodes[] = {
	[GESTURE_WORD_C]	= KEY_POWER,
	[GESTURE_WORD_W]	= KEY_POWER,
	[GESTURE_WORD_V]	= KEY_POWER,
	[GESTURE_DOUBLE_CLICK]	= KEY_POWER,
	[GESTURE_WORD_Z]	= KEY_POWER,
	[GESTURE_WORD_M]	= KEY_POWER,
	[GESTURE_WORD_O]	= KEY_POWER,
	[GESTURE_WORD_e]	= KEY_POWER,
	[GESTURE_WORD_S]	= KEY_POWER,
	[GESTURE_SLIDE_UP]	= KEY_POWER,
	[GESTURE_SLIDE_DOWN]	= KEY_POWER,
	[GESTURE_SLIDE_LEFT]	= KEY_POWER,
	[GESTURE_SLIDE_RIGHT]	= KEY_POWER,
};

static void nvt_irq_enable(struct nvt_ts_data *ts, bool enable)
{
	struct irq_desc *desc;

	// TODO: cleanup at the end of the journey
	if (enable) {
		if (!WARN_ON(ts->irq_enabled))
			enable_irq(ts->client->irq);
	} else {
		if (WARN_ON(ts->irq_enabled))
			disable_irq(ts->client->irq);
	}

	ts->irq_enabled = enable;

	desc = irq_data_to_desc(irq_get_irq_data(ts->client->irq));
	NVT_LOG("enable=%d, desc->depth=%d\n", enable, desc->depth);
}

/* customized gesture id */
#define DATA_PROTOCOL           30
/* function page definition */
#define FUNCPAGE_GESTURE         1

#define GESTURE_DOUBLE_CLICK    15
static void nvt_ts_wakeup_gesture_report(struct nvt_ts_data *ts, u8 gesture_id, u8 *data)
{
	u8 func_type = data[2];
	u8 func_id = data[3];
	u32 keycode = 0;

	if (WARN_ON_ONCE(gesture_id <= GESTURE_FIRST || gesture_id >= GESTURE_MAX))
		return;

	/* support fw specifal data protocol */
	if (gesture_id == DATA_PROTOCOL && func_type == FUNCPAGE_GESTURE) {
		gesture_id = func_id;
	} else if (gesture_id > DATA_PROTOCOL) {
		NVT_ERR("gesture_id %d is invalid, func_type=%d, func_id=%d\n", gesture_id, func_type, func_id);
		return;
	}

	NVT_LOG("gesture_id = %d\n", gesture_id);

	switch (gesture_id) {
		case GESTURE_DOUBLE_CLICK:
			pr_err("Triggered gesture %d\n", gesture_id);
			keycode = gesture_default_keycodes[gesture_id];
			break;
		default:
			break;
	}

	if (keycode > 0) {
		input_report_key(ts->input_dev, keycode, 1);
		input_sync(ts->input_dev);
		input_report_key(ts->input_dev, keycode, 0);
		input_sync(ts->input_dev);
	}
}

/*******************************************************
Description:
	Novatek touchscreen parse device tree function.

return:
	n.a.
*******************************************************/
static int nvt_parse_dt(struct nvt_ts_data *ts)
{
	struct device *dev = &ts->client->dev;
	struct device_node *np = dev->of_node;
	int ret = 0;

	ts->reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ts->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(ts->reset_gpio), "Couldn't get reset gpio\n");

	ts->pen_support = of_property_read_bool(np, "novatek,pen-support");
	NVT_LOG("novatek,pen-support=%d\n", ts->pen_support);

	ts->wgp_stylus = of_property_read_bool(np, "novatek,wgp-stylus");
	NVT_LOG("novatek,wgp-stylus=%d\n", ts->wgp_stylus);

	ret = of_property_read_u32(np, "novatek,swrst-n8-addr", &ts->swrst_n8_addr);
	if (ret) {
		NVT_ERR("error reading novatek,swrst-n8-addr. ret=%d\n", ret);
		return ret;
	} else {
		NVT_LOG("swrst_n8_addr=0x%06X\n", ts->swrst_n8_addr);
	}

	ret = of_property_read_u32(np, "novatek,spi-rd-fast-addr", &ts->spi_rd_fast_addr);
	if (ret) {
		NVT_LOG("not support novatek,spi-rd-fast-addr\n");
		ts->spi_rd_fast_addr = 0;
		ret = 0;
	} else {
		NVT_LOG("spi_rd_fast_addr=0x%06X\n", ts->spi_rd_fast_addr);
	}

	return ret;
}

static u8 nvt_fw_recovery(u8 *point_data)
{
	int i;

	/* Look for 0x77777777777777 */
	for (i = 1; i < 7; i++)
		if (point_data[i] != 0x77)
			return false;

	return true;
}

#define RECOVERY_COUNT_MAX		10
static u8 nvt_wdt_fw_recovery(u8 *point_data)
{
	bool recovery_enable = false;
	static u8 recovery_cnt = 0;
	int i;

	recovery_cnt++;

	/* check pattern */
	for (i = 1; i < 7; i++) {
		if ((point_data[i] != 0xFD) && (point_data[i] != 0xFE)) {
			recovery_cnt = 0;
			break;
		}
	}

	if (recovery_cnt > RECOVERY_COUNT_MAX){
		recovery_enable = true;
		recovery_cnt = 0;
	}

	return recovery_enable;
}

#define PEN_DATA_LEN 14
#if CHECK_PEN_DATA_CHECKSUM
static int nvt_ts_pen_data_checksum(u8 *buf, u8 length)
{
	u8 checksum = 0;
	int i = 0;

	// Calculate checksum
	for (i = 0; i < length - 1; i++) {
		checksum += buf[i];
	}
	checksum = (~checksum + 1);

	// Compare ckecksum and dump fail data
	if (checksum != buf[length - 1]) {
		NVT_ERR("pen packet checksum not match. (buf[%d]=0x%02X, checksum=0x%02X)\n",
			length - 1, buf[length - 1], checksum);
		//--- dump pen buf ---
		for (i = 0; i < length; i++) {
			printk("%02X ", buf[i]);
		}
		printk("\n");

		return -1;
	}

	return 0;
}
#endif // #if CHECK_PEN_DATA_CHECKSUM

static int nvt_ts_point_data_checksum(u8 *buf, u8 length)
{
	u8 checksum = 0;
	int i;

	// Generate checksum
	for (i = 0; i < length - 1; i++)
		checksum += buf[i + 1];

	checksum = (~checksum + 1);

	return checksum != buf[length] ? -EINVAL : 0;
}

#define POINT_DATA_LEN 65
/*******************************************************
Description:
	Novatek touchscreen work function.

return:
	n.a.
*******************************************************/
#define PEN_FORMAT_ID_PRESENT			0x01
#define PEN_FORMAT_ID_DIDNT_MOVE		0xF0 /* "Still present, didn't move since last irq" */
#define PEN_FORMAT_ID_NO_PEN			0xFF
static irqreturn_t nvt_ts_work_func(int irq, void *data)
{
	u8 point_data[POINT_DATA_LEN + PEN_DATA_LEN + 1 + DUMMY_BYTES] = { 0 };
	bool finger_present[TOUCH_MAX_FINGER_NUM] = { false };
	u32 touch_x, touch_y, touch_major;
	bool any_finger_present = false;
	u32 pen_pressure, pen_distance;
	struct nvt_ts_data *ts = data;
	u32 pen_button1, pen_button2;
	s8 pen_tilt_x, pen_tilt_y;
	u32 max_touch_pressure;
	u8 pen_format_id;
	u32 pen_x, pen_y;
	u32 pen_battery;
	u8 input_id;
	u32 index;
	int ret;
	int i;

	if (ts->gesture_support && ts->suspended)
		pm_wakeup_event(&ts->input_dev->dev, 5000);

	guard(mutex)(&ts->lock);

	if (ts->pen_support)
		ret = CTP_SPI_READ(ts->client, point_data, POINT_DATA_LEN + PEN_DATA_LEN + 1);
	else
		ret = CTP_SPI_READ(ts->client, point_data, POINT_DATA_LEN + 1);
	if (ret < 0) {
		NVT_ERR("CTP_SPI_READ failed.(%d)\n", ret);
		return IRQ_HANDLED;
	}

	/* ESD protect by WDT */
	if (nvt_wdt_fw_recovery(point_data)) {
		NVT_ERR("Recover for fw reset, %02X\n", point_data[1]);
		nvt_update_firmware(ts, BOOT_UPDATE_FIRMWARE_NAME);
		return IRQ_HANDLED;
	}

	/* ESD protect by FW handshake */
	if (nvt_fw_recovery(point_data)) {
		return IRQ_HANDLED;
	}

	if (POINT_DATA_LEN >= POINT_DATA_CHECKSUM_LEN) {
		ret = nvt_ts_point_data_checksum(point_data, POINT_DATA_CHECKSUM_LEN);
		if (ret)
			return IRQ_HANDLED;
	}

	if (ts->gesture_support && ts->suspended) {
		input_id = (u8)(point_data[1] >> 3);

		nvt_ts_wakeup_gesture_report(ts, input_id, point_data);
		return IRQ_HANDLED;
	}

	for (i = 0; i < ts->max_finger_num; i++) {
		index = 1 + 6 * i;
		input_id = (u8)(point_data[index + 0] >> 3);
		if ((!input_id) || (input_id > ts->max_finger_num))
			continue;

		if (((point_data[index] & 0x07) == 0x01) || ((point_data[index] & 0x07) == 0x02)) {	//finger down (enter & moving)
			touch_x = (u32)(point_data[index + 1] << 4) + (u32) (point_data[index + 3] >> 4);
			if (touch_x > ts->abs_x_max)
				continue;

			touch_y = (u32)(point_data[index + 2] << 4) + (u32) (point_data[index + 3] & 0x0F);
			if (touch_y > ts->abs_y_max)
				continue;

			touch_major = (u32)(point_data[index + 4]);

			if (i < 2) {
				max_touch_pressure = (u32)(point_data[index + 5]) + (u32)(point_data[i + 63] << 8);
				max_touch_pressure = min_t(u32, max_touch_pressure, TOUCH_FORCE_NUM);
			} else {
				max_touch_pressure = point_data[index + 5];
			}

			finger_present[input_id - 1] = true;
			input_mt_slot(ts->input_dev, input_id - 1);
			input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, true);
			touchscreen_report_pos(ts->input_dev, &ts->prop, touch_x, touch_y, true);

			input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, touch_major ?: 1);
			input_report_abs(ts->input_dev, ABS_MT_PRESSURE, max_touch_pressure ?: 1);

			any_finger_present = true;
		}
	}

	/* "Untouch" all fingers that weren't reported with this irq */
	for (i = 0; i < ts->max_finger_num; i++) {
		if (!finger_present[i]) {
			input_mt_slot(ts->input_dev, i);
			input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0);
			input_report_abs(ts->input_dev, ABS_MT_PRESSURE, 0);
			input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, false);
		}
	}

	input_report_key(ts->input_dev, BTN_TOUCH, any_finger_present);
	input_sync(ts->input_dev);

	if (!ts->pen_support)
		return IRQ_HANDLED;

#if CHECK_PEN_DATA_CHECKSUM
	if (nvt_ts_pen_data_checksum(&point_data[66], PEN_DATA_LEN)) {
		// pen data packet checksum not match, skip it
		goto XFER_ERROR;
	}
#endif // #if CHECK_PEN_DATA_CHECKSUM

	// parse and handle pen report
	pen_format_id = point_data[66];
	if (pen_format_id == PEN_FORMAT_ID_NO_PEN) {
		input_report_abs(ts->pen_input_dev, ABS_X, 0);
		input_report_abs(ts->pen_input_dev, ABS_Y, 0);
		input_report_abs(ts->pen_input_dev, ABS_PRESSURE, 0);
		input_report_abs(ts->pen_input_dev, ABS_TILT_X, 0);
		input_report_abs(ts->pen_input_dev, ABS_TILT_Y, 0);
		input_report_abs(ts->pen_input_dev, ABS_DISTANCE, 0);

		input_report_key(ts->pen_input_dev, BTN_TOUCH, 0);
		input_report_key(ts->pen_input_dev, BTN_TOOL_PEN, 0);
		input_report_key(ts->pen_input_dev, BTN_STYLUS, 0);
		input_report_key(ts->pen_input_dev, BTN_STYLUS2, 0);
	} else {
		if (pen_format_id == PEN_FORMAT_ID_PRESENT) {
			// report pen data
			pen_x = (u32)(point_data[67] << 8) + (u32)(point_data[68]);
			pen_y = (u32)(point_data[69] << 8) + (u32)(point_data[70]);
			pen_pressure = (u32)(point_data[71] << 8) + (u32)(point_data[72]);
			pen_tilt_x = point_data[73];
			pen_tilt_y = point_data[74];
			pen_distance = (u32)(point_data[75] << 8) + (u32)(point_data[76]);
			pen_button1 = point_data[77] & BIT(0);
			pen_button2 = point_data[77] & BIT(1);

			// TODO: returns 0x10 when the pen works ok, maybe check for "low bat" warnings?
			pen_battery = (u32)point_data[78];

			// HACK invert xy
			input_report_abs(ts->pen_input_dev, ABS_X, ts->abs_x_max * 2 - pen_x);
			input_report_abs(ts->pen_input_dev, ABS_Y, ts->abs_y_max * 2 - pen_y);

			input_report_abs(ts->pen_input_dev, ABS_PRESSURE, pen_pressure);
			input_report_key(ts->pen_input_dev, BTN_TOUCH, !!pen_pressure);
			input_report_abs(ts->pen_input_dev, ABS_TILT_X, pen_tilt_x);
			input_report_abs(ts->pen_input_dev, ABS_TILT_Y, pen_tilt_y);
			input_report_abs(ts->pen_input_dev, ABS_DISTANCE, pen_distance);
			input_report_key(ts->pen_input_dev, BTN_TOOL_PEN, !!pen_distance || !!pen_pressure);
			input_report_key(ts->pen_input_dev, BTN_STYLUS, pen_button1);
			input_report_key(ts->pen_input_dev, BTN_STYLUS2, pen_button2);
		} else if (pen_format_id == PEN_FORMAT_ID_DIDNT_MOVE) {
			/* No change */
		} else {
			NVT_ERR("Unknown pen format id!\n");
			return IRQ_HANDLED;
		}
	}

	input_sync(ts->pen_input_dev);

	return IRQ_HANDLED;
}

static int nt36xxx_check_hw_id(struct nvt_ts_data *ts,
			       struct device *dev,
			       bool legacy_addr)
{
	u32 address = legacy_addr ? CHIP_VER_TRIM_OLD_ADDR : CHIP_VER_TRIM_ADDR;
	const struct nt36xxx_match_data *data;
	u8 buf[8] = { 0 };
	int i, index;
	int retries;

	data = device_get_match_data(dev);
	if (!data)
		return -EINVAL;

	for (retries = 5; retries > 0; retries--) {
		nvt_bootloader_reset(ts);

		nvt_set_page(ts, address);

		buf[0] = address & GENMASK(7, 0);
		buf[1] = 0x00;
		buf[2] = 0x00;
		buf[3] = 0x00;
		buf[4] = 0x00;
		buf[5] = 0x00;
		buf[6] = 0x00;
		CTP_SPI_READ(ts->client, buf, 7);

		/* Go over all expected IDs */
		for (index = 0; index < data->num_ids; index++) {
			const u8 *match_id = data->ids[index].bytes;

			for (i = 0; i < NVT_ID_BYTE_MAX; i++) {
				/* Ignore some parts of the ID (by design) */
				if (match_id[i] == ID_MATCH_ANY)
					continue;

				if (match_id[i] != buf[i + 1])
					break;
			}

			if (i == NVT_ID_BYTE_MAX) {
				print_hex_dump_bytes("Device id:\t", DUMP_PREFIX_NONE, match_id, NVT_ID_BYTE_MAX);
				print_hex_dump_bytes("Matched id:\t", DUMP_PREFIX_NONE, (buf + 1), NVT_ID_BYTE_MAX);
				ts->mmap = data->mmap;
				ts->hw_crc = data->hw_crc;
				return 0;
			}
		}

		usleep_range(10000, 11000);
	}

	return -ENODEV;
}

static int nt36xxx_touch_inputdev_init(struct nvt_ts_data *ts)
{
	struct device *dev = &ts->client->dev;
	int ret;
	int i;

	ts->input_dev = devm_input_allocate_device(dev);
	if (!ts->input_dev)
		return -ENOMEM;

	ts->max_finger_num = TOUCH_MAX_FINGER_NUM;

	set_bit(EV_ABS, ts->input_dev->evbit);
	set_bit(EV_KEY, ts->input_dev->evbit);
	set_bit(EV_SYN, ts->input_dev->evbit);
	set_bit(BTN_TOUCH, ts->input_dev->keybit);
	set_bit(INPUT_PROP_DIRECT, ts->input_dev->propbit);

	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 0, ts->abs_x_max, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0, ts->abs_y_max, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0); //area = 255
	input_set_abs_params(ts->input_dev, ABS_MT_PRESSURE, 0, TOUCH_FORCE_NUM, 0, 0); //pressure = TOUCH_FORCE_NUM

	ret = input_mt_init_slots(ts->input_dev, ts->max_finger_num, INPUT_MT_DIRECT);
	if (ret)
		return ret;

	if (ts->gesture_support) {
		for (i = 0; i < ARRAY_SIZE(gesture_default_keycodes); i++)
			input_set_capability(ts->input_dev, EV_KEY, gesture_default_keycodes[i]);
	}

	sprintf(ts->phys, "input/ts");
	ts->input_dev->name = NVT_TS_NAME;
	ts->input_dev->phys = ts->phys;
	ts->input_dev->id.bustype = BUS_SPI;

	touchscreen_parse_properties(ts->input_dev, true, &ts->prop);

	ret = input_register_device(ts->input_dev);
	if (ret)
		return dev_err_probe(dev, ret, "Couldn't register touchscreen input device\n");

	return 0;
}

static int nt36xxx_pen_inputdev_init(struct nvt_ts_data *ts)
{
	struct device *dev = &ts->client->dev;
	int ret;

	ts->pen_input_dev = devm_input_allocate_device(dev);
	if (!ts->pen_input_dev)
		return -ENOMEM;

	set_bit(EV_ABS, ts->pen_input_dev->evbit);
	set_bit(EV_KEY, ts->pen_input_dev->evbit);

	set_bit(BTN_TOUCH, ts->pen_input_dev->keybit);
	set_bit(BTN_TOOL_PEN, ts->pen_input_dev->keybit);
	set_bit(BTN_STYLUS, ts->pen_input_dev->keybit);
	set_bit(BTN_STYLUS2, ts->pen_input_dev->keybit);

	set_bit(INPUT_PROP_DIRECT, ts->pen_input_dev->propbit);
	set_bit(INPUT_PROP_POINTER, ts->pen_input_dev->propbit);

	if (ts->wgp_stylus) {
		input_set_abs_params(ts->pen_input_dev, ABS_X, 0, ts->abs_x_max * 2, 0, 0);
		input_set_abs_params(ts->pen_input_dev, ABS_Y, 0, ts->abs_y_max * 2, 0, 0);
	} else {
		input_set_abs_params(ts->pen_input_dev, ABS_X, 0, ts->abs_x_max, 0, 0);
		input_set_abs_params(ts->pen_input_dev, ABS_Y, 0, ts->abs_y_max, 0, 0);
	}
	input_set_abs_params(ts->pen_input_dev, ABS_PRESSURE, 0, PEN_PRESSURE_MAX, 0, 0);
	input_set_abs_params(ts->pen_input_dev, ABS_DISTANCE, 0, PEN_DISTANCE_MAX, 0, 0);
	input_set_abs_params(ts->pen_input_dev, ABS_TILT_X, PEN_TILT_MIN, PEN_TILT_MAX, 0, 0);
	input_set_abs_params(ts->pen_input_dev, ABS_TILT_Y, PEN_TILT_MIN, PEN_TILT_MAX, 0, 0);

	sprintf(ts->pen_phys, "input/pen");
	ts->pen_input_dev->name = NVT_PEN_NAME;
	ts->pen_input_dev->phys = ts->pen_phys;
	ts->pen_input_dev->id.bustype = BUS_SPI;

	ret = input_register_device(ts->pen_input_dev);
	if (ret)
		return dev_err_probe(dev, ret, "Couldn't register pen input device\n");

	return 0;
}

static void Boot_Update_Firmware(struct work_struct *work)
{
	struct nvt_ts_data *ts = container_of(work, struct nvt_ts_data, nvt_fwu_work.work);

	mutex_lock(&ts->lock);
	nvt_update_firmware(ts, BOOT_UPDATE_FIRMWARE_NAME);
	mutex_unlock(&ts->lock);
}

static int nvt_ts_probe(struct spi_device *client)
{
	struct device *dev = &client->dev;
	struct nvt_ts_data *ts;
	int ret;

	if (client->controller->flags & SPI_CONTROLLER_HALF_DUPLEX)
		return dev_err_probe(dev, -EIO, "The controller doesn't support full duplex\n");

	ts = devm_kzalloc(dev, sizeof(struct nvt_ts_data), GFP_KERNEL);
	if (!ts)
		return -ENOMEM;

	ts->xbuf = devm_kzalloc(dev, (NVT_TRANSFER_LEN + 1 + DUMMY_BYTES), GFP_KERNEL);
	if (!ts->xbuf)
		return -ENOMEM;

	ts->rbuf = devm_kzalloc(dev, NVT_READ_LEN, GFP_KERNEL);
	if (!ts->rbuf)
		return -ENOMEM;

	ts->client = client;
	ts->client->bits_per_word = 8;
	ts->client->mode = SPI_MODE_0;
	spi_set_drvdata(client, ts);

	ret = spi_setup(ts->client);
	if (ret < 0)
		return dev_err_probe(dev, ret, "Failed to perform SPI setup\n");

	ret = nvt_parse_dt(ts);
	if (ret) {
		NVT_ERR("parse dt error\n");
		return ret;
	}

	mutex_init(&ts->lock);
	mutex_init(&ts->xbuf_lock);

	/* Perform a software reset before pulling the reset pin */
	nvt_eng_reset(ts);

	gpiod_set_value_cansleep(ts->reset_gpio, 1);

	/* Wait at least 10ms after reset */
	usleep_range(10000, 11000);

	/* Check the "new address" (depends on fw version?) first */
	ret = nt36xxx_check_hw_id(ts, dev, false);
	if (ret) {
		/* Retry with the legacy address if the above failed */
		ret = nt36xxx_check_hw_id(ts, dev, true);
		if (ret)
			return dev_err_probe(dev, -ENODEV, "Unknown chip id\n");
	}

	ts->abs_x_max = TOUCH_DEFAULT_MAX_WIDTH;
	ts->abs_y_max = TOUCH_DEFAULT_MAX_HEIGHT;

	ret = nt36xxx_touch_inputdev_init(ts);
	if (ret)
		return ret;

	if (ts->pen_support) {
		ret = nt36xxx_pen_inputdev_init(ts);
		if (ret)
			return ret;
	}

	client->irq = platform_get_irq(to_platform_device(dev), 0);
	if (client->irq < 0)
		return dev_err_probe(dev, -EINVAL, "Couldn't get IRQ\n");

	ret = devm_request_threaded_irq(dev, client->irq, NULL, nvt_ts_work_func,
					IRQ_TYPE_EDGE_RISING | IRQF_NO_AUTOEN | IRQF_ONESHOT,
					"nt36xxx-spi", ts);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to request IRQ\n");

	ts->gesture_support = of_property_read_bool(dev->of_node, "novatek,gesture-support");
	device_init_wakeup(&ts->input_dev->dev, ts->gesture_support);

	ts->nvt_fwu_wq = alloc_workqueue("nvt_fwu_wq", WQ_UNBOUND | WQ_MEM_RECLAIM, 1);
	if (!ts->nvt_fwu_wq) {
		NVT_ERR("nvt_fwu_wq create workqueue failed\n");
		ret = -ENOMEM;
		goto err_create_nvt_fwu_wq_failed;
	}
	INIT_DELAYED_WORK(&ts->nvt_fwu_work, Boot_Update_Firmware);
	// please make sure boot update start after display reset(RESX) sequence
	queue_delayed_work(ts->nvt_fwu_wq, &ts->nvt_fwu_work, msecs_to_jiffies(14000));

	ts->suspended = false;

	nvt_irq_enable(ts, true);

	return 0;

err_create_nvt_fwu_wq_failed:
#if WAKEUP_GESTURE
	device_init_wakeup(&ts->input_dev->dev, false);
#endif

	return ret;
}

/*******************************************************
Description:
	Novatek touchscreen driver release function.

return:
	Executive outcomes. 0---succeed.
*******************************************************/
static void nvt_ts_remove(struct spi_device *client)
{
	struct nvt_ts_data *ts = spi_get_drvdata(client);

#if WAKEUP_GESTURE
	device_init_wakeup(&ts->input_dev->dev, false);
#endif

	if (ts->nvt_fwu_wq) {
		cancel_delayed_work_sync(&ts->nvt_fwu_work);
		destroy_workqueue(ts->nvt_fwu_wq);
		ts->nvt_fwu_wq = NULL;
	}

	if (ts->irq_enabled)
		nvt_irq_enable(ts, false);

	if (ts->pen_input_dev)
		input_unregister_device(ts->pen_input_dev);

	input_unregister_device(ts->input_dev);

	mutex_destroy(&ts->lock);
	mutex_destroy(&ts->xbuf_lock);
}

static void nvt_ts_shutdown(struct spi_device *client)
{
	struct nvt_ts_data *ts = spi_get_drvdata(client);

	NVT_LOG("Shutdown driver...\n");

	if (ts->irq_enabled)
		nvt_irq_enable(ts, false);

	if (ts->nvt_fwu_wq) {
		cancel_delayed_work_sync(&ts->nvt_fwu_work);
		destroy_workqueue(ts->nvt_fwu_wq);
		ts->nvt_fwu_wq = NULL;
	}

#if WAKEUP_GESTURE
	device_init_wakeup(&ts->input_dev->dev, false);
#endif
}

/*******************************************************
Description:
	Novatek touchscreen driver suspend function.

return:
	Executive outcomes. 0---succeed.
*******************************************************/
#define NVT_TS_SUSPEND_DEEP_SLEEP_MODE			0x11
#define NVT_TS_SUSPEND_WAKEUP_GESTURE_MODE		0x13
static int nvt_ts_suspend(struct device *dev)
{
	struct spi_device *client = to_spi_device(dev);
	struct nvt_ts_data *ts = spi_get_drvdata(client);
	u8 buf[4] = {0};
	int i;

	if (ts->suspended) {
		NVT_LOG("Touch is already suspend\n");
		return 0;
	}

	if (!ts->gesture_support)
		nvt_irq_enable(ts, false);

	mutex_lock(&ts->lock);

	NVT_LOG("start\n");

	ts->suspended = true;

	buf[0] = EVENT_MAP_HOST_CMD;
	if (ts->gesture_support) {
		buf[1] = NVT_TS_SUSPEND_WAKEUP_GESTURE_MODE;
		CTP_SPI_WRITE(ts->client, buf, 2);
		enable_irq_wake(ts->client->irq);
		NVT_LOG("Enabled touch wakeup gesture\n");
	} else {
		buf[1] = NVT_TS_SUSPEND_DEEP_SLEEP_MODE;
		CTP_SPI_WRITE(ts->client, buf, 2);
		NVT_LOG("deep sleep mode\n");
	} 

	mutex_unlock(&ts->lock);

	/* Release all touches */
	for (i = 0; i < ts->max_finger_num; i++) {
		input_mt_slot(ts->input_dev, i);
		input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0);
		input_report_abs(ts->input_dev, ABS_MT_PRESSURE, 0);
		input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, 0);
	}

	input_report_key(ts->input_dev, BTN_TOUCH, 0);

	input_sync(ts->input_dev);

	msleep(50);

	NVT_LOG("end\n");

	return 0;
}

/*******************************************************
Description:
	Novatek touchscreen driver resume function.

return:
	Executive outcomes. 0---succeed.
*******************************************************/
static int nvt_ts_resume(struct device *dev)
{
	struct spi_device *client = to_spi_device(dev);
	struct nvt_ts_data *ts = spi_get_drvdata(client);

	if (!ts->suspended) {
		NVT_LOG("Touch is already resume\n");
		return 0;
	}

	guard(mutex)(&ts->lock);

	NVT_LOG("start\n");

	// please make sure display reset(RESX) sequence and mipi dsi cmds sent before this
	gpiod_set_value_cansleep(ts->reset_gpio, 1);

	if (nvt_update_firmware(ts, BOOT_UPDATE_FIRMWARE_NAME)) {
		NVT_ERR("download firmware failed, ignore check fw state\n");
	} else {
		nvt_check_fw_reset_state(ts, RESET_STATE_REK);
	}

	if (!ts->gesture_support)
		nvt_irq_enable(ts, true);

	ts->suspended = false;

	NVT_LOG("end\n");

	return 0;
}
static DEFINE_SIMPLE_DEV_PM_OPS(nt36xxx_pm_ops, nvt_ts_suspend, nvt_ts_resume);

static const struct spi_device_id nvt_ts_id[] = {
	{ .name = "nt36xxx" },
	{ },
};
MODULE_DEVICE_TABLE(spi, nvt_ts_id);

static const struct nt36xxx_hw_id nt36523_hw_ids[] = {
	{ 0x0a, ID_MATCH_ANY, ID_MATCH_ANY, 0x23, 0x65, 0x03 },
	{ 0x0b, ID_MATCH_ANY, ID_MATCH_ANY, 0x23, 0x65, 0x03 },
	{ 0x0c, ID_MATCH_ANY, ID_MATCH_ANY, 0x23, 0x65, 0x03 },
	{ 0x20, ID_MATCH_ANY, ID_MATCH_ANY, 0x23, 0x65, 0x03 },
	{ ID_MATCH_ANY, ID_MATCH_ANY, ID_MATCH_ANY, 0x23, 0x65, 0x03 },
};

static const struct nt36xxx_match_data nt36523_data = {
	.ids = nt36523_hw_ids,
	.num_ids = ARRAY_SIZE(nt36523_hw_ids),
	.mmap = &nt36523_memory_map,
	.hw_crc = 2,
};

static const struct nt36xxx_hw_id nt36675_hw_ids[] = {
	{ 0x0c, ID_MATCH_ANY, ID_MATCH_ANY, 0x72, 0x66, 0x03 },
	{ ID_MATCH_ANY, ID_MATCH_ANY, ID_MATCH_ANY, 0x75, 0x66, 0x03 },
};

static const struct nt36xxx_match_data nt36675_data = {
	.ids = nt36675_hw_ids,
	.num_ids = ARRAY_SIZE(nt36675_hw_ids),
	.mmap = &nt36675_memory_map,
	.hw_crc = 2,
};

static const struct nt36xxx_hw_id nt36526_hw_ids[] = {
	{ ID_MATCH_ANY, ID_MATCH_ANY, ID_MATCH_ANY, 0x26, 0x65, 0x03 },
};

static const struct nt36xxx_match_data nt36526_data = {
	.ids = nt36526_hw_ids,
	.num_ids = ARRAY_SIZE(nt36526_hw_ids),
	.mmap = &nt36526_memory_map,
	.hw_crc = 2,
};

static const struct nt36xxx_hw_id nt36672a_hw_ids[] = {
	{ 0x0a, ID_MATCH_ANY, ID_MATCH_ANY, 0x70, 0x66, 0x03 },
	{ 0x0a, ID_MATCH_ANY, ID_MATCH_ANY, 0x72, 0x65, 0x03 },
	{ 0x0a, ID_MATCH_ANY, ID_MATCH_ANY, 0x72, 0x66, 0x03 },
	{ 0x0a, ID_MATCH_ANY, ID_MATCH_ANY, 0x72, 0x67, 0x03 },
	{ 0x0a, ID_MATCH_ANY, ID_MATCH_ANY, 0x82, 0x66, 0x03 },
	{ 0x0b, ID_MATCH_ANY, ID_MATCH_ANY, 0x25, 0x65, 0x03 },
	{ 0x0b, ID_MATCH_ANY, ID_MATCH_ANY, 0x70, 0x66, 0x03 },
	{ 0x0b, ID_MATCH_ANY, ID_MATCH_ANY, 0x72, 0x66, 0x03 },
	{ 0x0b, ID_MATCH_ANY, ID_MATCH_ANY, 0x82, 0x66, 0x03 },
};
static const struct nt36xxx_match_data nt36672a_data = {
	.ids = nt36672a_hw_ids,
	.num_ids = ARRAY_SIZE(nt36672a_hw_ids),
	.mmap = &nt36672a_memory_map,
	.hw_crc = 1,
};

static const struct nt36xxx_hw_id nt36772_hw_ids[] = {
	{ 0x55, 0x00, ID_MATCH_ANY, 0x00, 0x00, 0x00 },
	{ 0x55, 0x72, ID_MATCH_ANY, 0x00, 0x00, 0x00 },
	{ 0xaa, 0x00, ID_MATCH_ANY, 0x00, 0x00, 0x00 },
	{ 0xaa, 0x72, ID_MATCH_ANY, 0x00, 0x00, 0x00 },
	{ ID_MATCH_ANY, ID_MATCH_ANY, ID_MATCH_ANY, 0x70, 0x66, 0x03 },
	{ ID_MATCH_ANY, ID_MATCH_ANY, ID_MATCH_ANY, 0x70, 0x67, 0x03 },
	{ ID_MATCH_ANY, ID_MATCH_ANY, ID_MATCH_ANY, 0x72, 0x66, 0x03 },
	{ ID_MATCH_ANY, ID_MATCH_ANY, ID_MATCH_ANY, 0x72, 0x67, 0x03 },
};
static const struct nt36xxx_match_data nt36772_data = {
	.ids = nt36772_hw_ids,
	.num_ids = ARRAY_SIZE(nt36772_hw_ids),
	.mmap = &nt36772_memory_map,
	.hw_crc = 0,
};

static const struct nt36xxx_hw_id nt36525_hw_ids[] = {
	{ ID_MATCH_ANY, ID_MATCH_ANY, ID_MATCH_ANY, 0x25, 0x65, 0x03 },
};

static const struct nt36xxx_match_data nt36525_data = {
	.ids = nt36525_hw_ids,
	.num_ids = ARRAY_SIZE(nt36525_hw_ids),
	.mmap = &nt36525_memory_map,
	.hw_crc = 0,
};

static const struct nt36xxx_hw_id nt36676f_hw_ids[] = {
	{ ID_MATCH_ANY, ID_MATCH_ANY, ID_MATCH_ANY, 0x76, 0x66, 0x03 },
};

static const struct nt36xxx_match_data nt36676f_data = {
	.ids = nt36676f_hw_ids,
	.num_ids = ARRAY_SIZE(nt36676f_hw_ids),
	.mmap = &nt36676f_memory_map,
	.hw_crc = 0,
};

static const struct of_device_id nt36xxx_of_match_table[] = {
	{ .compatible = "novatek,nt36523", .data = &nt36523_data },
	{ .compatible = "novatek,nt36675", .data = &nt36675_data },
	{ .compatible = "novatek,nt36526", .data = &nt36526_data },
	{ .compatible = "novatek,nt36672a", .data = &nt36672a_data },
	{ .compatible = "novatek,nt36772", .data = &nt36772_data },
	{ .compatible = "novatek,nt36525", .data = &nt36525_data },
	{ .compatible = "novatek,nt36676f", .data = &nt36676f_data },
	{ },
};
MODULE_DEVICE_TABLE(of, nt36xxx_of_match_table);

static struct spi_driver nvt_spi_driver = {
	.driver = {
		.name = "nt36xxx-spi",
		.of_match_table = nt36xxx_of_match_table,
		// TODO:
		// .pm = pm_sleep_ptr(&nt36xxx_pm_ops),
	},
	.probe = nvt_ts_probe,
	.remove = nvt_ts_remove,
	// .shutdown = nvt_ts_shutdown,
	.id_table = nvt_ts_id,
};
module_spi_driver(nvt_spi_driver);

MODULE_DESCRIPTION("Novatek Touchscreen Driver");
MODULE_LICENSE("GPL");
