// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2010 - 2018 Novatek, Inc.
 * Copyright (c) 2024, Linaro Ltd.
 */

#include <linux/cleanup.h>
#include <linux/gpio.h>
#include <linux/input/mt.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/irqnr.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/proc_fs.h>

#include <drm/drm_panel.h>

#include "nt36xxx.h"

// TODO: BOOT_UPDATE_FIRMWARE should prob be a module parameter
// obv true for flashless chips, default to false for flash-equipped ones

#if WAKEUP_GESTURE
const uint16_t gesture_key_array[] = {
	KEY_POWER,  //GESTURE_WORD_C
	KEY_POWER,  //GESTURE_WORD_W
	KEY_POWER,  //GESTURE_WORD_V
	KEY_POWER,  //GESTURE_DOUBLE_CLICK
	KEY_POWER,  //GESTURE_WORD_Z
	KEY_POWER,  //GESTURE_WORD_M
	KEY_POWER,  //GESTURE_WORD_O
	KEY_POWER,  //GESTURE_WORD_e
	KEY_POWER,  //GESTURE_WORD_S
	KEY_POWER,  //GESTURE_SLIDE_UP
	KEY_POWER,  //GESTURE_SLIDE_DOWN
	KEY_POWER,  //GESTURE_SLIDE_LEFT
	KEY_POWER,  //GESTURE_SLIDE_RIGHT
};
#endif

static bool bTouchIsAwake = 0;

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

#if WAKEUP_GESTURE
/* customized gesture id */
#define DATA_PROTOCOL           30
/* function page definition */
#define FUNCPAGE_GESTURE         1

#if NVT_WAKEUP_GESTURE_CUSTOMIZE
#define GESTURE_DOUBLE_CLICK    15
/*******************************************************
Description:
	Novatek touchscreen wake up gesture key report function.

return:
	n.a.
*******************************************************/
static void nvt_ts_wakeup_gesture_report_customize(struct nvt_ts_data *ts,
						   uint8_t gesture_id, uint8_t *data)
{
	u8 func_type = data[2];
	u8 func_id = data[3];
	u32 keycode = 0;

	/* support fw specifal data protocol */
	if ((gesture_id == DATA_PROTOCOL) && (func_type == FUNCPAGE_GESTURE)) {
		gesture_id = func_id;
	} else if (gesture_id > DATA_PROTOCOL) {
		NVT_ERR("gesture_id %d is invalid, func_type=%d, func_id=%d\n", gesture_id, func_type, func_id);
		return;
	}

	NVT_LOG("gesture_id = %d\n", gesture_id);

	switch (gesture_id) {
		case GESTURE_DOUBLE_CLICK:
			NVT_LOG("Gesture : Double Click.\n");
			keycode = gesture_key_array[3];
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
#else
#define GESTURE_WORD_C          12
#define GESTURE_WORD_W          13
#define GESTURE_WORD_V          14
#define GESTURE_DOUBLE_CLICK    15
#define GESTURE_WORD_Z          16
#define GESTURE_WORD_M          17
#define GESTURE_WORD_O          18
#define GESTURE_WORD_e          19
#define GESTURE_WORD_S          20
#define GESTURE_SLIDE_UP        21
#define GESTURE_SLIDE_DOWN      22
#define GESTURE_SLIDE_LEFT      23
#define GESTURE_SLIDE_RIGHT     24
/*******************************************************
Description:
	Novatek touchscreen wake up gesture key report function.

return:
	n.a.
*******************************************************/
static void nvt_ts_wakeup_gesture_report(uint8_t gesture_id, uint8_t *data)
{
	uint32_t keycode = 0;
	uint8_t func_type = data[2];
	uint8_t func_id = data[3];

	/* support fw specifal data protocol */
	if ((gesture_id == DATA_PROTOCOL) && (func_type == FUNCPAGE_GESTURE)) {
		gesture_id = func_id;
	} else if (gesture_id > DATA_PROTOCOL) {
		NVT_ERR("gesture_id %d is invalid, func_type=%d, func_id=%d\n", gesture_id, func_type, func_id);
		return;
	}

	NVT_LOG("gesture_id = %d\n", gesture_id);

	switch (gesture_id) {
		case GESTURE_WORD_C:
			NVT_LOG("Gesture : Word-C.\n");
			keycode = gesture_key_array[0];
			break;
		case GESTURE_WORD_W:
			NVT_LOG("Gesture : Word-W.\n");
			keycode = gesture_key_array[1];
			break;
		case GESTURE_WORD_V:
			NVT_LOG("Gesture : Word-V.\n");
			keycode = gesture_key_array[2];
			break;
		case GESTURE_DOUBLE_CLICK:
			NVT_LOG("Gesture : Double Click.\n");
			keycode = gesture_key_array[3];
			break;
		case GESTURE_WORD_Z:
			NVT_LOG("Gesture : Word-Z.\n");
			keycode = gesture_key_array[4];
			break;
		case GESTURE_WORD_M:
			NVT_LOG("Gesture : Word-M.\n");
			keycode = gesture_key_array[5];
			break;
		case GESTURE_WORD_O:
			NVT_LOG("Gesture : Word-O.\n");
			keycode = gesture_key_array[6];
			break;
		case GESTURE_WORD_e:
			NVT_LOG("Gesture : Word-e.\n");
			keycode = gesture_key_array[7];
			break;
		case GESTURE_WORD_S:
			NVT_LOG("Gesture : Word-S.\n");
			keycode = gesture_key_array[8];
			break;
		case GESTURE_SLIDE_UP:
			NVT_LOG("Gesture : Slide UP.\n");
			keycode = gesture_key_array[9];
			break;
		case GESTURE_SLIDE_DOWN:
			NVT_LOG("Gesture : Slide DOWN.\n");
			keycode = gesture_key_array[10];
			break;
		case GESTURE_SLIDE_LEFT:
			NVT_LOG("Gesture : Slide LEFT.\n");
			keycode = gesture_key_array[11];
			break;
		case GESTURE_SLIDE_RIGHT:
			NVT_LOG("Gesture : Slide RIGHT.\n");
			keycode = gesture_key_array[12];
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
#endif
#endif

/*******************************************************
Description:
	Novatek touchscreen parse device tree function.

return:
	n.a.
*******************************************************/
static int32_t nvt_parse_dt(struct nvt_ts_data *ts)
{
	struct device_node *np = ts->client->dev.of_node;
	int32_t ret = 0;

	ts->reset_gpio = of_get_named_gpio(np, "novatek,reset-gpio", 0);
	NVT_LOG("novatek,reset-gpio=%d\n", ts->reset_gpio);

	ts->irq_gpio = of_get_named_gpio(np, "novatek,irq-gpio", 0);
	NVT_LOG("novatek,irq-gpio=%d\n", ts->irq_gpio);

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

/*******************************************************
Description:
	Novatek touchscreen config and request gpio

return:
	Executive outcomes. 0---succeed. not 0---failed.
*******************************************************/
static int nvt_gpio_config(struct nvt_ts_data *ts)
{
	int32_t ret = 0;

#if NVT_TOUCH_SUPPORT_HW_RST
	/* request RST-pin (Output/High) */
	if (gpio_is_valid(ts->reset_gpio)) {
		ret = gpio_request_one(ts->reset_gpio, GPIOF_OUT_INIT_LOW, "NVT-tp-rst");
		if (ret) {
			NVT_ERR("Failed to request NVT-tp-rst GPIO\n");
		}
	}
#endif

	/* request INT-pin (Input) */
	if (gpio_is_valid(ts->irq_gpio)) {
		ret = gpio_request_one(ts->irq_gpio, GPIOF_IN, "NVT-int");
		if (ret) {
			NVT_ERR("Failed to request NVT-int GPIO\n");
			goto err_request_irq_gpio;
		}
	}

	return ret;

err_request_irq_gpio:
	gpio_free(ts->reset_gpio);

	return ret;
}

static uint8_t nvt_fw_recovery(uint8_t *point_data)
{
	uint8_t i = 0;
	uint8_t detected = true;

	/* check pattern */
	for (i=1 ; i<7 ; i++) {
		if (point_data[i] != 0x77) {
			detected = false;
			break;
		}
	}

	return detected;
}

#define RECOVERY_COUNT_MAX		10
static uint8_t nvt_wdt_fw_recovery(uint8_t *point_data)
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
static int32_t nvt_ts_pen_data_checksum(uint8_t *buf, uint8_t length)
{
	uint8_t checksum = 0;
	int32_t i = 0;

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

static int nvt_ts_point_data_checksum(uint8_t *buf, uint8_t length)
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
static irqreturn_t nvt_ts_work_func(int irq, void *data)
{
	struct nvt_ts_data *ts = data;
	int32_t ret = -1;
	uint8_t point_data[POINT_DATA_LEN + PEN_DATA_LEN + 1 + DUMMY_BYTES] = { 0 };
	uint32_t position = 0;
	uint32_t input_x = 0;
	uint32_t input_y = 0;
	uint32_t input_w = 0;
	uint32_t input_p = 0;
	uint8_t input_id = 0;
	uint8_t press_id[TOUCH_MAX_FINGER_NUM] = {0};
	int32_t i = 0;
	int32_t finger_cnt = 0;
	uint8_t pen_format_id = 0;
	uint32_t pen_x = 0;
	uint32_t pen_y = 0;
	uint32_t pen_pressure = 0;
	uint32_t pen_distance = 0;
	int8_t pen_tilt_x = 0;
	int8_t pen_tilt_y = 0;
	uint32_t pen_btn1 = 0;
	uint32_t pen_btn2 = 0;
	uint32_t pen_battery = 0;

#if WAKEUP_GESTURE
	if (!bTouchIsAwake)
		pm_wakeup_event(&ts->input_dev->dev, 5000);
#endif

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

#if WAKEUP_GESTURE
	if (!bTouchIsAwake) {
		input_id = (uint8_t)(point_data[1] >> 3);
#if NVT_WAKEUP_GESTURE_CUSTOMIZE
		nvt_ts_wakeup_gesture_report_customize(ts, input_id, point_data);
#else
		nvt_ts_wakeup_gesture_report(input_id, point_data);
#endif
		return IRQ_HANDLED;
	}
#endif

	finger_cnt = 0;

	for (i = 0; i < ts->max_touch_num; i++) {
		position = 1 + 6 * i;
		input_id = (uint8_t)(point_data[position + 0] >> 3);
		if ((input_id == 0) || (input_id > ts->max_touch_num))
			continue;

		if (((point_data[position] & 0x07) == 0x01) || ((point_data[position] & 0x07) == 0x02)) {	//finger down (enter & moving)
			input_x = (uint32_t)(point_data[position + 1] << 4) + (uint32_t) (point_data[position + 3] >> 4);
			input_y = (uint32_t)(point_data[position + 2] << 4) + (uint32_t) (point_data[position + 3] & 0x0F);
			if ((input_x < 0) || (input_y < 0))
				continue;
			if ((input_x > ts->abs_x_max) || (input_y > ts->abs_y_max))
				continue;
			input_w = (uint32_t)(point_data[position + 4]);
			if (input_w == 0)
				input_w = 1;
			if (i < 2) {
				input_p = (uint32_t)(point_data[position + 5]) + (uint32_t)(point_data[i + 63] << 8);
				if (input_p > TOUCH_FORCE_NUM)
					input_p = TOUCH_FORCE_NUM;
			} else {
				input_p = (uint32_t)(point_data[position + 5]);
			}
			if (input_p == 0)
				input_p = 1;

			press_id[input_id - 1] = 1;
			input_mt_slot(ts->input_dev, input_id - 1);
			input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, true);

			// input_report_abs(ts->input_dev, ABS_MT_POSITION_X, input_x);
			// input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, input_y);

			// TODO: touchscreen_set_mt_pos
			touchscreen_report_pos(ts->input_dev, &ts->prop, input_x, input_y, true);
			input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, input_w);
			input_report_abs(ts->input_dev, ABS_MT_PRESSURE, input_p);

			finger_cnt++;
		}
	}

	for (i = 0; i < ts->max_touch_num; i++) {
		if (press_id[i] != 1) {
			input_mt_slot(ts->input_dev, i);
			input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0);
			input_report_abs(ts->input_dev, ABS_MT_PRESSURE, 0);
			input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, false);
		}
	}

	input_report_key(ts->input_dev, BTN_TOUCH, (finger_cnt > 0));

	input_sync(ts->input_dev);

	if (ts->pen_support) {
#if CHECK_PEN_DATA_CHECKSUM
		if (nvt_ts_pen_data_checksum(&point_data[66], PEN_DATA_LEN)) {
			// pen data packet checksum not match, skip it
			goto XFER_ERROR;
		}
#endif // #if CHECK_PEN_DATA_CHECKSUM

		// parse and handle pen report
		pen_format_id = point_data[66];
		if (pen_format_id != 0xFF) {
			if (pen_format_id == 0x01) {
				// report pen data
				pen_x = (uint32_t)(point_data[67] << 8) + (uint32_t)(point_data[68]);
				pen_y = (uint32_t)(point_data[69] << 8) + (uint32_t)(point_data[70]);
				pen_pressure = (uint32_t)(point_data[71] << 8) + (uint32_t)(point_data[72]);
				pen_tilt_x = (int32_t)point_data[73];
				pen_tilt_y = (int32_t)point_data[74];
				pen_distance = (uint32_t)(point_data[75] << 8) + (uint32_t)(point_data[76]);
				pen_btn1 = (uint32_t)(point_data[77] & 0x01);
				pen_btn2 = (uint32_t)((point_data[77] >> 1) & 0x01);
				pen_battery = (uint32_t)point_data[78];

				// HACK invert xy
				input_report_abs(ts->pen_input_dev, ABS_X, ts->abs_x_max * 2 - pen_x);
				input_report_abs(ts->pen_input_dev, ABS_Y, ts->abs_y_max * 2 - pen_y);

				input_report_abs(ts->pen_input_dev, ABS_PRESSURE, pen_pressure);
				input_report_key(ts->pen_input_dev, BTN_TOUCH, !!pen_pressure);
				input_report_abs(ts->pen_input_dev, ABS_TILT_X, pen_tilt_x);
				input_report_abs(ts->pen_input_dev, ABS_TILT_Y, pen_tilt_y);
				input_report_abs(ts->pen_input_dev, ABS_DISTANCE, pen_distance);
				input_report_key(ts->pen_input_dev, BTN_TOOL_PEN, !!pen_distance || !!pen_pressure);
				input_report_key(ts->pen_input_dev, BTN_STYLUS, pen_btn1);
				input_report_key(ts->pen_input_dev, BTN_STYLUS2, pen_btn2);
				// TBD: pen battery event report
				// NVT_LOG("pen_battery=%d\n", pen_battery);
			} else if (pen_format_id == 0xF0) {
				// report Pen ID
			} else {
				NVT_ERR("Unknown pen format id!\n");
				return IRQ_HANDLED;
			}
		} else { // pen_format_id = 0xFF, i.e. no pen present
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
		}

		input_sync(ts->pen_input_dev);
	}

	return IRQ_HANDLED;
}


/*******************************************************
Description:
	Novatek touchscreen check chip version trim function.

return:
	Executive outcomes. 0---NVT IC. -1---not NVT IC.
*******************************************************/
static int8_t nvt_ts_check_chip_ver_trim(struct nvt_ts_data *ts, uint32_t chip_ver_trim_addr)
{
	u8 buf[8] = { 0 };
	int retry = 0;
	int i, index;

	NVT_LOG("%s:enter\n",__func__);
	for (retry = 5; retry > 0; retry--) {
		nvt_bootloader_reset(ts);

		nvt_set_page(ts, chip_ver_trim_addr);

		buf[0] = chip_ver_trim_addr & 0x7F;
		buf[1] = 0x00;
		buf[2] = 0x00;
		buf[3] = 0x00;
		buf[4] = 0x00;
		buf[5] = 0x00;
		buf[6] = 0x00;
		CTP_SPI_READ(ts->client, buf, 7);

		// compare read chip id on supported list
		for (index = 0; index < ARRAY_SIZE(trim_id_table); index++) {
			// compare each byte
			for (i = 0; i < NVT_ID_BYTE_MAX; i++) {
				if (trim_id_table[index].mask[i]) {
					if (buf[i + 1] != trim_id_table[index].id[i])
						break;
				}
			}

			if (i == NVT_ID_BYTE_MAX) {
				NVT_LOG("This is NVT touch IC\n");
				ts->mmap = trim_id_table[index].mmap;
				ts->carrier_system = trim_id_table[index].hwinfo->carrier_system;
				ts->hw_crc = trim_id_table[index].hwinfo->hw_crc;
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

	//---allocate input device---
	ts->input_dev = devm_input_allocate_device(dev);
	if (!ts->input_dev)
		return -ENOMEM;

	ts->max_touch_num = TOUCH_MAX_FINGER_NUM;

	set_bit(EV_ABS, ts->input_dev->evbit);
	set_bit(EV_KEY, ts->input_dev->evbit);
	set_bit(EV_SYN, ts->input_dev->evbit);
	set_bit(BTN_TOUCH, ts->input_dev->keybit);
	set_bit(INPUT_PROP_DIRECT, ts->input_dev->propbit);

	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 0, ts->abs_x_max, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0, ts->abs_y_max, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0); //area = 255
	input_set_abs_params(ts->input_dev, ABS_MT_PRESSURE, 0, TOUCH_FORCE_NUM, 0, 0); //pressure = TOUCH_FORCE_NUM

	ret = input_mt_init_slots(ts->input_dev, ts->max_touch_num, INPUT_MT_DIRECT);
	if (ret)
		return ret;

#if WAKEUP_GESTURE
	for (i = 0; i < ARRAY_SIZE(gesture_key_array); i++)
		input_set_capability(ts->input_dev, EV_KEY, gesture_key_array[i]);
#endif

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

	//---set pen input device info.---
	set_bit(EV_ABS, ts->pen_input_dev->evbit);
	set_bit(EV_KEY, ts->pen_input_dev->evbit);

	set_bit(BTN_TOUCH, ts->pen_input_dev->keybit);
	set_bit(BTN_TOOL_PEN, ts->pen_input_dev->keybit);
	set_bit(BTN_STYLUS, ts->pen_input_dev->keybit);
	set_bit(BTN_STYLUS2, ts->pen_input_dev->keybit);

	set_bit(BTN_TOOL_MOUSE, ts->pen_input_dev->keybit);


	/* Kernel document event-codes.txt suggest tablet device need to add property INPUT_PROP_DIRECT,
		* but if add this property, pen cursor will not shown when hover.
		* Customer need to decide whehter to add this INPUT_PROP_DIRECT property themselves.
		*/
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

	//---register pen input device---
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

	//---request and config GPIOs---
	ret = nvt_gpio_config(ts);
	if (ret) {
		NVT_ERR("gpio config error!\n");
		return ret;
	}

	mutex_init(&ts->lock);
	mutex_init(&ts->xbuf_lock);

	//---eng reset before TP_RESX high
	nvt_eng_reset(ts);

	gpio_set_value(ts->reset_gpio, 1);

	// need 10ms delay after POR(power on reset)
	usleep_range(10000, 11000);

	//---check chip version trim---
	ret = nvt_ts_check_chip_ver_trim(ts, CHIP_VER_TRIM_ADDR);
	if (ret) {
		NVT_LOG("try to check from old chip ver trim address\n");
		ret = nvt_ts_check_chip_ver_trim(ts, CHIP_VER_TRIM_OLD_ADDR);
		if (ret)
			return dev_err_probe(dev, -ENODEV, "Unknown chip id\n"); // TODO: what chip id?
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

	//---set int-pin & request irq---
	client->irq = gpio_to_irq(ts->irq_gpio);
	if (!client->irq)
		return dev_err_probe(dev, -EINVAL, "tragic\n"); //

	ret = devm_request_threaded_irq(dev, client->irq, NULL, nvt_ts_work_func,
					IRQ_TYPE_EDGE_RISING | IRQF_NO_AUTOEN | IRQF_ONESHOT,
					"nt36xxx-spi", ts);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to request IRQ\n");

#if WAKEUP_GESTURE
	ts->gesture_enabled = false;
	device_init_wakeup(&ts->input_dev->dev, true);
#endif

	ts->nvt_fwu_wq = alloc_workqueue("nvt_fwu_wq", WQ_UNBOUND | WQ_MEM_RECLAIM, 1);
	if (!ts->nvt_fwu_wq) {
		NVT_ERR("nvt_fwu_wq create workqueue failed\n");
		ret = -ENOMEM;
		goto err_create_nvt_fwu_wq_failed;
	}
	INIT_DELAYED_WORK(&ts->nvt_fwu_work, Boot_Update_Firmware);
	// please make sure boot update start after display reset(RESX) sequence
	queue_delayed_work(ts->nvt_fwu_wq, &ts->nvt_fwu_work, msecs_to_jiffies(14000));

	bTouchIsAwake = 1;

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
static int32_t nvt_ts_suspend(struct device *dev)
{
	struct spi_device *client = to_spi_device(dev);
	struct nvt_ts_data *ts = spi_get_drvdata(client);
	uint8_t buf[4] = {0};
	int i;

	if (!bTouchIsAwake) {
		NVT_LOG("Touch is already suspend\n");
		return 0;
	}

	if (!ts->gesture_enabled)
		nvt_irq_enable(ts, false);

	mutex_lock(&ts->lock);

	NVT_LOG("start\n");

	bTouchIsAwake = false;

	buf[0] = EVENT_MAP_HOST_CMD;
	if (ts->gesture_enabled) {
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
	for (i = 0; i < ts->max_touch_num; i++) {
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
static int32_t nvt_ts_resume(struct device *dev)
{
	struct spi_device *client = to_spi_device(dev);
	struct nvt_ts_data *ts = spi_get_drvdata(client);

	if (bTouchIsAwake) {
		NVT_LOG("Touch is already resume\n");
		return 0;
	}

	guard(mutex)(&ts->lock);

	NVT_LOG("start\n");

	// please make sure display reset(RESX) sequence and mipi dsi cmds sent before this
	gpio_set_value(ts->reset_gpio, 1);

	if (nvt_update_firmware(ts, BOOT_UPDATE_FIRMWARE_NAME)) {
		NVT_ERR("download firmware failed, ignore check fw state\n");
	} else {
		nvt_check_fw_reset_state(ts, RESET_STATE_REK);
	}

	if (!ts->gesture_enabled)
		nvt_irq_enable(ts, true);

	bTouchIsAwake = true;

	NVT_LOG("end\n");

	return 0;
}

static const struct spi_device_id nvt_ts_id[] = {
	{ .name = "nt36xxx" },
	{ },
};
MODULE_DEVICE_TABLE(spi, nvt_ts_id);

static const struct of_device_id nt36xxx_of_match_table[] = {
	{ .compatible = "novatek,nt36xxx" },
	{ },
};
MODULE_DEVICE_TABLE(of, nt36xxx_of_match_table);

static struct spi_driver nvt_spi_driver = {
	.driver = {
		.name = "nt36xxx-spi",
		.of_match_table = nt36xxx_of_match_table,
	},
	.probe = nvt_ts_probe,
	.remove = nvt_ts_remove,
	// .shutdown = nvt_ts_shutdown,
	.id_table = nvt_ts_id,
};
module_spi_driver(nvt_spi_driver);

MODULE_DESCRIPTION("Novatek Touchscreen Driver");
MODULE_LICENSE("GPL");
