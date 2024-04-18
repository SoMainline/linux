/*
 * Copyright (C) 2010 - 2018 Novatek, Inc.
 *
 * $Revision: 69262 $
 * $Date: 2020-09-23 15:07:14 +0800 (週三, 23 九月 2020) $
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 */
#ifndef 	_LINUX_NVT_TOUCH_H
#define		_LINUX_NVT_TOUCH_H

#include <linux/delay.h>
#include <linux/input.h>
#include <linux/of.h>
#include <linux/spi/spi.h>
#include <linux/uaccess.h>
#include <linux/input/touchscreen.h>

#include "nt36xxx_mem_map.h"

#define NVT_DEBUG 1

//---GPIO number---
#define NVTTOUCH_RST_PIN 980
#define NVTTOUCH_INT_PIN 943

#define CHECK_TOUCH_VENDOR
//---SPI driver info.---
#define NVT_LOG(fmt, args...)    pr_err("[%s] nt36xxx-spi %d: " fmt, __func__, __LINE__, ##args)
#define NVT_ERR(fmt, args...)    pr_err("[%s] nt36xxx-spi %d: " fmt, __func__, __LINE__, ##args)

//---Input device info.---
#define NVT_TS_NAME "NVTCapacitiveTouchScreen"
#define NVT_PEN_NAME "NVTCapacitivePen"

//---Touch info.---
#define TOUCH_DEFAULT_MAX_WIDTH 1200
#define TOUCH_DEFAULT_MAX_HEIGHT 2000
#define TOUCH_MAX_FINGER_NUM 10
#define TOUCH_FORCE_NUM 1000
//---for Pen---
#define PEN_PRESSURE_MAX (4095)
#define PEN_DISTANCE_MAX (1)
#define PEN_TILT_MIN (-60)
#define PEN_TILT_MAX (60)

/* Enable only when module have tp reset pin and connected to host */
#define NVT_TOUCH_SUPPORT_HW_RST 0

//---Customerized func.---
#define NVT_TOUCH_PROC 1
#define NVT_TOUCH_EXT_PROC 1
#define NVT_TOUCH_MP 1
#define MT_PROTOCOL_B 1
#define WAKEUP_GESTURE 1
#define NVT_WAKEUP_GESTURE_CUSTOMIZE 1
#if WAKEUP_GESTURE
extern const u16 gesture_key_array[];
#endif
#define BOOT_UPDATE_FIRMWARE 1
#define BOOT_UPDATE_FIRMWARE_NAME "novatek_ts_fw.bin"
#define MP_UPDATE_FIRMWARE_NAME   "novatek_ts_mp.bin"
#define POINT_DATA_CHECKSUM 1
#define POINT_DATA_CHECKSUM_LEN 65
#define CHECK_PEN_DATA_CHECKSUM 0

struct nvt_ts_data {
	struct spi_device *client;
	struct input_dev *input_dev;
	u16 addr;
	s8 phys[32];
	u8 fw_ver;
	bool suspended;
	u8 x_num;
	u8 y_num;
	u16 abs_x_max;
	u16 abs_y_max;
	u8 max_finger_num;
	u8 max_button_num;
	struct gpio_desc *reset_gpio;
	struct mutex lock;
	const struct nvt_ts_mem_map *mmap;
	u8 carrier_system;
	u8 hw_crc;
	u16 nvt_pid;
	u8 *rbuf;
	u8 *xbuf;
	struct mutex xbuf_lock;
	bool irq_enabled;
	u8 cascade;
	bool pen_support;
	bool wgp_stylus;
	u8 x_gang_num;
	u8 y_gang_num;
	struct input_dev *pen_input_dev;
	s8 pen_phys[32];
	bool gesture_support;
	struct touchscreen_properties prop;
	struct touchscreen_properties pen_prop;
	u32 swrst_n8_addr;
	u32 spi_rd_fast_addr;
	struct workqueue_struct *nvt_fwu_wq;
	struct delayed_work nvt_fwu_work;
};

#if NVT_TOUCH_PROC
struct nvt_flash_data{
	rwlock_t lock;
};
#endif

typedef enum {
	RESET_STATE_INIT = 0xA0,// IC reset
	RESET_STATE_REK,		// ReK baseline
	RESET_STATE_REK_FINISH,	// baseline is ready
	RESET_STATE_NORMAL_RUN,	// normal run
	RESET_STATE_MAX  = 0xAF
} RST_COMPLETE_STATE;

typedef enum {
    EVENT_MAP_HOST_CMD                      = 0x50,
    EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE   = 0x51,
    EVENT_MAP_RESET_COMPLETE                = 0x60,
    EVENT_MAP_FWINFO                        = 0x78,
    EVENT_MAP_PROJECTID                     = 0x9A,
} SPI_EVENT_MAP;

//---SPI READ/WRITE---
#define SPI_WRITE_MASK(a)	(a | 0x80)
#define SPI_READ_MASK(a)	(a & 0x7F)

#define DUMMY_BYTES (1)
#define NVT_TRANSFER_LEN	(63*1024)
#define NVT_READ_LEN		(2*1024)

typedef enum {
	NVTWRITE = 0,
	NVTREAD  = 1
} NVT_SPI_RW;

//---extern functions---
extern int CTP_SPI_READ(struct spi_device *client, u8 *buf, u16 len);
extern int CTP_SPI_WRITE(struct spi_device *client, u8 *buf, u16 len);
void nvt_bootloader_reset(struct nvt_ts_data *ts);
void nvt_eng_reset(struct nvt_ts_data *ts);
void nvt_sw_reset(struct nvt_ts_data *ts);
void nt36xxx_set_boot_ready(struct nvt_ts_data *ts);
void nt36xxx_enable_bl_crc(struct nvt_ts_data *ts);
void nt36xxx_enable_fw_crc(struct nvt_ts_data *ts);
int nvt_update_firmware(struct nvt_ts_data *ts, char *firmware_name);
int nvt_check_fw_reset_state(struct nvt_ts_data *ts, RST_COMPLETE_STATE check_reset_state);
int nvt_get_fw_info(struct nvt_ts_data *ts);
int nvt_clear_fw_status(struct nvt_ts_data *ts);
int nvt_check_fw_status(struct nvt_ts_data *ts);
int nvt_check_spi_dma_tx_info(struct nvt_ts_data *ts);
int nvt_set_addr(struct nvt_ts_data *ts, int addr);
int nvt_write_addr(struct nvt_ts_data *ts, int addr, u8 data);

#endif
