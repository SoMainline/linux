// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2010 - 2018 Novatek, Inc.
 * Copyright (c) 2024, Linaro Ltd.
 */
#ifndef __NT36XXX_SPI_H__
#define __NT36XXX_SPI_H__

#include <linux/input.h>
#include <linux/of.h>
#include <linux/spi/spi.h>
#include <linux/input/touchscreen.h>

#define NVT_LOG(fmt, args...)    pr_err("[%s] nt36xxx-spi %d: " fmt, __func__, __LINE__, ##args)
#define NVT_ERR(fmt, args...)    pr_err("[%s] nt36xxx-spi %d: " fmt, __func__, __LINE__, ##args)

#define TOUCH_DEFAULT_MAX_WIDTH		1200
#define TOUCH_DEFAULT_MAX_HEIGHT	2000
#define TOUCH_FORCE_NUM			1000
#define TOUCH_MAX_FINGER_NUM		10

#define BOOT_UPDATE_FIRMWARE 1
#define BOOT_UPDATE_FIRMWARE_NAME "novatek_ts_fw.bin"
#define MP_UPDATE_FIRMWARE_NAME   "novatek_ts_mp.bin"
#define POINT_DATA_CHECKSUM 1
#define POINT_DATA_CHECKSUM_LEN 65
#define CHECK_PEN_DATA_CHECKSUM 1

struct nvt_ts_data {
	struct spi_device *client;
	struct input_dev *input_dev;
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

/* Right out of IC reset */
#define RESET_STATE_INIT	0xA0
/* Firmware loaded */
#define RESET_STATE_REK		0xA1
#define RESET_STATE_MAX		0xAF

#define EVENT_MAP_HOST_CMD			0x50
#define EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE	0x51
#define EVENT_MAP_RESET_COMPLETE		0x60
#define EVENT_MAP_FWINFO			0x78
#define EVENT_MAP_PROJECTID			0x9A

#define DUMMY_BYTES		1
#define NVT_TRANSFER_LEN	(63 * SZ_1K)
#define NVT_READ_LEN		SZ_2K

#define NVT_ID_BYTE_MAX		6
/* 0xff seems to be unused as far as actual matching goes, use it as an "ignore" */
#define ID_MATCH_ANY		0xFF
struct nt36xxx_hw_id {
	const u8 bytes[NVT_ID_BYTE_MAX];
};

#define ADDR_WITHIN_PAGE(x) (x & GENMASK(6, 0))
struct nvt_ts_mem_map {
	u32 event_buf;

	/* Phase 2 Host Download */
	u32 boot_rdy_addr;
	u32 por_cd_addr;
	u32 tx_auto_copy_en;
	u32 spi_dma_tx_info;

	/* Bootloader CRC */
	u32 ilm_length_addr;
	u32 dlm_length_addr;
	u32 ilm_dest_addr;
	u32 dlm_dest_addr;
	u32 g_ilm_checksum_addr;
	u32 g_dlm_checksum_addr;
	u32 r_ilm_checksum_addr;
	u32 bld_crc_en_addr;
};

struct nt36xxx_match_data {
	const struct nt36xxx_hw_id *ids;
	int num_ids;
	const struct nvt_ts_mem_map *mmap;
	u8 hw_crc;
};

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

//---extern functions---
extern int nt36xxx_spi_read(struct spi_device *client, u8 *buf, u16 len);
extern int nt36xxx_spi_write(struct spi_device *client, u8 *buf, u16 len);
void nvt_bootloader_reset(struct nvt_ts_data *ts);
void nvt_eng_reset(struct nvt_ts_data *ts);
void nvt_sw_reset(struct nvt_ts_data *ts);
void nt36xxx_set_boot_ready(struct nvt_ts_data *ts);
void nt36xxx_enable_bl_crc(struct nvt_ts_data *ts);
void nt36xxx_enable_fw_crc(struct nvt_ts_data *ts);
int nvt_update_firmware(struct nvt_ts_data *ts, char *firmware_name);
int nvt_check_fw_reset_state(struct nvt_ts_data *ts, u8 desired_state);
int nvt_get_fw_info(struct nvt_ts_data *ts);
int nvt_clear_fw_status(struct nvt_ts_data *ts);
int nvt_check_fw_status(struct nvt_ts_data *ts);
int nvt_check_spi_dma_tx_info(struct nvt_ts_data *ts);
int nvt_set_page(struct nvt_ts_data *ts, int addr);
int nvt_write_addr(struct nvt_ts_data *ts, int addr, u8 data);

#endif
