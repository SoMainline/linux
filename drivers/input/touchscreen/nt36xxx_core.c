// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2010 - 2018 Novatek, Inc.
 * Copyright (c) 2024, Linaro Ltd.
 */

#include <linux/delay.h>

#include "nt36xxx.h"

int nt36xxx_spi_read(struct spi_device *client, u8 *buf, u16 len)
{
	struct nvt_ts_data *ts = spi_get_drvdata(client);
	struct spi_message m = { 0 };
	struct spi_transfer t = {
		.tx_buf = ts->xbuf,
		.rx_buf = ts->rbuf,
		.len = len + DUMMY_BYTES,
	};
	int ret;

	guard(mutex)(&ts->xbuf_lock);

	buf[0] &= ~NT36XXX_SPI_WRITE;

	memset(ts->xbuf, 0, len + DUMMY_BYTES);
	memcpy(ts->xbuf, buf, len);

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);

	ret = spi_sync(client, &m);
	if (!ret)
		memcpy(buf + 1, ts->rbuf + 2, len - 1);

	return ret;
}
EXPORT_SYMBOL_GPL(nt36xxx_spi_read);

int nt36xxx_spi_write(struct spi_device *client, u8 *buf, u16 len)
{
	struct nvt_ts_data *ts = spi_get_drvdata(client);
	struct spi_message m = { 0 };
	struct spi_transfer t = {
		.tx_buf = ts->xbuf,
		.len = len,
	};

	guard(mutex)(&ts->xbuf_lock);

	buf[0] |= NT36XXX_SPI_WRITE;

	memset(ts->xbuf, 0, len + DUMMY_BYTES);
	memcpy(ts->xbuf, buf, len);

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);

	return spi_sync(client, &m);
}

int nvt_set_page(struct nvt_ts_data *ts, int addr)
{
	u8 buf[4] = { 0 };

	buf[0] = NVT_SET_PAGE_CMD;
	buf[1] = (addr >> 15) & GENMASK(7, 0);
	buf[2] = (addr >> 7) & GENMASK(7, 0);

	return nt36xxx_spi_write(ts->client, buf, 3);
}

int nvt_write_addr(struct nvt_ts_data *ts, int addr, u8 data)
{
	u8 buf[4] = { 0 };
	int ret;

	ret = nvt_set_page(ts, addr);
	if (ret) {
		dev_err(&ts->client->dev, "Couldn't set page 0x%x\n", addr);
		return ret;
	}

	buf[0] = ADDR_WITHIN_PAGE(addr);
	buf[1] = data;
	ret = nt36xxx_spi_write(ts->client, buf, 2);
	if (ret)
		dev_err(&ts->client->dev, "Couldn't write 0x%x to 0x%x\n", data, addr);

	return ret;
}

void nt36xxx_enable_bl_crc(struct nvt_ts_data *ts)
{
	u8 buf[4] = { 0 };

	nvt_set_page(ts, ts->mmap->bld_crc_en_addr);

	/* Read back the current value */
	buf[0] = ADDR_WITHIN_PAGE(ts->mmap->bld_crc_en_addr);
	buf[1] = 0xFF;
	nt36xxx_spi_read(ts->client, buf, 2);

	/* Set bit 7 */
	buf[0] = ADDR_WITHIN_PAGE(ts->mmap->bld_crc_en_addr);
	buf[1] = buf[1] | BIT(7);
	nt36xxx_spi_write(ts->client, buf, 2);
}

#define NT36XXX_EVENT_MAP_HOST_CMD_EN_FW_CRC	0xAE
void nt36xxx_enable_fw_crc(struct nvt_ts_data *ts)
{
	u8 buf[4] = { 0 };

	nvt_set_page(ts, ts->mmap->event_buf);

	/* Clear the firmware reset status first */
	buf[0] = EVENT_MAP_RESET_COMPLETE;
	buf[1] = 0x00;
	nt36xxx_spi_write(ts->client, buf, 2);

	/* Enable firmware CRC */
	buf[0] = EVENT_MAP_HOST_CMD;
	buf[1] = NT36XXX_EVENT_MAP_HOST_CMD_EN_FW_CRC;
	nt36xxx_spi_write(ts->client, buf, 2);
}

/* Set "boot ready" flag after flashing the firmware */
void nt36xxx_set_boot_ready(struct nvt_ts_data *ts)
{
	nvt_write_addr(ts, ts->mmap->boot_rdy_addr, 1);

	mdelay(5);

	if (!ts->hw_crc) {
		nvt_write_addr(ts, ts->mmap->boot_rdy_addr, 0);
		nvt_write_addr(ts, ts->mmap->por_cd_addr, 0xa0);
	}
}

#define SPI_DMA_TX_INFO_MAX_RETRIES		200
int nvt_check_spi_dma_tx_info(struct nvt_ts_data *ts)
{
	u8 buf[8] = { 0 };
	int i;

	for (i = 0; i < SPI_DMA_TX_INFO_MAX_RETRIES; i++) {
		nvt_set_page(ts, ts->mmap->spi_dma_tx_info);

		buf[0] = ADDR_WITHIN_PAGE(ts->mmap->spi_dma_tx_info);
		buf[1] = 0xFF;
		nt36xxx_spi_read(ts->client, buf, 2);

		/* "Things are as expected", this is not very well explained */
		if (!buf[1])
			return 0;

		usleep_range(1000, 1010);
	}

	return -EIO;
}

#define ENG_RST_ADDR		0x7FFF80
void nvt_eng_reset(struct nvt_ts_data *ts)
{
	nvt_write_addr(ts, ENG_RST_ADDR, 0x5a);
	mdelay(1);
}
EXPORT_SYMBOL_GPL(nvt_eng_reset);

/* Software-reset the MCU (TODO: passing 0xAA resets and enters "idle mode") */
void nvt_sw_reset(struct nvt_ts_data *ts)
{
	nvt_write_addr(ts, ts->swrst_n8_addr, 0x55);
	msleep(10);
}
EXPORT_SYMBOL_GPL(nvt_sw_reset);

/* Reset the MCU into FW download mode */
void nvt_bootloader_reset(struct nvt_ts_data *ts)
{
	nvt_write_addr(ts, ts->swrst_n8_addr, 0x69);
	mdelay(5);

	/* Disable SPI_RD_FAST if present */
	if (ts->spi_rd_fast_addr)
		nvt_write_addr(ts, ts->spi_rd_fast_addr, 0);
}
EXPORT_SYMBOL_GPL(nvt_bootloader_reset);

#define CLEAR_FW_STATUS_MAX_RETRIES		20
int nvt_clear_fw_status(struct nvt_ts_data *ts)
{
	u8 buf[8] = { 0 };
	int i;

	for (i = 0; i < CLEAR_FW_STATUS_MAX_RETRIES; i++) {
		nvt_set_page(ts, ts->mmap->event_buf | EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE);

		/* Clear firmware status */
		buf[0] = EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE;
		buf[1] = 0x00;
		nt36xxx_spi_write(ts->client, buf, 2);

		/* Read it back */
		buf[0] = EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE;
		buf[1] = 0xFF;
		nt36xxx_spi_read(ts->client, buf, 2);

		/* Keep retrying until the write has been committed */
		if (!buf[1])
			return 0;

		usleep_range(10000, 10000);
	}

	return -EIO;
}

#define CHECK_FW_STATUS_MAX_RETRIES		20
int nvt_check_fw_status(struct nvt_ts_data *ts)
{
	u8 buf[8] = { 0 };
	int i = 0;

	for (i = 0; i < CHECK_FW_STATUS_MAX_RETRIES; i++) {
		nvt_set_page(ts, ts->mmap->event_buf | EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE);

		/* Read the firwmare status */
		buf[0] = EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE;
		buf[1] = 0x00;
		nt36xxx_spi_read(ts->client, buf, 2);

		if ((buf[1] & 0xF0) == 0xA0)
			return 0;

		usleep_range(10000, 10000);
	}

	return -EIO;
}

int nvt_check_fw_reset_state(struct nvt_ts_data *ts, u8 desired_state)
{
	int retries = desired_state == RESET_STATE_INIT ? 10 : 50;
	u8 buf[8] = { 0 };

	nvt_set_page(ts, ts->mmap->event_buf | EVENT_MAP_RESET_COMPLETE);

	for (; retries > 0; retries--) {
		/* Read the reset state */
		buf[0] = EVENT_MAP_RESET_COMPLETE;
		buf[1] = 0x00;
		nt36xxx_spi_read(ts->client, buf, 6);

		if (buf[1] >= desired_state && buf[1] <= RESET_STATE_MAX)
			return 0;

		usleep_range(10000, 10000);
	}

	return -ETIMEDOUT;
}

// "read project ID" for fw binary matching?
static int nvt_read_pid(struct nvt_ts_data *ts)
{
	u8 buf[4] = { 0 };
	int ret = 0;

	//---set xdata index to EVENT BUF ADDR---
	nvt_set_page(ts, ts->mmap->event_buf | EVENT_MAP_PROJECTID);

	//---read project id---
	buf[0] = EVENT_MAP_PROJECTID;
	buf[1] = 0x00;
	buf[2] = 0x00;
	nt36xxx_spi_read(ts->client, buf, 3);

	ts->nvt_pid = (buf[2] << 8) + buf[1];

	//---set xdata index to EVENT BUF ADDR---
	nvt_set_page(ts, ts->mmap->event_buf);

	NVT_LOG("PID=%04X\n", ts->nvt_pid);

	return ret;
}

int nvt_get_fw_info(struct nvt_ts_data *ts)
{
	u8 buf[64] = { 0 };
	int retries;
	int ret;

	ret = nvt_set_page(ts, ts->mmap->event_buf | EVENT_MAP_FWINFO);
	if (ret)
		return ret;

	for (retries = 3; retries > 0; retries--) {
		buf[0] = EVENT_MAP_FWINFO;
		nt36xxx_spi_read(ts->client, buf, 39);

		/* buf[2] should hold a NOT-ed buf[1], otherwise assume a broken state */
		if (~buf[1] == buf[2])
			break;

		if (retries == 0)
			return -EINVAL;
	}

	ts->fw_ver = buf[1];
	ts->x_num = buf[3];
	ts->y_num = buf[4];
	ts->abs_x_max = (u16)((buf[5] << 8) | buf[6]);
	ts->abs_y_max = (u16)((buf[7] << 8) | buf[8]);
	ts->max_button_num = buf[11];
	ts->cascade = buf[34] & BIT(0);
	if (ts->pen_support) {
		ts->x_gang_num = buf[37];
		ts->y_gang_num = buf[38];
	}

	dev_info(&ts->client->dev, "Firmware info: version = 0x%x, type = 0x%x\n", ts->fw_ver, buf[14]);

	return nvt_read_pid(ts);
}
