// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2010 - 2018 Novatek, Inc.
 * Copyright (c) 2024, Linaro Ltd.
 */

#include <linux/delay.h>

#include "nt36xxx.h"

#define NT36XXX_SPI_WRITE		BIT(7)
static int spi_read_write(struct spi_device *client,
			  u8 *buf, size_t len,
			  bool read)
{
	struct nvt_ts_data *ts = spi_get_drvdata(client);
	int retries;
	int ret;

	for (retries = 5; retries > 0; retries--) {
		struct spi_transfer t = {
			.tx_buf = ts->xbuf,
			.len = len,
		};
		struct spi_message m = { 0 };

		if (read) {
			t.rx_buf = ts->rbuf;
			t.len += DUMMY_BYTES;
			buf[0] &= ~NT36XXX_SPI_WRITE;
		} else {
			buf[0] |= NT36XXX_SPI_WRITE;
		}

		memset(ts->xbuf, 0, len + DUMMY_BYTES);
		memcpy(ts->xbuf, buf, len);

		spi_message_init(&m);
		spi_message_add_tail(&t, &m);

		ret = spi_sync(client, &m);
		if (!ret)
			return 0;
	}

	return ret;
}

int CTP_SPI_READ(struct spi_device *client, u8 *buf, u16 len)
{
	struct nvt_ts_data *ts = spi_get_drvdata(client);
	int ret;

	guard(mutex)(&ts->xbuf_lock);

	ret = spi_read_write(client, buf, len, true);
	if (!ret)
		memcpy(buf + 1, ts->rbuf + 2, len - 1);

	return ret;
}
EXPORT_SYMBOL_GPL(CTP_SPI_READ);

int CTP_SPI_WRITE(struct spi_device *client, u8 *buf, u16 len)
{
	struct nvt_ts_data *ts = spi_get_drvdata(client);

	guard(mutex)(&ts->xbuf_lock);

	return spi_read_write(client, buf, len, false);
}

#define NVT_SET_ADDR_CMD		0xFF
int nvt_set_addr(struct nvt_ts_data *ts, int addr)
{
	u8 buf[4] = { 0 };

	buf[0] = NVT_SET_ADDR_CMD;
	buf[1] = (addr >> 15) & GENMASK(7, 0);
	buf[2] = (addr >> 7) & GENMASK(7, 0);

	return CTP_SPI_WRITE(ts->client, buf, 3);
}

int nvt_write_addr(struct nvt_ts_data *ts, int addr, u8 data)
{
	u8 buf[4] = { 0 };
	int ret;

	ret = nvt_set_addr(ts, addr);
	if (ret) {
		dev_err(&ts->client->dev, "Couldn't set page 0x%x\n", addr);
		return ret;
	}

	buf[0] = addr & GENMASK(6, 0);
	buf[1] = data;
	ret = CTP_SPI_WRITE(ts->client, buf, 2);
	if (ret)
		dev_err(&ts->client->dev, "Couldn't write 0x%x to 0x%x\n", data, addr);

	return ret;
}

void nt36xxx_enable_bl_crc(struct nvt_ts_data *ts)
{
	u8 buf[4] = { 0 };

	nvt_set_addr(ts, ts->mmap->bld_crc_en_addr);

	/* Read back the current value */
	buf[0] = ts->mmap->bld_crc_en_addr & GENMASK(6, 0);
	buf[1] = 0xFF;
	CTP_SPI_READ(ts->client, buf, 2);

	/* Set bit 7 */
	buf[0] = ts->mmap->bld_crc_en_addr & GENMASK(6, 0);
	buf[1] = buf[1] | BIT(7);
	CTP_SPI_WRITE(ts->client, buf, 2);
}

#define NT36XXX_EVENT_MAP_HOST_CMD_EN_FW_CRC		0xAE
void nt36xxx_enable_fw_crc(struct nvt_ts_data *ts)
{
	u8 buf[4] = { 0 };

	nvt_set_addr(ts, ts->mmap->event_buf);

	//---clear fw reset status---
	buf[0] = EVENT_MAP_RESET_COMPLETE & GENMASK(6, 0);
	buf[1] = 0x00;
	CTP_SPI_WRITE(ts->client, buf, 2);

	//---enable fw crc---
	buf[0] = EVENT_MAP_HOST_CMD & GENMASK(6, 0);
	buf[1] = NT36XXX_EVENT_MAP_HOST_CMD_EN_FW_CRC;
	CTP_SPI_WRITE(ts->client, buf, 2);
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
		nvt_set_addr(ts, ts->mmap->spi_dma_tx_info);

		buf[0] = ts->mmap->spi_dma_tx_info & GENMASK(6, 0);
		buf[1] = 0xFF;
		CTP_SPI_READ(ts->client, buf, 2);

		/* "Things are as expected", not very well documented */
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

#define CLEAR_FW_STATUS_MAX_RETRIES	20
int nvt_clear_fw_status(struct nvt_ts_data *ts)
{
	u8 buf[8] = { 0 };
	int i;

	for (i = 0; i < CLEAR_FW_STATUS_MAX_RETRIES; i++) {
		nvt_set_addr(ts, ts->mmap->event_buf | EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE);

		/* Clear firmware status */
		buf[0] = EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE;
		buf[1] = 0x00;
		CTP_SPI_WRITE(ts->client, buf, 2);

		/* Read it back */
		buf[0] = EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE;
		buf[1] = 0xFF;
		CTP_SPI_READ(ts->client, buf, 2);

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
		nvt_set_addr(ts, ts->mmap->event_buf | EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE);

		/* Read the firwmare status */
		buf[0] = EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE;
		buf[1] = 0x00;
		CTP_SPI_READ(ts->client, buf, 2);

		if ((buf[1] & 0xF0) == 0xA0)
			return 0;

		usleep_range(10000, 10000);
	}

	return -EIO;
}

int nvt_check_fw_reset_state(struct nvt_ts_data *ts, RST_COMPLETE_STATE check_reset_state)
{
	u8 buf[8] = { 0 };
	int ret = 0;
	int retry = 0;
	int retry_max = (check_reset_state == RESET_STATE_INIT) ? 10 : 50;

	nvt_set_addr(ts, ts->mmap->event_buf | EVENT_MAP_RESET_COMPLETE);

	while (1) {
		/* Read the reset state */
		buf[0] = EVENT_MAP_RESET_COMPLETE;
		buf[1] = 0x00;
		CTP_SPI_READ(ts->client, buf, 6);

		if ((buf[1] >= check_reset_state) && (buf[1] <= RESET_STATE_MAX)) {
			ret = 0;
			break;
		}

		retry++;
		if(unlikely(retry > retry_max)) {
			NVT_ERR("error, retry=%d, buf[1]=0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X\n",
				retry, buf[1], buf[2], buf[3], buf[4], buf[5]);
			return -1;
		}

		usleep_range(10000, 10000);
	}

	return ret;
}

// "read project ID" for fw binary matching?
static int nvt_read_pid(struct nvt_ts_data *ts)
{
	u8 buf[4] = { 0 };
	int ret = 0;

	//---set xdata index to EVENT BUF ADDR---
	nvt_set_addr(ts, ts->mmap->event_buf | EVENT_MAP_PROJECTID);

	//---read project id---
	buf[0] = EVENT_MAP_PROJECTID;
	buf[1] = 0x00;
	buf[2] = 0x00;
	CTP_SPI_READ(ts->client, buf, 3);

	ts->nvt_pid = (buf[2] << 8) + buf[1];

	//---set xdata index to EVENT BUF ADDR---
	nvt_set_addr(ts, ts->mmap->event_buf);

	NVT_LOG("PID=%04X\n", ts->nvt_pid);

	return ret;
}

int nvt_get_fw_info(struct nvt_ts_data *ts)
{
	u8 buf[64] = { 0 };
	int retries;
	int ret;

	ret = nvt_set_addr(ts, ts->mmap->event_buf | EVENT_MAP_FWINFO);
	if (ret)
		return ret;

	for (retries = 3; retries > 0; retries--) {
		buf[0] = EVENT_MAP_FWINFO;
		CTP_SPI_READ(ts->client, buf, 39);

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
