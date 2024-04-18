// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2010 - 2018 Novatek, Inc.
 * Copyright (c) 2024, Linaro Ltd.
 */

#include "nt36xxx.h"

#define ENG_RST_ADDR		0x7FFF80

static int spi_read_write(struct spi_device *client,
			  u8 *buf, size_t len,
			  NVT_SPI_RW rw)
{
	struct nvt_ts_data *ts = spi_get_drvdata(client);
	struct spi_transfer t = { .len = len };
	struct spi_message m;

	memset(ts->xbuf, 0, len + DUMMY_BYTES);
	memcpy(ts->xbuf, buf, len);

	switch (rw) {
		case NVTREAD:
			t.tx_buf = ts->xbuf;
			t.rx_buf = ts->rbuf;
			t.len = (len + DUMMY_BYTES);
			break;

		case NVTWRITE:
			t.tx_buf = ts->xbuf;
			break;
	}

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);

	return spi_sync(client, &m);
}

int CTP_SPI_READ(struct spi_device *client, u8 *buf, u16 len)
{
	struct nvt_ts_data *ts = spi_get_drvdata(client);
	int ret = -1;
	int retries = 0;

	guard(mutex)(&ts->xbuf_lock);

	buf[0] = SPI_READ_MASK(buf[0]);

	while (retries < 5) {
		ret = spi_read_write(client, buf, len, NVTREAD);
		if (ret == 0) break;
		retries++;
	}

	if (unlikely(retries == 5)) {
		NVT_ERR("read error, ret=%d\n", ret);
		ret = -EIO;
	} else {
		memcpy((buf+1), (ts->rbuf+2), (len-1));
	}

	return ret;
}
EXPORT_SYMBOL_GPL(CTP_SPI_READ);

#define SPI_WRITE_MAX_RETRIES		5
int CTP_SPI_WRITE(struct spi_device *client, u8 *buf, u16 len)
{
	struct nvt_ts_data *ts = spi_get_drvdata(client);
	int retries = 0;
	int ret = -1;

	guard(mutex)(&ts->xbuf_lock);

	buf[0] = SPI_WRITE_MASK(buf[0]);

	while (retries < SPI_WRITE_MAX_RETRIES && ret) {
		ret = spi_read_write(client, buf, len, NVTWRITE);
		retries++;
	}

	return ret;
}

/*******************************************************
Description:
	Novatek touchscreen set index/page/addr address.

return:
	Executive outcomes. 0---succeed. -5---access fail.
*******************************************************/
int nvt_set_page(struct nvt_ts_data *ts, int addr)
{
	u8 buf[4] = {0};

	buf[0] = 0xFF;	//set index/page/addr command
	buf[1] = (addr >> 15) & 0xFF;
	buf[2] = (addr >> 7) & 0xFF;

	return CTP_SPI_WRITE(ts->client, buf, 3);
}

/*******************************************************
Description:
	Novatek touchscreen write data to specify address.

return:
	Executive outcomes. 0---succeed. -5---access fail.
*******************************************************/
int nvt_write_addr(struct nvt_ts_data *ts, int addr, u8 data)
{
	int ret = 0;
	u8 buf[4] = {0};

	//---set xdata index---
	buf[0] = 0xFF;	//set index/page/addr command
	buf[1] = (addr >> 15) & 0xFF;
	buf[2] = (addr >> 7) & 0xFF;
	ret = CTP_SPI_WRITE(ts->client, buf, 3);
	if (ret) {
		NVT_ERR("set page 0x%06X failed, ret = %d\n", addr, ret);
		return ret;
	}

	//---write data to index---
	buf[0] = addr & GENMASK(6, 0);
	buf[1] = data;
	ret = CTP_SPI_WRITE(ts->client, buf, 2);
	if (ret) {
		NVT_ERR("write data to 0x%06X failed, ret = %d\n", addr, ret);
		return ret;
	}

	return ret;
}

/* Bootloader CRC? */
void nvt_bld_crc_enable(struct nvt_ts_data *ts)
{
	u8 buf[4] = { 0 };

	//---set xdata index to BLD_CRC_EN_ADDR---
	nvt_set_page(ts, ts->mmap->BLD_CRC_EN_ADDR);

	//---read data from index---
	buf[0] = ts->mmap->BLD_CRC_EN_ADDR & GENMASK(6, 0);
	buf[1] = 0xFF;
	CTP_SPI_READ(ts->client, buf, 2);

	//---write data to index---
	buf[0] = ts->mmap->BLD_CRC_EN_ADDR & GENMASK(6, 0);
	buf[1] = buf[1] | (0x01 << 7);
	CTP_SPI_WRITE(ts->client, buf, 2);
}

/* Firmware CRC? */
void nvt_fw_crc_enable(struct nvt_ts_data *ts)
{
	u8 buf[4] = { 0 };

	//---set xdata index to EVENT BUF ADDR---
	nvt_set_page(ts, ts->mmap->EVENT_BUF_ADDR);

	//---clear fw reset status---
	buf[0] = EVENT_MAP_RESET_COMPLETE & GENMASK(6, 0);
	buf[1] = 0x00;
	CTP_SPI_WRITE(ts->client, buf, 2);

	//---enable fw crc---
	buf[0] = EVENT_MAP_HOST_CMD & GENMASK(6, 0);
	buf[1] = 0xAE;	//enable fw crc command
	CTP_SPI_WRITE(ts->client, buf, 2);
}

/* Set "boot ready" flag after flashing the firmware */
void nvt_boot_ready(struct nvt_ts_data *ts)
{
	//---write BOOT_RDY status cmds---
	nvt_write_addr(ts, ts->mmap->BOOT_RDY_ADDR, 1);

	mdelay(5);

	if (!ts->hw_crc) {
		//---write BOOT_RDY status cmds---
		nvt_write_addr(ts, ts->mmap->BOOT_RDY_ADDR, 0);

		//---write POR_CD cmds---
		nvt_write_addr(ts, ts->mmap->POR_CD_ADDR, 0xA0);
	}
}

/*******************************************************
Description:
	Novatek touchscreen check spi dma tx info function.

return:
	N/A.
*******************************************************/
#define SPI_DMA_TX_INFO_MAX_RETRIES		200
int nvt_check_spi_dma_tx_info(struct nvt_ts_data *ts)
{
	u8 buf[8] = { 0 };
	int i;

	for (i = 0; i < SPI_DMA_TX_INFO_MAX_RETRIES; i++) {
		//---set xdata index to EVENT BUF ADDR---
		nvt_set_page(ts, ts->mmap->SPI_DMA_TX_INFO);

		//---read fw status---
		buf[0] = ts->mmap->SPI_DMA_TX_INFO & GENMASK(6, 0);
		buf[1] = 0xFF;
		CTP_SPI_READ(ts->client, buf, 2);

		if (buf[1] == 0x00)
			break;

		usleep_range(1000, 1010);
	}

	return i == SPI_DMA_TX_INFO_MAX_RETRIES;
}

/*******************************************************
Description:
	Novatek touchscreen eng reset cmd
    function.

return:
	n.a.
*******************************************************/
void nvt_eng_reset(struct nvt_ts_data *ts)
{
	nvt_write_addr(ts, ENG_RST_ADDR, 0x5A);
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
	//---reset cmds to SWRST_N8_ADDR---
	nvt_write_addr(ts, ts->swrst_n8_addr, 0x69);
	mdelay(5);

	/* Disable SPI_RD_FAST if present */
	if (ts->spi_rd_fast_addr)
		nvt_write_addr(ts, ts->spi_rd_fast_addr, 0);
}
EXPORT_SYMBOL_GPL(nvt_bootloader_reset);

/*******************************************************
Description:
	Novatek touchscreen clear FW status function.

return:
	Executive outcomes. 0---succeed. -1---fail.
*******************************************************/
#define CLEAR_FW_STATUS_MAX_RETRIES	20
int nvt_clear_fw_status(struct nvt_ts_data *ts)
{
	u8 buf[8] = { 0 };
	int i;

	for (i = 0; i < CLEAR_FW_STATUS_MAX_RETRIES; i++) {
		//---set xdata index to EVENT BUF ADDR---
		nvt_set_page(ts, ts->mmap->EVENT_BUF_ADDR | EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE);

		//---clear fw status---
		buf[0] = EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE;
		buf[1] = 0x00;
		CTP_SPI_WRITE(ts->client, buf, 2);

		//---read fw status---
		buf[0] = EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE;
		buf[1] = 0xFF;
		CTP_SPI_READ(ts->client, buf, 2);

		if (buf[1] == 0x00)
			break;

		usleep_range(10000, 10000);
	}

	return i == CLEAR_FW_STATUS_MAX_RETRIES;
}

/*******************************************************
Description:
	Novatek touchscreen check FW status function.

return:
	Executive outcomes. 0---succeed. -1---failed.
*******************************************************/
#define CHECK_FW_STATUS_MAX_RETRIES	20
int nvt_check_fw_status(struct nvt_ts_data *ts)
{
	u8 buf[8] = { 0 };
	int i = 0;

	for (i = 0; i < CHECK_FW_STATUS_MAX_RETRIES; i++) {
		//---set xdata index to EVENT BUF ADDR---
		nvt_set_page(ts, ts->mmap->EVENT_BUF_ADDR | EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE);

		//---read fw status---
		buf[0] = EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE;
		buf[1] = 0x00;
		CTP_SPI_READ(ts->client, buf, 2);

		if ((buf[1] & 0xF0) == 0xA0)
			break;

		usleep_range(10000, 10000);
	}

	return i == CHECK_FW_STATUS_MAX_RETRIES;
}

/*******************************************************
Description:
	Novatek touchscreen check FW reset state function.

return:
	Executive outcomes. 0---succeed. -1---failed.
*******************************************************/
int nvt_check_fw_reset_state(struct nvt_ts_data *ts, RST_COMPLETE_STATE check_reset_state)
{
	u8 buf[8] = { 0 };
	int ret = 0;
	int retry = 0;
	int retry_max = (check_reset_state == RESET_STATE_INIT) ? 10 : 50;

	//---set xdata index to EVENT BUF ADDR---
	nvt_set_page(ts, ts->mmap->EVENT_BUF_ADDR | EVENT_MAP_RESET_COMPLETE);

	while (1) {
		//---read reset state---
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
			ret = -1;
			break;
		}

		usleep_range(10000, 10000);
	}

	return ret;
}

/*******************************************************
Description:
	Novatek touchscreen get novatek project id information
	function.

return:
	Executive outcomes. 0---success. -1---fail.
*******************************************************/
static int nvt_read_pid(struct nvt_ts_data *ts)
{
	u8 buf[4] = { 0 };
	int ret = 0;

	//---set xdata index to EVENT BUF ADDR---
	nvt_set_page(ts, ts->mmap->EVENT_BUF_ADDR | EVENT_MAP_PROJECTID);

	//---read project id---
	buf[0] = EVENT_MAP_PROJECTID;
	buf[1] = 0x00;
	buf[2] = 0x00;
	CTP_SPI_READ(ts->client, buf, 3);

	ts->nvt_pid = (buf[2] << 8) + buf[1];

	//---set xdata index to EVENT BUF ADDR---
	nvt_set_page(ts, ts->mmap->EVENT_BUF_ADDR);

	NVT_LOG("PID=%04X\n", ts->nvt_pid);

	return ret;
}

/*******************************************************
Description:
	Novatek touchscreen get firmware related information
	function.

return:
	Executive outcomes. 0---success. -1---fail.
*******************************************************/
int nvt_get_fw_info(struct nvt_ts_data *ts)
{
	u8 buf[64] = {0};
	int retries;
	int ret;

	ret = nvt_set_page(ts, ts->mmap->EVENT_BUF_ADDR | EVENT_MAP_FWINFO);
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
