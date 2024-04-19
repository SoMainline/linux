// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2010 - 2018 Novatek, Inc.
 * Copyright (c) 2024, Linaro Ltd.
 */

#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/gpio.h>

#include "nt36xxx.h"

#define FLASH_SECTOR_SIZE		SZ_4K

#define FW_BIN_VER_OFFSET		(fw_need_write_size - SZ_4K)
#define FW_BIN_VER_BAR_OFFSET		(FW_BIN_VER_OFFSET + 1)


const struct firmware *fw_entry = NULL;
static size_t fw_need_write_size = 0;
static u8 *fwbuf = NULL;

struct nvt_ts_bin_map {
	__le32 SRAM_addr;
	__le32 size;
	__le32 BIN_addr;
	__le32 crc;
} __packed;

static struct nvt_ts_bin_map *bin_map;

static int nvt_download_init(struct nvt_ts_data *ts)
{
	if (fwbuf)
		return 0;

	fwbuf = kzalloc((NVT_TRANSFER_LEN + 1 + DUMMY_BYTES), GFP_KERNEL);
	if (!fwbuf)
		return -ENOMEM;

	return 0;
}

static u32 CheckSum(const u8 *data, size_t len)
{
	u32 checksum = 0;
	u32 i;

	for (i = 0; i <= len; i++)
		checksum += data[i];

	checksum += len;
	checksum = ~checksum +1;

	return checksum;
}

enum fw_partition_indices {
	PART_IDX_ILM,
	PART_IDX_DLM,
	PART_IDX_OTHER,
};

static u32 num_parts = 0;
static u8 cascade_2nd_header_info = 0;
static int nvt_bin_header_parser(struct nvt_ts_data *ts, const u8 *fwdata, size_t fwsize)
{
	struct device *dev = &ts->client->dev;
	bool overlay_present = false;
	u8 num_overlay_sections = 0;
	bool bin_hdr_found = false;
	u8 num_info_sections = 0;
	u32 header_end;
	u32 part_idx;
	u32 pos;

	/* Find the header size */
	header_end = ((__le32 *)fwdata)[0];

	/* check cascade next header */
	cascade_2nd_header_info = !!(fwdata[0x20] & BIT(1));
	NVT_LOG("cascade_2nd_header_info = %d\n", cascade_2nd_header_info);

	/* Info section start */
	pos = 0x30;
	if (cascade_2nd_header_info) {
		while (pos < (header_end / 2)) {
			num_info_sections++;
			pos += 0x10;
		}

		num_info_sections++;
	} else {
		while (pos < header_end) {
			num_info_sections++;
			pos += 0x10;
		}
	}

	overlay_present = FIELD_GET(BIT(4), fwdata[0x28]);
	if (overlay_present)
		num_overlay_sections = FIELD_GET(GENMASK(3, 0), fwdata[0x28]);

	/*
	 * Count the total number of partitions
	 * 1 (ILM) + 1 (DLM) + num_overlay_sections + num_info_sections
	 */
	num_parts = 2 + num_overlay_sections + num_info_sections;
	NVT_LOG("overlay_present = %d, num_overlay_sections = %d, num_info_sections = %d, num_parts = %d\n",
		overlay_present, num_overlay_sections, num_info_sections, num_parts);

	bin_map = kcalloc(num_parts + 1, sizeof(struct nvt_ts_bin_map), GFP_KERNEL);
	if (!bin_map)
		return -ENOMEM;

	bin_map[PART_IDX_ILM].BIN_addr = header_end;
	bin_map[PART_IDX_ILM].SRAM_addr = ((__le32 *)fwdata)[0x4];
	bin_map[PART_IDX_ILM].size = ((__le32 *)fwdata)[0x8];
	bin_map[PART_IDX_ILM].crc = ((__le32 *)fwdata)[0x18];

	bin_map[PART_IDX_DLM].BIN_addr = ((__le32 *)fwdata)[0xc];
	bin_map[PART_IDX_DLM].SRAM_addr = ((__le32 *)fwdata)[0x10];
	bin_map[PART_IDX_DLM].size = ((__le32 *)fwdata)[0x14];
	bin_map[PART_IDX_DLM].crc = ((__le32 *)fwdata)[0x1c];

	for (part_idx = PART_IDX_OTHER; part_idx < num_parts; part_idx++) {
		if (part_idx < PART_IDX_OTHER + num_info_sections) {
			if (!bin_hdr_found) {
				/* others partition located at 0x30 offset */
				pos = 0x30 + (0x10 * (part_idx - PART_IDX_OTHER));
			} else if (bin_hdr_found && cascade_2nd_header_info) {
				/* cascade 2nd header info */
				pos = header_end - 0x10;
			}

			memcpy(&bin_map[part_idx], &fwdata[pos], sizeof(struct nvt_ts_bin_map));

			/* ... */
			if (bin_map[part_idx].BIN_addr < header_end && bin_map[part_idx].size)
				bin_hdr_found = true;
		} else {
			/* overlay info located at DLM (list = 1) start addr */
			pos = bin_map[PART_IDX_DLM].BIN_addr + (0x10 * (part_idx - PART_IDX_OTHER - num_info_sections));

			memcpy(&bin_map[part_idx], &fwdata[pos], sizeof(struct nvt_ts_bin_map));
		}

		if (bin_map[part_idx].BIN_addr + bin_map[part_idx].size > fwsize) {
			dev_err(dev, "Access range [0x%8x-0x%8x] goes out of firwmare bounds\n",
				bin_map[part_idx].BIN_addr, bin_map[part_idx].BIN_addr + bin_map[part_idx].size);
			return -EINVAL;
		}

		if (!ts->hw_crc)
			bin_map[part_idx].crc = CheckSum(&fwdata[bin_map[part_idx].BIN_addr], bin_map[part_idx].size);
	}

	return 0;
}

static void update_firmware_release(void)
{
	if (fw_entry)
		release_firmware(fw_entry);

	fw_entry = NULL;
}

static int update_firmware_request(struct nvt_ts_data *ts, char *filename)
{
	struct device *dev = &ts->client->dev;
	int ret = 0;

	if (!filename)
		return -ENOENT;

	while (1) {
		NVT_LOG("filename is %s\n", filename);

		ret = request_firmware(&fw_entry, filename, dev);
		if (ret) {
			NVT_ERR("firmware load failed, ret=%d\n", ret);
			return ret;
		}

		// check if FW version add FW version bar equals 0xFF
		if (*(fw_entry->data + FW_BIN_VER_OFFSET) + *(fw_entry->data + FW_BIN_VER_BAR_OFFSET) != 0xFF) {
			NVT_ERR("bin file FW_VER + FW_VER_BAR should be 0xFF!\n");
			NVT_ERR("FW_VER=0x%02X, FW_VER_BAR=0x%02X\n", *(fw_entry->data + FW_BIN_VER_OFFSET), *(fw_entry->data + FW_BIN_VER_BAR_OFFSET));
			ret = -ENOEXEC;
			goto invalid;
		}

		/* BIN Header Parser */
		ret = nvt_bin_header_parser(ts, fw_entry->data, fw_entry->size);
		if (!ret)
			return 0;

		NVT_ERR("bin header parser failed\n");

invalid:
		update_firmware_release();
		if (!IS_ERR_OR_NULL(bin_map)) {
			kfree(bin_map);
			bin_map = NULL;
		}
	}

	return ret;
}

static int nvt_write_sram(struct nvt_ts_data *ts,
			  const u8 *fwdata,
			  u32 SRAM_addr,
			  u32 size,
			  u32 BIN_addr)
{
	struct device *dev = &ts->client->dev;
	int ret = 0;
	u32 i = 0;
	u16 len = 0;
	int count = 0;

	if (size % NVT_TRANSFER_LEN)
		count = (size / NVT_TRANSFER_LEN) + 1;
	else
		count = (size / NVT_TRANSFER_LEN);

	pr_err("foobarbaz = %d\n", count);
	for (i = 0; i < count; i++) {
		len = min(size, NVT_TRANSFER_LEN);

		ret = nvt_set_page(ts, SRAM_addr);
		if (ret) {
			NVT_ERR("set page failed, ret = %d\n", ret);
			return ret;
		}

		fwbuf[0] = ADDR_WITHIN_PAGE(SRAM_addr);
		memcpy(fwbuf + 1, &fwdata[BIN_addr], len);

		ret = nt36xxx_spi_write(ts->client, fwbuf, len + 1);
		if (ret) {
			dev_err(dev, "Couldn't write firmware to SRAM: %d\n", ret);
			return ret;
		}

		SRAM_addr += NVT_TRANSFER_LEN;
		BIN_addr += NVT_TRANSFER_LEN;
		size -= NVT_TRANSFER_LEN;
	}

	return 0;
}

/*******************************************************
Description:
	Novatek touchscreen nvt_write_firmware function to write
firmware into each partition.

return:
	n.a.
*******************************************************/
static int nvt_write_firmware(struct nvt_ts_data *ts, const u8 *fwdata, size_t fwsize)
{
	u32 BIN_addr, SRAM_addr, size;
	u32 list = 0;
	int ret;

	memset(fwbuf, 0, (NVT_TRANSFER_LEN + 1));

	for (list = 0; list < num_parts; list++) {
		/* initialize variable */
		SRAM_addr = bin_map[list].SRAM_addr;
		size = bin_map[list].size;
		BIN_addr = bin_map[list].BIN_addr;

		/* Check data size */
		if ((BIN_addr + size) > fwsize) {
			NVT_ERR("access range (0x%08X to 0x%08X) is larger than bin size!\n",
					BIN_addr, BIN_addr + size);
			return -EINVAL;
		}

		/* ignore reserved partition (Reserved Partition size is zero) */
		if (!size)
			continue;
		else
			size = size +1;

		/* write data to SRAM */
		ret = nvt_write_sram(ts, fwdata, SRAM_addr, size, BIN_addr);
		if (ret) {
			NVT_ERR("sram program failed, ret = %d\n", ret);
			return ret;
		}
	}

	return 0;
}

/*******************************************************
Description:
	Novatek touchscreen check checksum function.
This function will compare file checksum and fw checksum.

return:
	n.a.
*******************************************************/
static int nvt_check_fw_checksum(struct nvt_ts_data *ts)
{
	u32 len = num_parts * 4;
	u32 fw_checksum;
	u32 part_idx;
	int ret;

	memset(fwbuf, 0, (len + 1));

	//---set xdata index to checksum---
	nvt_set_page(ts, ts->mmap->r_ilm_checksum_addr);

	/* read checksum */
	fwbuf[0] = (ts->mmap->r_ilm_checksum_addr) & 0x7F;
	ret = nt36xxx_spi_read(ts->client, fwbuf, len+1);
	if (ret) {
		NVT_ERR("Read fw checksum failed\n");
		return ret;
	}

	/*
	 * Compare each checksum from fw
	 * ILM + DLM + Overlay + Info
	 */
	for (part_idx = 0; part_idx < num_parts; part_idx++) {
		fw_checksum = ((__le32 *)fwbuf)[4 * part_idx + 1];

		/* ignore reserved partition (Reserved Partition size is zero) */
		if (!bin_map[part_idx].size)
			continue;

		if (bin_map[part_idx].crc != fw_checksum) {
			NVT_ERR("[%d] BIN_checksum=0x%08X, FW_checksum=0x%08X\n",
				part_idx, bin_map[part_idx].crc, fw_checksum);
			ret = -EIO;
		}
	}

	return ret;
}

static void nt36xxx_set_bl_crc_bank(struct nvt_ts_data *ts, u8 bank_idx)
{
	u32 DEST_ADDR = bank_idx == 0 ? ts->mmap->ilm_dest_addr : ts->mmap->dlm_dest_addr;
	u32 G_CHECKSUM_ADDR = bank_idx == 0 ? ts->mmap->g_ilm_checksum_addr : ts->mmap->g_dlm_checksum_addr;
	u32 LENGTH_ADDR = bank_idx == 0 ? ts->mmap->ilm_length_addr : ts->mmap->dlm_length_addr;
	u32 sram_addr = bin_map[bank_idx].SRAM_addr;
	u32 size = bin_map[bank_idx].size;
	u32 crc = bin_map[bank_idx].crc;

	nvt_set_page(ts, DEST_ADDR);
	fwbuf[0] = ADDR_WITHIN_PAGE(DEST_ADDR);
	fwbuf[1] = sram_addr & GENMASK(7, 0);
	fwbuf[2] = (sram_addr >> 8) & GENMASK(7, 0);
	fwbuf[3] = (sram_addr >> 16) & GENMASK(7, 0);
	nt36xxx_spi_write(ts->client, fwbuf, 4);

	fwbuf[0] = ADDR_WITHIN_PAGE(LENGTH_ADDR);
	fwbuf[1] = size & GENMASK(7, 0);
	fwbuf[2] = (size >> 8) & GENMASK(7, 0);
	fwbuf[3] = (size >> 16) & BIT(0);
	nt36xxx_spi_write(ts->client, fwbuf, ts->hw_crc > 1 ? 4 : 3);

	fwbuf[0] = ADDR_WITHIN_PAGE(G_CHECKSUM_ADDR);
	fwbuf[1] = crc & GENMASK(7, 0);
	fwbuf[2] = (crc >> 8) & GENMASK(7, 0);
	fwbuf[3] = (crc >> 16) & GENMASK(7, 0);
	fwbuf[4] = (crc >> 24) & GENMASK(7, 0);
	nt36xxx_spi_write(ts->client, fwbuf, 5);
}

static int nvt_download_firmware_hw_crc(struct nvt_ts_data *ts)
{
	int ret;

	/* bootloader reset to reset MCU */
	nvt_bootloader_reset(ts);

	/* Set ILM & DLM register banks for bootloader HW CRC */
	nt36xxx_set_bl_crc_bank(ts, 0);
	nt36xxx_set_bl_crc_bank(ts, 1);

	if (cascade_2nd_header_info)
		nvt_write_addr(ts, ts->mmap->tx_auto_copy_en, 0x69);

	ret = nvt_write_firmware(ts, fw_entry->data, fw_entry->size);
	if (ret) {
		NVT_ERR("Write_Firmware failed. (%d)\n", ret);
		return ret;
	}

	if (cascade_2nd_header_info) {
		ret = nvt_check_spi_dma_tx_info(ts);
		if (ret) {
			NVT_ERR("spi dma tx info failed. (%d)\n", ret);
			return ret;
		}
	}

	nt36xxx_enable_bl_crc(ts);

	nt36xxx_enable_fw_crc(ts);

	nt36xxx_set_boot_ready(ts);

	return nvt_check_fw_reset_state(ts, RESET_STATE_INIT);
}

static int nvt_download_firmware(struct nvt_ts_data *ts)
{
	int ret;

	gpiod_set_value_cansleep(ts->reset_gpio, 1);
	mdelay(1);

	nvt_eng_reset(ts);

	gpiod_set_value_cansleep(ts->reset_gpio, 0);
	mdelay(10);

	nvt_bootloader_reset(ts);

	/* Clear FW reset status */
	nvt_write_addr(ts, ts->mmap->event_buf | EVENT_MAP_RESET_COMPLETE, 0);

	ret = nvt_write_firmware(ts, fw_entry->data, fw_entry->size);
	if (ret) {
		NVT_ERR("Write_Firmware failed. (%d)\n", ret);
		return ret;
	}

	nt36xxx_set_boot_ready(ts);

	ret = nvt_check_fw_reset_state(ts, RESET_STATE_INIT);
	if (ret) {
		NVT_ERR("nvt_check_fw_reset_state failed. (%d)\n", ret);
		return ret;
	}

	/* check fw checksum result */
	return nvt_check_fw_checksum(ts);
}

int nvt_update_firmware(struct nvt_ts_data *ts, char *firmware_name)
{
	struct device *dev = &ts->client->dev;
	int ret;

	ret = update_firmware_request(ts, firmware_name);
	if (ret) {
		NVT_ERR("update_firmware_request failed. (%d)\n", ret);
		goto request_firmware_fail;
	}

	/* initial buffer and variable */
	ret = nvt_download_init(ts);
	if (ret) {
		NVT_ERR("Download Init failed. (%d)\n", ret);
		goto download_fail;
	}

	/* HW CRC-enabled chips require a slightly different procedure */
	if (ts->hw_crc)
		ret = nvt_download_firmware_hw_crc(ts);
	else
		ret = nvt_download_firmware(ts);
	if (ret) {
		pr_err("Firmware download failed: %d\n", ret);
		goto download_fail;
	}

	/* Get FW Info */
	ret = nvt_get_fw_info(ts);
	if (ret)
		dev_err(dev, "Couldn't get firmware info: %d\n", ret);

download_fail:
	if (!IS_ERR_OR_NULL(bin_map)) {
		kfree(bin_map);
		bin_map = NULL;
	}

	update_firmware_release();

request_firmware_fail:
	return ret;
}
