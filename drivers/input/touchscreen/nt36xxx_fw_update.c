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

const struct firmware *fw_entry = NULL;
static u8 *fwbuf = NULL;

struct nt36xxx_part_hdr {
	u32 sram_dest_addr;
	u32 size;
	u32 addr_in_fw;
	u32 crc;
} __packed *part_hdr;

static u32 nt36xxx_calc_part_crc(const u8 *data, u32 addr, u32 len)
{
	u32 checksum = 0;
	u32 i;

	for (i = 0; i <= len; i++)
		checksum += data[addr + i];

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
static bool cascade_2nd_header = false;
static int nt36xxx_fw_header_parser(struct nt36xxx_data *ts, const u32 *fwdata, size_t fwsize)
{
	struct device *dev = &ts->client->dev;
	bool overlay_present = false;
	u8 num_overlay_sections = 0;
	bool bin_hdr_found = false;
	u8 num_info_sections = 0;
	u32 header_end;
	u32 part_idx;
	u32 pos;

	header_end = le32_to_cpu(((__le32 *)fwdata)[0]) / sizeof(u32);
	cascade_2nd_header = !!(fwdata[8] & BIT(1));

	/* Info section start */
	pos = 3 * sizeof(u32);
	if (cascade_2nd_header) {
		while (pos < (header_end / 2)) {
			num_info_sections++;
			pos += sizeof(u32);
		}

		num_info_sections++;
	} else {
		while (pos < header_end) {
			num_info_sections++;
			pos += sizeof(u32);
		}
	}

	overlay_present = FIELD_GET(BIT(4), fwdata[10]);
	if (overlay_present)
		num_overlay_sections = FIELD_GET(GENMASK(3, 0), fwdata[10]);

	/*
	 * Count the total number of partitions
	 * 1 (ILM) + 1 (DLM) + num_overlay_sections + num_info_sections
	 */
	num_parts = 2 + num_overlay_sections + num_info_sections;

	part_hdr = kcalloc(num_parts + 1, sizeof(struct nt36xxx_part_hdr), GFP_KERNEL);
	if (!part_hdr)
		return -ENOMEM;

	/* The "necessary" partitions are a bit all over the place */
	part_hdr[PART_IDX_ILM].addr_in_fw = le32_to_cpu(fwdata[0]);
	part_hdr[PART_IDX_ILM].sram_dest_addr = le32_to_cpu(fwdata[1]);
	part_hdr[PART_IDX_ILM].size = le32_to_cpu(fwdata[2]);
	part_hdr[PART_IDX_ILM].crc = le32_to_cpu(fwdata[6]);

	part_hdr[PART_IDX_DLM].addr_in_fw = le32_to_cpu(fwdata[3]);
	part_hdr[PART_IDX_DLM].sram_dest_addr = le32_to_cpu(fwdata[4]);
	part_hdr[PART_IDX_DLM].size = le32_to_cpu(fwdata[5]);
	part_hdr[PART_IDX_DLM].crc = le32_to_cpu(fwdata[7]);

	/* The overlays follow a more structured pattern */
	for (part_idx = PART_IDX_OTHER; part_idx < num_parts; part_idx++) {
		/* "info" / "other" partitions */
		if (part_idx < PART_IDX_OTHER + num_info_sections) {
			if (!bin_hdr_found) {
				/* others partition located at 0x30 offset */
				pos = 3 * sizeof(u32);
				pos += sizeof(u32) * (part_idx - PART_IDX_OTHER);
			} else if (bin_hdr_found && cascade_2nd_header) {
				/* cascade 2nd header info */
				pos = header_end - sizeof(u32);
			}

			memcpy(&part_hdr[part_idx], &fwdata[pos], sizeof(struct nt36xxx_part_hdr));

			/* ... */
			if (le32_to_cpu(part_hdr[part_idx].addr_in_fw) < header_end && le32_to_cpu(part_hdr[part_idx].size))
				bin_hdr_found = true;
		} else {
			/* Overlay partitions */
			/* overlay info located at DLM (list = 1) start addr */
			pos = part_hdr[PART_IDX_DLM].addr_in_fw + sizeof(u32) * (part_idx - PART_IDX_OTHER - num_info_sections);

			memcpy(&part_hdr[part_idx], &fwdata[pos], sizeof(struct nt36xxx_part_hdr));
		}

		part_hdr[part_idx].addr_in_fw = le32_to_cpu(part_hdr[part_idx].addr_in_fw);
		part_hdr[part_idx].sram_dest_addr = le32_to_cpu(part_hdr[part_idx].sram_dest_addr);
		part_hdr[part_idx].size = le32_to_cpu(part_hdr[part_idx].size);
		part_hdr[part_idx].crc = le32_to_cpu(part_hdr[part_idx].crc);

		if (part_hdr[part_idx].addr_in_fw + part_hdr[part_idx].size > fwsize) {
			dev_err(dev, "Range %d [0x%8x-0x%8x] goes out of firwmare bounds\n",
				part_idx,
				part_hdr[part_idx].addr_in_fw,
				part_hdr[part_idx].addr_in_fw + part_hdr[part_idx].size);
			goto fail;
		}

		if (!ts->hw_crc) {
			part_hdr[part_idx].crc = nt36xxx_calc_part_crc((const u8 *)&fwdata,
								       part_hdr[part_idx].addr_in_fw,
								       part_hdr[part_idx].size);
		}
	}

	return 0;

fail:
	kfree(part_hdr);
	part_hdr = NULL;

	return -EINVAL;
}

static int nt36xxx_prepare_fw(struct nt36xxx_data *ts, char *filename)
{
	struct device *dev = &ts->client->dev;
	u32 real_fw_size = 0;
	u32 version_offset;
	int sector;
	int ret;

	if (!filename)
		return -ENOENT;

	ret = request_firmware(&fw_entry, filename, &ts->client->dev);
	if (ret) {
		dev_err(dev, "request_firmware failed: %d\n", ret);
		goto fail;
	}

	/* The firmware can apparently contain some trailing garbage/data.. Find the "real end" */
	for (sector = fw_entry->size / FLASH_SECTOR_SIZE; sector > 0; sector--)
		if (!strncmp(&fw_entry->data[sector * FLASH_SECTOR_SIZE - 3], "NVT", 3))
			real_fw_size = sector * FLASH_SECTOR_SIZE;

	if (!real_fw_size) {
		ret = -EINVAL;
		goto fail;
	}

	/* Sanity check */
	version_offset = real_fw_size - FLASH_SECTOR_SIZE;
	if (fw_entry->data[version_offset] + fw_entry->data[version_offset + 1] != U8_MAX) {
		dev_err(dev, "Firmware file is corrupted!\n");

		if (fw_entry)
			release_firmware(fw_entry);

		fw_entry = NULL;

		if (!IS_ERR_OR_NULL(part_hdr)) {
			kfree(part_hdr);
			part_hdr = NULL;
		}

		ret = -ENOEXEC;
		goto fail;
	}

	/* BIN Header Parser */
	ret = nt36xxx_fw_header_parser(ts, (const u32 *)fw_entry->data, fw_entry->size);
	if (ret) {
		dev_err(dev, "Failed to parse the firmware header\n");
		goto fail;
	}

	return 0;

fail:
	kfree(part_hdr);
	part_hdr = NULL;

	return ret;
}

static int nt36xxx_write_sram(struct nt36xxx_data *ts, const u8 *fwdata,
			  u32 sram_dest_addr, u32 size, u32 addr_in_fw)
{
	int count = size / NVT_TRANSFER_LEN;
	int ret;
	u16 len;
	u32 i;

	if (size % NVT_TRANSFER_LEN)
		count += 1;

	for (i = 0; i < count; i++) {
		len = min(size, NVT_TRANSFER_LEN);

		ret = nt36xxx_set_page(ts, sram_dest_addr);
		if (ret)
			return ret;

		fwbuf[0] = ADDR_WITHIN_PAGE(sram_dest_addr);
		memcpy(fwbuf + 1, &fwdata[addr_in_fw], len);

		ret = nt36xxx_spi_write(ts->client, fwbuf, len + 1);
		if (ret)
			return ret;

		sram_dest_addr += NVT_TRANSFER_LEN;
		addr_in_fw += NVT_TRANSFER_LEN;
		size -= NVT_TRANSFER_LEN;
	}

	return 0;
}

static int nt36xxx_write_firmware(struct nt36xxx_data *ts, const u8 *fwdata, size_t fwsize)
{
	struct device *dev = &ts->client->dev;
	u32 part_idx;
	int ret;

	memset(fwbuf, 0, NVT_TRANSFER_LEN + 1);

	for (part_idx = 0; part_idx < num_parts; part_idx++) {
		/* Ignore 'reserved' partitions (with size = 0) */
		if (!part_hdr[part_idx].size)
			continue;

		/* Write data to SRAM */
		ret = nt36xxx_write_sram(ts, fwdata,
				     part_hdr[part_idx].sram_dest_addr,
				     part_hdr[part_idx].size + 1,
				     part_hdr[part_idx].addr_in_fw);
		if (ret) {
			dev_err(dev, "SRAM programming failed: %d\n", ret);
			return ret;
		}
	}

	return 0;
}

static int nt36xxx_check_fw_checksum(struct nt36xxx_data *ts)
{
	struct device *dev = &ts->client->dev;
	u32 len = num_parts * 4;
	u32 fw_checksum;
	u32 part_idx;
	int ret;

	memset(fwbuf, 0, (len + 1));

	nt36xxx_set_page(ts, ts->mmap->r_ilm_checksum_addr);

	/* Read the checksum */
	fwbuf[0] = ADDR_WITHIN_PAGE(ts->mmap->r_ilm_checksum_addr);
	ret = nt36xxx_spi_read(ts->client, fwbuf, len + 1);
	if (ret)
		return ret;

	for (part_idx = 0; part_idx < num_parts; part_idx++) {
		fw_checksum = le32_to_cpu(((__le32 *)fwbuf)[4 * part_idx + 1]);

		/* Ignore 'reserved' partitions (with size = 0) */
		if (!part_hdr[part_idx].size)
			continue;

		if (part_hdr[part_idx].crc != fw_checksum) {
			dev_err(dev, "Firmware checksum doesn't match for partition %d\n", part_idx);
			return -EINVAL;
		}
	}

	return 0;
}

static void nt36xxx_set_bl_crc_bank(struct nt36xxx_data *ts, u8 bank_idx)
{
	u32 DEST_ADDR = bank_idx == 0 ? ts->mmap->ilm_dest_addr : ts->mmap->dlm_dest_addr;
	u32 G_CHECKSUM_ADDR = bank_idx == 0 ? ts->mmap->g_ilm_checksum_addr : ts->mmap->g_dlm_checksum_addr;
	u32 LENGTH_ADDR = bank_idx == 0 ? ts->mmap->ilm_length_addr : ts->mmap->dlm_length_addr;
	u32 sram_dest_addr = part_hdr[bank_idx].sram_dest_addr;
	u32 size = part_hdr[bank_idx].size;
	u32 crc = part_hdr[bank_idx].crc;

	nt36xxx_set_page(ts, DEST_ADDR);
	fwbuf[0] = ADDR_WITHIN_PAGE(DEST_ADDR);
	fwbuf[1] = FIELD_GET(GENMASK(7, 0), sram_dest_addr);
	fwbuf[2] = FIELD_GET(GENMASK(15, 8), sram_dest_addr);
	fwbuf[3] = FIELD_GET(GENMASK(23, 16), sram_dest_addr);
	nt36xxx_spi_write(ts->client, fwbuf, 4);

	fwbuf[0] = ADDR_WITHIN_PAGE(LENGTH_ADDR);
	fwbuf[1] = FIELD_GET(GENMASK(7, 0), size);
	fwbuf[2] = FIELD_GET(GENMASK(15, 8), size);
	fwbuf[3] = FIELD_GET(GENMASK(23, 16), size);
	nt36xxx_spi_write(ts->client, fwbuf, ts->hw_crc == 1 ? 3 : 4);

	fwbuf[0] = ADDR_WITHIN_PAGE(G_CHECKSUM_ADDR);
	fwbuf[1] = FIELD_GET(GENMASK(7, 0), crc);
	fwbuf[2] = FIELD_GET(GENMASK(15, 8), crc);
	fwbuf[3] = FIELD_GET(GENMASK(23, 16), crc);
	fwbuf[4] = FIELD_GET(GENMASK(31, 24), crc);
	nt36xxx_spi_write(ts->client, fwbuf, 5);
}

static int nt36xxx_download_fw_hw_crc(struct nt36xxx_data *ts)
{
	struct device *dev = &ts->client->dev;
	int ret;

	nt36xxx_bootloader_reset(ts);

	/* Set ILM & DLM register banks for bootloader HW CRC */
	nt36xxx_set_bl_crc_bank(ts, 0);
	nt36xxx_set_bl_crc_bank(ts, 1);

	if (cascade_2nd_header)
		nt36xxx_write_addr(ts, ts->mmap->tx_auto_copy_en, 0x69);

	ret = nt36xxx_write_firmware(ts, fw_entry->data, fw_entry->size);
	if (ret) {
		dev_err(dev, "Failed to write firmware: %d\n", ret);
		return ret;
	}

	if (cascade_2nd_header) {
		ret = nt36xxx_check_spi_dma_tx_info(ts);
		if (ret) {
			dev_err(dev, "SPI DMA TX info check failed: %d\n", ret);
			return ret;
		}
	}

	nt36xxx_enable_bl_crc(ts);
	nt36xxx_enable_fw_crc(ts);
	nt36xxx_set_boot_ready(ts);

	return nt36xxx_check_fw_reset_state(ts, RESET_STATE_INIT);
}

static int nt36xxx_download_fw(struct nt36xxx_data *ts)
{
	struct device *dev = &ts->client->dev;
	int ret;

	gpiod_set_value_cansleep(ts->reset_gpio, 1);
	mdelay(1);

	nt36xxx_eng_reset(ts);

	gpiod_set_value_cansleep(ts->reset_gpio, 0);
	mdelay(10);

	nt36xxx_bootloader_reset(ts);

	/* Clear FW reset status */
	nt36xxx_write_addr(ts, ts->mmap->event_buf | EVENT_MAP_RESET_COMPLETE, 0);

	ret = nt36xxx_write_firmware(ts, fw_entry->data, fw_entry->size);
	if (ret) {
		dev_err(dev, "Failed to write firmware: %d\n", ret);
		return ret;
	}

	nt36xxx_set_boot_ready(ts);

	ret = nt36xxx_check_fw_reset_state(ts, RESET_STATE_INIT);
	if (ret) {
		dev_err(dev, "Firmware reset state check failed: %d\n", ret);
		return ret;
	}

	/* Validate the flashed firmware checksum */
	return nt36xxx_check_fw_checksum(ts);
}

int nt36xxx_update_fw(struct nt36xxx_data *ts, char *firmware_name)
{
	struct device *dev = &ts->client->dev;
	int ret;

	ret = nt36xxx_prepare_fw(ts, firmware_name);
	if (ret) {
		dev_err(dev, "Couldn't prepare firmware: %d\n", ret);
		return ret;
	}

	/* initial buffer and variable */
	if (!fwbuf) {
		fwbuf = kzalloc(NVT_TRANSFER_LEN + 1 + DUMMY_BYTES, GFP_KERNEL);
		if (!fwbuf) {
			ret = -ENOMEM;
			goto fail;
		}
	}

	/* HW CRC-enabled chips require a slightly different procedure */
	if (ts->hw_crc)
		ret = nt36xxx_download_fw_hw_crc(ts);
	else
		ret = nt36xxx_download_fw(ts);
	if (ret) {
		dev_err(dev, "Firmware download failed: %d\n", ret);
		goto fail;
	}

	/* Get FW Info */
	ret = nt36xxx_get_fw_info(ts);
	if (ret)
		dev_err(dev, "Couldn't get firmware info: %d\n", ret);

fail:
	kfree(part_hdr);
	part_hdr = NULL;

	if (fw_entry)
		release_firmware(fw_entry);

	fw_entry = NULL;

	return ret;
}
