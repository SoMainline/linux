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

#define NVT_FLASH_END_FLAG_ADDR		(fw_need_write_size - NVT_FLASH_END_FLAG_LEN)
#define NVT_FLASH_END_FLAG_LEN		3

const struct firmware *fw_entry = NULL;
static size_t fw_need_write_size = 0;
static u8 *fwbuf = NULL;

struct nvt_ts_bin_map {
	char name[12];
	u32 BIN_addr;
	u32 SRAM_addr;
	u32 size;
	u32 crc;
};

static struct nvt_ts_bin_map *bin_map;

static int nvt_get_fw_need_write_size(const struct firmware *fw_entry)
{
	int total_sectors_to_check = fw_entry->size / FLASH_SECTOR_SIZE;
	int i;

	for (i = total_sectors_to_check; i > 0; i--) {
		/* printk("current end flag address checked = 0x%X\n", i * FLASH_SECTOR_SIZE - NVT_FLASH_END_FLAG_LEN); */
		/* check if there is end flag "NVT" at the end of this sector */
		if (strncmp(&fw_entry->data[i * FLASH_SECTOR_SIZE - NVT_FLASH_END_FLAG_LEN], "NVT", NVT_FLASH_END_FLAG_LEN) == 0) {
			fw_need_write_size = i * FLASH_SECTOR_SIZE;
			NVT_LOG("fw_need_write_size = %zu(0x%zx), NVT end flag\n", fw_need_write_size, fw_need_write_size);
			return 0;
		}

		/* check if there is end flag "MOD" at the end of this sector */
		if (strncmp(&fw_entry->data[i * FLASH_SECTOR_SIZE - NVT_FLASH_END_FLAG_LEN], "MOD", NVT_FLASH_END_FLAG_LEN) == 0) {
			fw_need_write_size = i * FLASH_SECTOR_SIZE;
			NVT_LOG("fw_need_write_size = %zu(0x%zx), MOD end flag\n", fw_need_write_size, fw_need_write_size);
			return 0;
		}
	}

	NVT_ERR("end flag \"NVT\" \"MOD\" not found!\n");

	return -1;
}

/*******************************************************
Description:
	Novatek touchscreen init variable and allocate buffer
for download firmware function.

return:
	n.a.
*******************************************************/
static int nvt_download_init(struct nvt_ts_data *ts)
{
	if (fwbuf)
		return 0;

	fwbuf = kzalloc((NVT_TRANSFER_LEN + 1 + DUMMY_BYTES), GFP_KERNEL);
	if (!fwbuf)
		return -ENOMEM;

	return 0;
}

/*******************************************************
Description:
	Novatek touchscreen checksum function. Calculate bin
file checksum for comparison.

return:
	n.a.
*******************************************************/
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

static u32 byte_to_word(const u8 *data, unsigned int start_idx)
{
	return (data[start_idx] << 0) |
	       (data[start_idx + 1] << 8) |
	       (data[start_idx + 2] << 16) |
	       (data[start_idx + 3] << 24);
}

/*******************************************************
Description:
	Novatek touchscreen parsing bin header function.

return:
	n.a.
*******************************************************/
static u32 partition = 0;
static u8 ilm_dlm_num = 2;
static u8 cascade_2nd_header_info = 0;
static int nvt_bin_header_parser(struct nvt_ts_data *ts, const u8 *fwdata, size_t fwsize)
{
	u32 list = 0;
	u32 pos = 0x00;
	u32 end = 0x00;
	u8 info_sec_num = 0;
	u8 ovly_sec_num = 0;
	u8 ovly_info = 0;
	u8 find_bin_header = 0;

	/* Find the header size */
	end = byte_to_word(fwdata, 0);

	/* check cascade next header */
	cascade_2nd_header_info = (fwdata[0x20] & 0x02) >> 1;
	NVT_LOG("cascade_2nd_header_info = %d\n", cascade_2nd_header_info);

	if (cascade_2nd_header_info) {
		pos = 0x30;	// info section start at 0x30 offset
		while (pos < (end / 2)) {
			info_sec_num ++;
			pos += 0x10;	/* each header info is 16 bytes */
		}

		info_sec_num = info_sec_num + 1; //next header section
	} else {
		pos = 0x30;	// info section start at 0x30 offset
		while (pos < end) {
			info_sec_num ++;
			pos += 0x10;	/* each header info is 16 bytes */
		}
	}

	/*
	 * Find the DLM OVLY section
	 * [0:3] Overlay Section Number
	 * [4]   Overlay Info
	 */
	ovly_info = (fwdata[0x28] & 0x10) >> 4;
	ovly_sec_num = (ovly_info) ? (fwdata[0x28] & GENMASK(3, 0)) : 0;

	/*
	 * calculate all partition number
	 * ilm_dlm_num (ILM & DLM) + ovly_sec_num + info_sec_num
	 */
	partition = ilm_dlm_num + ovly_sec_num + info_sec_num;
	NVT_LOG("ovly_info = %d, ilm_dlm_num = %d, ovly_sec_num = %d, info_sec_num = %d, partition = %d\n",
			ovly_info, ilm_dlm_num, ovly_sec_num, info_sec_num, partition);

	/* allocated memory for header info */
	bin_map = kcalloc((partition+1), sizeof(struct nvt_ts_bin_map), GFP_KERNEL);
	if (!bin_map) {
		NVT_ERR("kzalloc for bin_map failed!\n");
		return -ENOMEM;
	}

	for (list = 0; list < partition; list++) {
		/*
		 * [1] parsing ILM & DLM header info
		 * BIN_addr : SRAM_addr : size (12-bytes)
		 * crc located at 0x18 & 0x1C
		 */
		if (list < ilm_dlm_num) {
			bin_map[list].BIN_addr = byte_to_word(fwdata, 12 * list);
			bin_map[list].SRAM_addr = byte_to_word(fwdata, 12 * list + 4);
			bin_map[list].size = byte_to_word(fwdata, 12 * list + 8);
			if (ts->hw_crc)
				bin_map[list].crc = byte_to_word(fwdata, 4 * list + 0x18);
			else {
				if ((bin_map[list].BIN_addr + bin_map[list].size) < fwsize)
					bin_map[list].crc = CheckSum(&fwdata[bin_map[list].BIN_addr], bin_map[list].size);
				else {
					NVT_ERR("access range (0x%08X to 0x%08X) is larger than bin size!\n",
							bin_map[list].BIN_addr, bin_map[list].BIN_addr + bin_map[list].size);
					return -EINVAL;
				}
			}
			if (list == 0)
				sprintf(bin_map[list].name, "ILM");
			else if (list == 1)
				sprintf(bin_map[list].name, "DLM");
		}

		/*
		 * [2] parsing others header info
		 * SRAM_addr : size : BIN_addr : crc (16-bytes)
		 */
		if ((list >= ilm_dlm_num) && (list < (ilm_dlm_num + info_sec_num))) {
			if (find_bin_header == 0) {
				/* others partition located at 0x30 offset */
				pos = 0x30 + (0x10 * (list - ilm_dlm_num));
			} else if (find_bin_header && cascade_2nd_header_info) {
				/* cascade 2nd header info */
				pos = end - 0x10;
			}

			bin_map[list].SRAM_addr = byte_to_word(fwdata, pos);
			bin_map[list].size = byte_to_word(fwdata, pos + 4);
			bin_map[list].BIN_addr = byte_to_word(fwdata, pos + 8);
			if (ts->hw_crc)
				bin_map[list].crc = byte_to_word(fwdata, pos + 12);
			else {
				if ((bin_map[list].BIN_addr + bin_map[list].size) < fwsize)
					bin_map[list].crc = CheckSum(&fwdata[bin_map[list].BIN_addr], bin_map[list].size);
				else {
					NVT_ERR("access range (0x%08X to 0x%08X) is larger than bin size!\n",
							bin_map[list].BIN_addr, bin_map[list].BIN_addr + bin_map[list].size);
					return -EINVAL;
				}
			}
			/* detect header end to protect parser function */
			if ((bin_map[list].BIN_addr < end) && (bin_map[list].size != 0)) {
				sprintf(bin_map[list].name, "Header");
				find_bin_header = 1;
			} else {
				sprintf(bin_map[list].name, "Info-%d", (list - ilm_dlm_num));
			}
		}

		/*
		 * [3] parsing overlay section header info
		 * SRAM_addr : size : BIN_addr : crc (16-bytes)
		 */
		if (list >= (ilm_dlm_num + info_sec_num)) {
			/* overlay info located at DLM (list = 1) start addr */
			pos = bin_map[1].BIN_addr + (0x10 * (list- ilm_dlm_num - info_sec_num));

			bin_map[list].SRAM_addr = byte_to_word(fwdata, pos);
			bin_map[list].size = byte_to_word(fwdata, pos + 4);
			bin_map[list].BIN_addr = byte_to_word(fwdata, pos + 8);
			if (ts->hw_crc)
				bin_map[list].crc = byte_to_word(fwdata, pos + 12);
			else {
				if ((bin_map[list].BIN_addr + bin_map[list].size) < fwsize)
					bin_map[list].crc = CheckSum(&fwdata[bin_map[list].BIN_addr], bin_map[list].size);
				else {
					NVT_ERR("access range (0x%08X to 0x%08X) is larger than bin size!\n",
							bin_map[list].BIN_addr, bin_map[list].BIN_addr + bin_map[list].size);
					return -EINVAL;
				}
			}
			sprintf(bin_map[list].name, "Overlay-%d", (list- ilm_dlm_num - info_sec_num));
		}

		/* BIN size error detect */
		if ((bin_map[list].BIN_addr + bin_map[list].size) > fwsize) {
			NVT_ERR("access range (0x%08X to 0x%08X) is larger than bin size!\n",
					bin_map[list].BIN_addr, bin_map[list].BIN_addr + bin_map[list].size);
			return -EINVAL;
		}
	}

	return 0;
}

/*******************************************************
Description:
	Novatek touchscreen release update firmware function.

return:
	n.a.
*******************************************************/
static void update_firmware_release(void)
{
	if (fw_entry) {
		release_firmware(fw_entry);
	}

	fw_entry = NULL;
}

/*******************************************************
Description:
	Novatek touchscreen request update firmware function.

return:
	Executive outcomes. 0---succeed. -1,-22---failed.
*******************************************************/
static int update_firmware_request(struct nvt_ts_data *ts, char *filename)
{
	u8 retry = 0;
	int ret = 0;

	if (NULL == filename) {
		return -ENOENT;
	}

	while (1) {
		NVT_LOG("filename is %s\n", filename);

		ret = request_firmware(&fw_entry, filename, &ts->client->dev);
		if (ret) {
			NVT_ERR("firmware load failed, ret=%d\n", ret);
			goto request_fail;
		}

		// check FW need to write size
		if (nvt_get_fw_need_write_size(fw_entry)) {
			NVT_ERR("get fw need to write size fail!\n");
			ret = -EINVAL;
			goto invalid;
		}

		// check if FW version add FW version bar equals 0xFF
		if (*(fw_entry->data + FW_BIN_VER_OFFSET) + *(fw_entry->data + FW_BIN_VER_BAR_OFFSET) != 0xFF) {
			NVT_ERR("bin file FW_VER + FW_VER_BAR should be 0xFF!\n");
			NVT_ERR("FW_VER=0x%02X, FW_VER_BAR=0x%02X\n", *(fw_entry->data+FW_BIN_VER_OFFSET), *(fw_entry->data+FW_BIN_VER_BAR_OFFSET));
			ret = -ENOEXEC;
			goto invalid;
		}

		/* BIN Header Parser */
		ret = nvt_bin_header_parser(ts, fw_entry->data, fw_entry->size);
		if (ret) {
			NVT_ERR("bin header parser failed\n");
			goto invalid;
		} else {
			break;
		}

invalid:
		update_firmware_release();
		if (!IS_ERR_OR_NULL(bin_map)) {
			kfree(bin_map);
			bin_map = NULL;
		}

request_fail:
		retry++;
		if(unlikely(retry > 2)) {
			NVT_ERR("error, retry=%d\n", retry);
			break;
		}
	}

	return ret;
}

/*******************************************************
Description:
	Novatek touchscreen write data to sram function.

- fwdata   : The buffer is written
- SRAM_addr: The sram destination address
- size     : Number of data bytes in @fwdata being written
- BIN_addr : The transferred data offset of @fwdata

return:
	Executive outcomes. 0---succeed. else---fail.
*******************************************************/
static int nvt_write_sram(struct nvt_ts_data *ts,
			  const u8 *fwdata,
			  u32 SRAM_addr,
			  u32 size,
			  u32 BIN_addr)
{
	int ret = 0;
	u32 i = 0;
	u16 len = 0;
	int count = 0;

	if (size % NVT_TRANSFER_LEN)
		count = (size / NVT_TRANSFER_LEN) + 1;
	else
		count = (size / NVT_TRANSFER_LEN);

	for (i = 0 ; i < count ; i++) {
		len = (size < NVT_TRANSFER_LEN) ? size : NVT_TRANSFER_LEN;

		//---set xdata index to start address of SRAM---
		ret = nvt_set_addr(ts, SRAM_addr);
		if (ret) {
			NVT_ERR("set page failed, ret = %d\n", ret);
			return ret;
		}

		//---write data into SRAM---
		fwbuf[0] = SRAM_addr & 0x7F;	//offset
		memcpy(fwbuf+1, &fwdata[BIN_addr], len);	//payload
		ret = nt36xxx_spi_write(ts->client, fwbuf, len + 1);
		if (ret) {
			NVT_ERR("write to sram failed, ret = %d\n", ret);
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
	char *name;
	int ret;

	memset(fwbuf, 0, (NVT_TRANSFER_LEN + 1));

	for (list = 0; list < partition; list++) {
		/* initialize variable */
		SRAM_addr = bin_map[list].SRAM_addr;
		size = bin_map[list].size;
		BIN_addr = bin_map[list].BIN_addr;
		name = bin_map[list].name;

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
	u32 len = partition * 4;
	u32 fw_checksum;
	u32 list = 0;
	int ret;

	memset(fwbuf, 0, (len + 1));

	//---set xdata index to checksum---
	nvt_set_addr(ts, ts->mmap->r_ilm_checksum_addr);

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
	 * ilm_dlm_num (ILM & DLM) + ovly_sec_num + info_sec_num
	 */
	for (list = 0; list < partition; list++) {
		fw_checksum = byte_to_word(fwbuf, 4 * list + 1);

		/* ignore reserved partition (Reserved Partition size is zero) */
		if (!bin_map[list].size)
			continue;

		if (bin_map[list].crc != fw_checksum) {
			NVT_ERR("[%d] BIN_checksum=0x%08X, FW_checksum=0x%08X\n",
					list, bin_map[list].crc, fw_checksum);
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

	nvt_set_addr(ts, DEST_ADDR);
	fwbuf[0] = DEST_ADDR & GENMASK(6, 0);
	fwbuf[1] = (sram_addr) & GENMASK(7, 0);
	fwbuf[2] = (sram_addr >> 8) & GENMASK(7, 0);
	fwbuf[3] = (sram_addr >> 16) & GENMASK(7, 0);
	nt36xxx_spi_write(ts->client, fwbuf, 4);

	fwbuf[0] = LENGTH_ADDR & GENMASK(6, 0);
	fwbuf[1] = (size) & GENMASK(7, 0);
	fwbuf[2] = (size >> 8) & GENMASK(7, 0);
	fwbuf[3] = (size >> 16) & 0x01;
	nt36xxx_spi_write(ts->client, fwbuf, ts->hw_crc > 1 ? 4 : 3);

	fwbuf[0] = G_CHECKSUM_ADDR & GENMASK(6, 0);
	fwbuf[1] = (crc) & GENMASK(7, 0);
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
	int ret;

	// request bin file in "/etc/firmware"
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
		pr_err("couldn't dl the fw\n");
		goto download_fail;
	}

	/* Get FW Info */
	ret = nvt_get_fw_info(ts);
	if (ret)
		NVT_ERR("nvt_get_fw_info failed. (%d)\n", ret);

download_fail:
	if (!IS_ERR_OR_NULL(bin_map)) {
		kfree(bin_map);
		bin_map = NULL;
	}

	update_firmware_release();

request_firmware_fail:
	return ret;
}
