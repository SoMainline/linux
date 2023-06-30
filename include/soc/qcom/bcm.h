/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2020-2021, The Linux Foundation. All rights reserved.
 */

#ifndef __SOC_QCOM_BCM_H__
#define __SOC_QCOM_BCM_H__

/**
 * struct bcm_db - Auxiliary data pertaining to each Bus Clock Manager (BCM)
 * @unit: divisor used to convert bytes/sec bw value to an RPMh msg
 * @width: multiplier used to convert bytes/sec bw value to an RPMh msg
 * @vcd: virtual clock domain that this bcm belongs to
 * @reserved: reserved field
 */
struct bcm_db {
	__le32 unit;
	__le16 width;
	u8 vcd;
	u8 reserved;
};

struct bcm {
	const char *name;
	struct bcm_db aux_data;
	bool fixed;
};

static inline u64 bcm_div(u64 num, u32 base)
{
	/* Ensure that small votes aren't lost. */
	if (num && num < base)
		return 1;

	do_div(num, base);

	return num;
}

int qcom_bcm_read_aux_data(struct device *dev, const char *name, struct bcm_db *aux_data);

#endif
