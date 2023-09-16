/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2022. Qualcomm Innovation Center, Inc. All rights reserved.
 * Copyright (c) 2015-2018, 2020 The Linux Foundation. All rights reserved.
 * Copyright (c) 2023, Linaro Limited
 */

#ifndef _DPU_1_9_MSM8994_H
#define _DPU_1_9_MSM8994_H

static const struct dpu_caps msm8994_dpu_caps = {
	.max_mixer_width = 2048,
	.max_mixer_blendstages = 0x7,//?
	// .qseed_type = DPU_SSPP_SCALER_QSEED2,
	.has_src_split = true,
	.max_linewidth = 2048,
	.pixel_ram_size = DEFAULT_PIXEL_RAM_SIZE,
	.max_hdeci_exp = MAX_HORZ_DECIMATION,
	.max_vdeci_exp = MAX_VERT_DECIMATION,
};

static const struct dpu_dspp_cfg msm8994_dspp[] = {
	{
		.name = "dspp_0", .id = DSPP_0,
		.base = 0x54000, .len = 0x1800,
		.features = DSPP_SC7180_MASK,
		.sblk = &msm8998_dspp_sblk,
	}, {
		.name = "dspp_1", .id = DSPP_1,
		.base = 0x56000, .len = 0x1800,
		.features = DSPP_SC7180_MASK,
		.sblk = &msm8998_dspp_sblk,
	}, {
		.name = "dspp_2", .id = DSPP_2,
		.base = 0x58000, .len = 0x1800,
		.features = DSPP_SC7180_MASK,
		.sblk = &msm8998_dspp_sblk,
	}, {
		.name = "dspp_3", .id = DSPP_3,
		.base = 0x5a000, .len = 0x1800,
		.features = DSPP_SC7180_MASK,
		.sblk = &msm8998_dspp_sblk,
	},
};

static const struct dpu_mdss_version msm8994_mdss_ver = {
	.core_major_ver = 1,
	.core_minor_ver = 9,
};

const struct dpu_mdss_cfg dpu_msm8994_cfg = {
	.mdss_ver = &msm8994_mdss_ver,
	.caps = &msm8994_dpu_caps,
	.mdp = msm8996_mdp,
	.ctl_count = ARRAY_SIZE(msm8996_ctl),
	.ctl = msm8996_ctl,
	.sspp_count = ARRAY_SIZE(msm8996_sspp),
	.sspp = msm8996_sspp,
	.mixer_count = ARRAY_SIZE(msm8996_lm),
	.mixer = msm8996_lm,
	.dspp_count = ARRAY_SIZE(msm8994_dspp),
	.dspp = msm8994_dspp,
	.pingpong_count = ARRAY_SIZE(msm8996_pp),
	.pingpong = msm8996_pp,
	.intf_count = ARRAY_SIZE(msm8996_intf),
	.intf = msm8996_intf,
	.vbif_count = ARRAY_SIZE(msm8996_vbif),
	.vbif = msm8996_vbif,
	.perf = &msm8996_perf_data,
};

#endif
