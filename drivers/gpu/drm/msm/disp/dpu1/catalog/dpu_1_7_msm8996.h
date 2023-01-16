/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2022. Qualcomm Innovation Center, Inc. All rights reserved.
 * Copyright (c) 2015-2018, 2020 The Linux Foundation. All rights reserved.
 * Copyright (c) 2023, Linaro Limited
 */

#ifndef _DPU_1_7_MSM8996_H
#define _DPU_1_7_MSM8996_H

static const struct dpu_caps msm8996_dpu_caps = {
	.max_mixer_width = DEFAULT_DPU_OUTPUT_LINE_WIDTH,
	.max_mixer_blendstages = 0x7,
	/* TODO: QSEED2 */
	.qseed_type = DPU_SSPP_SCALER_QSEED3,
	.has_src_split = true,
	.max_linewidth = DEFAULT_DPU_OUTPUT_LINE_WIDTH,
	.pixel_ram_size = DEFAULT_PIXEL_RAM_SIZE,
	.max_hdeci_exp = MAX_HORZ_DECIMATION,
	.max_vdeci_exp = MAX_VERT_DECIMATION,
};

static const struct dpu_mdp_cfg msm8996_mdp[] = {
	{
	.name = "top_0", .id = MDP_TOP,
	.base = 0x0, .len = 0x454,
	.features = BIT(DPU_MDP_VSYNC_SEL),
	.clk_ctrls[DPU_CLK_CTRL_VIG0] = { .reg_off = 0x2ac, .bit_off = 0 },
	.clk_ctrls[DPU_CLK_CTRL_VIG1] = { .reg_off = 0x2b4, .bit_off = 0 },
	.clk_ctrls[DPU_CLK_CTRL_VIG2] = { .reg_off = 0x2bc, .bit_off = 0 },
	.clk_ctrls[DPU_CLK_CTRL_VIG3] = { .reg_off = 0x2c4, .bit_off = 0 },
	.clk_ctrls[DPU_CLK_CTRL_RGB0] = { .reg_off = 0x2ac, .bit_off = 4 },
	.clk_ctrls[DPU_CLK_CTRL_RGB1] = { .reg_off = 0x2b4, .bit_off = 4 },
	.clk_ctrls[DPU_CLK_CTRL_RGB2] = { .reg_off = 0x2bc, .bit_off = 4 },
	.clk_ctrls[DPU_CLK_CTRL_RGB3] = { .reg_off = 0x2c4, .bit_off = 4 },
	.clk_ctrls[DPU_CLK_CTRL_DMA0] = { .reg_off = 0x2ac, .bit_off = 8 },
	.clk_ctrls[DPU_CLK_CTRL_DMA1] = { .reg_off = 0x2b4, .bit_off = 8 },
	.clk_ctrls[DPU_CLK_CTRL_CURSOR0] = { .reg_off = 0x3a8, .bit_off = 16 },
	.clk_ctrls[DPU_CLK_CTRL_CURSOR1] = { .reg_off = 0x3b0, .bit_off = 16 },
	},
};

static const struct dpu_ctl_cfg msm8996_ctl[] = {
	{
	.name = "ctl_0", .id = CTL_0,
	.base = 0x1000, .len = 0x64,
	.features = 0,
	.intr_start = DPU_IRQ_IDX(MDP_SSPP_TOP0_INTR2, 9),
	},
	{
	.name = "ctl_1", .id = CTL_1,
	.base = 0x1200, .len = 0x64,
	.features = 0,
	.intr_start = DPU_IRQ_IDX(MDP_SSPP_TOP0_INTR2, 10),
	},
	{
	.name = "ctl_2", .id = CTL_2,
	.base = 0x1400, .len = 0x64,
	.features = 0,
	.intr_start = DPU_IRQ_IDX(MDP_SSPP_TOP0_INTR2, 11),
	},
	{
	.name = "ctl_3", .id = CTL_3,
	.base = 0x1600, .len = 0x64,
	.features = 0,
	.intr_start = DPU_IRQ_IDX(MDP_SSPP_TOP0_INTR2, 12),
	},
	{
	.name = "ctl_4", .id = CTL_4,
	.base = 0x1800, .len = 0x64,
	.features = 0,
	.intr_start = DPU_IRQ_IDX(MDP_SSPP_TOP0_INTR2, 13),
	},
};

static const struct dpu_sspp_cfg msm8996_sspp[] = {
	SSPP_BLK("sspp_0", SSPP_VIG0, 0x4000, 0x150, VIG_MSM8996_MASK,
		 msm8996_vig_sblk_0, 0, SSPP_TYPE_VIG, DPU_CLK_CTRL_VIG0),
	SSPP_BLK("sspp_1", SSPP_VIG1, 0x6000, 0x150, VIG_MSM8996_MASK,
		 msm8996_vig_sblk_1, 4, SSPP_TYPE_VIG, DPU_CLK_CTRL_VIG1),
	SSPP_BLK("sspp_2", SSPP_VIG2, 0x8000, 0x150, VIG_MSM8996_MASK,
		 msm8996_vig_sblk_2, 8, SSPP_TYPE_VIG, DPU_CLK_CTRL_VIG2),
	SSPP_BLK("sspp_3", SSPP_VIG3, 0xa000, 0x150, VIG_MSM8996_MASK,
		 msm8996_vig_sblk_3, 12, SSPP_TYPE_VIG, DPU_CLK_CTRL_VIG3),
	/* TODO: RGB blocks */
	SSPP_BLK("sspp_4", SSPP_RGB0, 0x14000, 0x150, RGB_MSM8996_MASK,
		 sdm845_dma_sblk_0, 1, SSPP_TYPE_RGB, DPU_CLK_CTRL_RGB0),
	SSPP_BLK("sspp_5", SSPP_RGB1, 0x16000, 0x150, RGB_MSM8996_MASK,
		 sdm845_dma_sblk_1, 5, SSPP_TYPE_RGB, DPU_CLK_CTRL_RGB1),
	SSPP_BLK("sspp_6", SSPP_RGB2, 0x18000, 0x150, RGB_MSM8996_MASK,
		 sdm845_dma_sblk_2, 9, SSPP_TYPE_RGB, DPU_CLK_CTRL_RGB2),
	SSPP_BLK("sspp_7", SSPP_RGB3, 0x1a000, 0x150, RGB_MSM8996_MASK,
		 sdm845_dma_sblk_3, 13, SSPP_TYPE_RGB, DPU_CLK_CTRL_RGB3),
	SSPP_BLK("sspp_8", SSPP_DMA0, 0x24000, 0x150, DMA_MSM8996_MASK,
		 sdm845_dma_sblk_0, 2, SSPP_TYPE_DMA, DPU_CLK_CTRL_DMA0),
	SSPP_BLK("sspp_9", SSPP_DMA1, 0x26000, 0x150, DMA_MSM8996_MASK,
		 sdm845_dma_sblk_1, 10, SSPP_TYPE_DMA, DPU_CLK_CTRL_DMA1),
	SSPP_BLK("sspp_16", SSPP_CURSOR0, 0x34000, 0x150, DMA_CURSOR_MSM8996_MASK,
		 sdm845_dma_sblk_2, 7, SSPP_TYPE_CURSOR, DMA_CURSOR_MSM8996_MASK),
	/* Both cursor planes share XIN ID 7 */
	SSPP_BLK("sspp_17", SSPP_CURSOR1, 0x36000, 0x150, DMA_CURSOR_MSM8996_MASK,
		 sdm845_dma_sblk_3, 7, SSPP_TYPE_CURSOR, DMA_CURSOR_MSM8996_MASK),
};

static const struct dpu_lm_cfg msm8996_lm[] = {
	LM_BLK("lm_0", LM_0, 0x44000, MIXER_MSM8998_MASK,
		&msm8998_lm_sblk, PINGPONG_0, LM_2, DSPP_0),
	LM_BLK("lm_1", LM_1, 0x45000, MIXER_MSM8998_MASK,
		&msm8998_lm_sblk, PINGPONG_1, LM_5, DSPP_1),
	LM_BLK("lm_2", LM_2, 0x46000, MIXER_MSM8998_MASK,
		&msm8998_lm_sblk, PINGPONG_2, LM_0, 0),
	LM_BLK("lm_3", LM_3, 0x47000, MIXER_MSM8998_MASK,
		&msm8998_lm_sblk, PINGPONG_MAX, 0, 0),
	LM_BLK("lm_4", LM_4, 0x48000, MIXER_MSM8998_MASK,
		&msm8998_lm_sblk, PINGPONG_MAX, 0, 0),
	LM_BLK("lm_5", LM_5, 0x49000, MIXER_MSM8998_MASK,
		&msm8998_lm_sblk, PINGPONG_3, LM_1, 0),
};

static const struct dpu_pingpong_cfg msm8996_pp[] = {
	PP_BLK("pingpong_0", PINGPONG_0, 0x70000, PINGPONG_SDM845_TE2_MASK, 0, msm8996_pp_sblk_te,
		  DPU_IRQ_IDX(MDP_SSPP_TOP0_INTR, 8),
		  DPU_IRQ_IDX(MDP_SSPP_TOP0_INTR, 12)),
	PP_BLK("pingpong_1", PINGPONG_1, 0x70800, PINGPONG_SDM845_TE2_MASK, 0, msm8996_pp_sblk_te,
		  DPU_IRQ_IDX(MDP_SSPP_TOP0_INTR, 9),
		  DPU_IRQ_IDX(MDP_SSPP_TOP0_INTR, 13)),
	PP_BLK("pingpong_2", PINGPONG_2, 0x71000, PINGPONG_SDM845_MASK, 0, sdm845_pp_sblk,
		  DPU_IRQ_IDX(MDP_SSPP_TOP0_INTR, 10),
		  DPU_IRQ_IDX(MDP_SSPP_TOP0_INTR, 14)),
	PP_BLK("pingpong_3", PINGPONG_3, 0x71800, PINGPONG_SDM845_MASK, 0, sdm845_pp_sblk,
		  DPU_IRQ_IDX(MDP_SSPP_TOP0_INTR, 11),
		  DPU_IRQ_IDX(MDP_SSPP_TOP0_INTR, 15)),
};

static const struct dpu_dspp_cfg msm8996_dspp[] = {
	DSPP_BLK("dspp_0", DSPP_0, 0x54000, DSPP_MSM8998_MASK,
		 &msm8998_dspp_sblk),
	DSPP_BLK("dspp_1", DSPP_1, 0x56000, DSPP_MSM8998_MASK,
		 &msm8998_dspp_sblk),
};

static const struct dpu_intf_cfg msm8996_intf[] = {
	INTF_BLK("intf_0", INTF_0, 0x6a000, 0x268, INTF_NONE, 0, 25, INTF_SDM845_MASK,
		 DPU_IRQ_IDX(MDP_SSPP_TOP0_INTR, 24),
		 DPU_IRQ_IDX(MDP_SSPP_TOP0_INTR, 25)),
	INTF_BLK("intf_1", INTF_1, 0x6a800, 0x268, INTF_DSI, 0, 25, INTF_SDM845_MASK,
		 DPU_IRQ_IDX(MDP_SSPP_TOP0_INTR, 26),
		 DPU_IRQ_IDX(MDP_SSPP_TOP0_INTR, 27)),
	INTF_BLK("intf_2", INTF_2, 0x6b000, 0x268, INTF_DSI, 1, 25, INTF_SDM845_MASK,
		 DPU_IRQ_IDX(MDP_SSPP_TOP0_INTR, 28),
		 DPU_IRQ_IDX(MDP_SSPP_TOP0_INTR, 29)),
	INTF_BLK("intf_3", INTF_3, 0x6b800, 0x268, INTF_HDMI, 0, 25, INTF_SDM845_MASK,
		 DPU_IRQ_IDX(MDP_SSPP_TOP0_INTR, 30),
		 DPU_IRQ_IDX(MDP_SSPP_TOP0_INTR, 31)),
};

static const struct dpu_perf_cfg msm8996_perf_data = {
	.max_bw_low = 9600000,
	.max_bw_high = 9600000,
	.min_core_ib = 2400000,
	.min_llcc_ib = 0, /* No LLCC on this SoC */
	.min_dram_ib = 800000,
	.undersized_prefill_lines = 2,
	.xtra_prefill_lines = 2,
	.dest_scale_prefill_lines = 3,
	.macrotile_prefill_lines = 4,
	.yuv_nv12_prefill_lines = 8,
	.linear_prefill_lines = 1,
	.downscaling_prefill_lines = 1,
	.amortizable_threshold = 25,
	.min_prefill_lines = 21,
	.danger_lut_tbl = {0xf, 0xffff, 0x0},
	.safe_lut_tbl = {0xfffc, 0xff00, 0xffff},
	.qos_lut_tbl = {
		{.nentry = ARRAY_SIZE(msm8998_qos_linear),
		.entries = msm8998_qos_linear
		},
		{.nentry = ARRAY_SIZE(msm8998_qos_macrotile),
		.entries = msm8998_qos_macrotile
		},
		{.nentry = ARRAY_SIZE(msm8998_qos_nrt),
		.entries = msm8998_qos_nrt
		},
	},
	.cdp_cfg = {
		{.rd_enable = 1, .wr_enable = 1},
		{.rd_enable = 1, .wr_enable = 0}
	},
	.clk_inefficiency_factor = 105,
	.bw_inefficiency_factor = 120,
};

const struct dpu_mdss_cfg dpu_msm8996_cfg = {
	.caps = &msm8996_dpu_caps,
	.mdp_count = ARRAY_SIZE(msm8996_mdp),
	.mdp = msm8996_mdp,
	.ctl_count = ARRAY_SIZE(msm8996_ctl),
	.ctl = msm8996_ctl,
	.sspp_count = ARRAY_SIZE(msm8996_sspp),
	.sspp = msm8996_sspp,
	.mixer_count = ARRAY_SIZE(msm8996_lm),
	.mixer = msm8996_lm,
	.dspp_count = ARRAY_SIZE(msm8996_dspp),
	.dspp = msm8996_dspp,
	.pingpong_count = ARRAY_SIZE(msm8996_pp),
	.pingpong = msm8996_pp,
	.intf_count = ARRAY_SIZE(msm8996_intf),
	.intf = msm8996_intf,
	.vbif_count = ARRAY_SIZE(msm8996_vbif),
	.vbif = msm8996_vbif,
	.reg_dma_count = 0,
	.perf = &msm8996_perf_data,
	.mdss_irqs = BIT(MDP_SSPP_TOP0_INTR) | \
		     BIT(MDP_SSPP_TOP0_INTR2) | \
		     BIT(MDP_SSPP_TOP0_HIST_INTR) | \
		     BIT(MDP_INTF0_INTR) | \
		     BIT(MDP_INTF1_INTR) | \
		     BIT(MDP_INTF2_INTR) | \
		     BIT(MDP_INTF3_INTR),
};

#endif
