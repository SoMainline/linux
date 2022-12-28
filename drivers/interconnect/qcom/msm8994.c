// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2022, Linaro Limited.
 */

#include <dt-bindings/interconnect/qcom,msm8994.h>
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/interconnect-provider.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

#include "smd-rpm.h"
#include "icc-rpm.h"
#include "msm8994.h"

static const char * const mnoc_intf_clocks[] = {
	"maxi",
	"axi",
	"mdp_ahb",
	"mdp_axi",
	"mdp",
	"ocmem",
};

static const u16 mas_apps_proc_links[] = {
	MSM8994_BIMC_INT_APPS_SNOC,
	MSM8994_BIMC_INT_APPS_EBI
};

static struct qcom_icc_node mas_apps_proc = {
	.name = "mas_apps_proc",
	.id = MSM8994_MASTER_AMPSS_M0,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = -1,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_FIXED,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = 0,
	.num_links = ARRAY_SIZE(mas_apps_proc_links),
	.links = mas_apps_proc_links
};

static const u16 bimc_int_apps_ebi_links[] = {
	MSM8994_SLAVE_EBI_CH0,
};

static struct qcom_icc_node bimc_int_apps_ebi = {
	.name = "bimc_int_apps_ebi",
	.id = MSM8994_BIMC_INT_APPS_EBI,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = -1,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = 0,
	.num_links = ARRAY_SIZE(bimc_int_apps_ebi_links),
	.links = bimc_int_apps_ebi_links
};

static const u16 bimc_int_apps_snoc_links[] = {
	MSM8994_SLAVE_BIMC_SNOC,
};

static struct qcom_icc_node bimc_int_apps_snoc = {
	.name = "bimc_int_apps_snoc",
	.id = MSM8994_BIMC_INT_APPS_SNOC,
	.buswidth = 8,
	.mas_rpm_id = 1,
	.slv_rpm_id = -1,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = 0,
	.num_links = ARRAY_SIZE(bimc_int_apps_snoc_links),
	.links = bimc_int_apps_snoc_links
};

static const u16 mas_oxili_links[] = {
	MSM8994_SLAVE_BIMC_SNOC,
	MSM8994_SLAVE_AMPSS_L2,
	MSM8994_SLAVE_EBI_CH0
};

static struct qcom_icc_node mas_oxili = {
	.name = "mas_oxili",
	.id = MSM8994_MASTER_GRAPHICS_3D,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = -1,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_FIXED,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = 2,
	.num_links = ARRAY_SIZE(mas_oxili_links),
	.links = mas_oxili_links
};

static const u16 mas_mnoc_bimc_links[] = {
	MSM8994_SLAVE_AMPSS_L2,
	MSM8994_SLAVE_BIMC_SNOC,
	MSM8994_SLAVE_EBI_CH0
};

static struct qcom_icc_node mas_mnoc_bimc = {
	.name = "mas_mnoc_bimc",
	.id = MSM8994_MASTER_MNOC_BIMC,
	.buswidth = 16,
	.mas_rpm_id = -1,
	.slv_rpm_id = -1,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_BYPASS,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = 3,
	.num_links = ARRAY_SIZE(mas_mnoc_bimc_links),
	.links = mas_mnoc_bimc_links
};

static const u16 mas_snoc_bimc_links[] = {
	MSM8994_SLAVE_AMPSS_L2,
	MSM8994_SLAVE_EBI_CH0
};

static struct qcom_icc_node mas_snoc_bimc = {
	.name = "mas_snoc_bimc",
	.id = MSM8994_MASTER_SNOC_BIMC,
	.buswidth = 8,
	.mas_rpm_id = 3,
	.slv_rpm_id = -1,
	.qos.qos_mode = NOC_QOS_MODE_BYPASS,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = 4,
	.num_links = ARRAY_SIZE(mas_snoc_bimc_links),
	.links = mas_snoc_bimc_links
};

static struct qcom_icc_node slv_ebi = {
	.name = "slv_ebi",
	.id = MSM8994_SLAVE_EBI_CH0,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 0,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = 0, /* [0, 1] */
};

static struct qcom_icc_node slv_appss_l2 = {
	.name = "slv_appss_l2",
	.id = MSM8994_SLAVE_AMPSS_L2,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 1,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
};

static const u16 slv_bimc_snoc_links[] = {
	MSM8994_MASTER_BIMC_SNOC,
};

static struct qcom_icc_node slv_bimc_snoc = {
	.name = "slv_bimc_snoc",
	.id = MSM8994_SLAVE_BIMC_SNOC,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 2,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(slv_bimc_snoc_links),
	.links = slv_bimc_snoc_links
};

static const u16 mas_cnoc_mnoc_mmss_cfg_links[] = {
	MSM8994_SLAVE_CPR_CFG,
	MSM8994_SLAVE_AVSYNC_CFG,
	MSM8994_SLAVE_MNOC_MPU_CFG,
	MSM8994_SLAVE_VPU_CFG,
	MSM8994_SLAVE_MMSS_CLK_CFG,
	MSM8994_SLAVE_CAMERA_CFG,
	MSM8994_SLAVE_OCMEM_CFG,
	MSM8994_SLAVE_GRAPHICS_3D_CFG,
	MSM8994_SLAVE_DISPLAY_CFG,
	MSM8994_SLAVE_MISC_XPU_CFG,
	MSM8994_SLAVE_MISC_CFG,
	MSM8994_SLAVE_MISC_VENUS_CFG,
	MSM8994_SLAVE_MMSS_CLK_XPU_CFG,
	MSM8994_SLAVE_CPR_XPU_CFG
};

static struct qcom_icc_node mas_cnoc_mnoc_mmss_cfg = {
	.name = "mas_cnoc_mnoc_mmss_cfg",
	.id = MSM8994_MASTER_CNOC_MNOC_MMSS_CFG,
	.buswidth = 16,
	.mas_rpm_id = -1,
	.slv_rpm_id = -1,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(mas_cnoc_mnoc_mmss_cfg_links),
	.links = mas_cnoc_mnoc_mmss_cfg_links
};

static const u16 mas_cnoc_mnoc_cfg_links[] = {
	MSM8994_SLAVE_SRVC_MNOC,
};

static struct qcom_icc_node mas_cnoc_mnoc_cfg = {
	.name = "mas_cnoc_mnoc_cfg",
	.id = MSM8994_MASTER_CNOC_MNOC_CFG,
	.buswidth = 16,
	.mas_rpm_id = 5,
	.slv_rpm_id = -1,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(mas_cnoc_mnoc_cfg_links),
	.links = mas_cnoc_mnoc_cfg_links
};

static const u16 mas_jpeg_links[] = {
	MSM8994_SLAVE_MNOC_BIMC,
};

static struct qcom_icc_node mas_jpeg = {
	.name = "mas_jpeg",
	.id = MSM8994_MASTER_JPEG,
	.buswidth = 16,
	.mas_rpm_id = -1,
	.slv_rpm_id = -1,
	// .qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_BYPASS,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = 0,
	.num_links = ARRAY_SIZE(mas_jpeg_links),
	.links = mas_jpeg_links
};

static const u16 mas_mdp_p0_links[] = {
	MSM8994_SLAVE_MNOC_BIMC,
};

static struct qcom_icc_node mas_mdp_p0 = {
	.name = "mas_mdp_p0",
	.id = MSM8994_MASTER_MDP_PORT0,
	.buswidth = 16,
	.mas_rpm_id = -1,
	.slv_rpm_id = -1,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_BYPASS,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = 2,
	.num_links = ARRAY_SIZE(mas_mdp_p0_links),
	.links = mas_mdp_p0_links
};

static const u16 mas_mdp_p1_links[] = {
	MSM8994_SLAVE_MNOC_BIMC,
};

static struct qcom_icc_node mas_mdp_p1 = {
	.name = "mas_mdp_p1",
	.id = MSM8994_MASTER_MDP_PORT1,
	.buswidth = 16,
	.mas_rpm_id = -1,
	.slv_rpm_id = -1,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_BYPASS,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = 1,
	.num_links = ARRAY_SIZE(mas_mdp_p1_links),
	.links = mas_mdp_p1_links
};

static const u16 mas_venus_p0_links[] = {
	MSM8994_SLAVE_MNOC_BIMC,
};

static struct qcom_icc_node mas_venus_p0 = {
	.name = "mas_venus_p0",
	.id = MSM8994_MASTER_VIDEO_P0,
	.buswidth = 16,
	.mas_rpm_id = -1,
	.slv_rpm_id = -1,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_BYPASS,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = 3,
	.num_links = ARRAY_SIZE(mas_venus_p0_links),
	.links = mas_venus_p0_links
};

static const u16 mas_venus_p1_links[] = {
	MSM8994_SLAVE_MNOC_BIMC,
};

static struct qcom_icc_node mas_venus_p1 = {
	.name = "mas_venus_p1",
	.id = MSM8994_MASTER_VIDEO_P1,
	.buswidth = 16,
	.mas_rpm_id = -1,
	.slv_rpm_id = -1,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_BYPASS,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = 4,
	.num_links = ARRAY_SIZE(mas_venus_p1_links),
	.links = mas_venus_p1_links
};

static const u16 mas_vfe_links[] = {
	MSM8994_SLAVE_MNOC_BIMC,
};

static struct qcom_icc_node mas_vfe = {
	.name = "mas_vfe",
	.id = MSM8994_MASTER_VFE,
	.buswidth = 16,
	.mas_rpm_id = -1,
	.slv_rpm_id = -1,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_BYPASS,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = 6,
	.num_links = ARRAY_SIZE(mas_vfe_links),
	.links = mas_vfe_links
};

static const u16 mas_cpp_links[] = {
	MSM8994_SLAVE_MNOC_BIMC,
};

static struct qcom_icc_node mas_cpp = {
	.name = "mas_cpp",
	.id = MSM8994_MASTER_CPP,
	.buswidth = 16,
	.mas_rpm_id = -1,
	.slv_rpm_id = -1,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_BYPASS,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = 5,
	.num_links = ARRAY_SIZE(mas_cpp_links),
	.links = mas_cpp_links
};

static const u16 mas_vpu_links[] = {
	MSM8994_SLAVE_MNOC_BIMC,
};

static struct qcom_icc_node mas_vpu = {
	.name = "mas_vpu",
	.id = MSM8994_MASTER_VPU,
	.buswidth = 16,
	.mas_rpm_id = -1,
	.slv_rpm_id = -1,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_BYPASS,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = 8,
	.num_links = ARRAY_SIZE(mas_vpu_links),
	.links = mas_vpu_links
};

static struct qcom_icc_node slv_camera_cfg = {
	.name = "slv_camera_cfg",
	.id = MSM8994_SLAVE_CAMERA_CFG,
	.buswidth = 16,
	.mas_rpm_id = -1,
	.slv_rpm_id = 3,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
};

static struct qcom_icc_node slv_cpr_cfg = {
	.name = "slv_cpr_cfg",
	.id = MSM8994_SLAVE_CPR_CFG,
	.buswidth = 16,
	.mas_rpm_id = -1,
	.slv_rpm_id = 6,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
};

static struct qcom_icc_node slv_cpr_xpu_cfg = {
	.name = "slv_cpr_xpu_cfg",
	.id = MSM8994_SLAVE_CPR_XPU_CFG,
	.buswidth = 16,
	.mas_rpm_id = -1,
	.slv_rpm_id = 7,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
};

static struct qcom_icc_node slv_ocmem_cfg = {
	.name = "slv_ocmem_cfg",
	.id = MSM8994_SLAVE_OCMEM_CFG,
	.buswidth = 16,
	.mas_rpm_id = -1,
	.slv_rpm_id = 5,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
};

static struct qcom_icc_node slv_misc_cfg = {
	.name = "slv_misc_cfg",
	.id = MSM8994_SLAVE_MISC_CFG,
	.buswidth = 16,
	.mas_rpm_id = -1,
	.slv_rpm_id = 8,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
};

static struct qcom_icc_node slv_misc_xpu_cfg = {
	.name = "slv_misc_xpu_cfg",
	.id = MSM8994_SLAVE_MISC_XPU_CFG,
	.buswidth = 16,
	.mas_rpm_id = -1,
	.slv_rpm_id = 9,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
};

static struct qcom_icc_node slv_oxili_cfg = {
	.name = "slv_oxili_cfg",
	.id = MSM8994_SLAVE_GRAPHICS_3D_CFG,
	.buswidth = 16,
	.mas_rpm_id = -1,
	.slv_rpm_id = 11,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
};

static struct qcom_icc_node slv_venus_cfg = {
	.name = "slv_venus_cfg",
	.id = MSM8994_SLAVE_MISC_VENUS_CFG,
	.buswidth = 16,
	.mas_rpm_id = -1,
	.slv_rpm_id = 10,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
};

static struct qcom_icc_node slv_mnoc_clocks_cfg = {
	.name = "slv_mnoc_clocks_cfg",
	.id = MSM8994_SLAVE_MMSS_CLK_CFG,
	.buswidth = 16,
	.mas_rpm_id = -1,
	.slv_rpm_id = 12,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
};

static struct qcom_icc_node slv_mnoc_clocks_xpu_cfg = {
	.name = "slv_mnoc_clocks_xpu_cfg",
	.id = MSM8994_SLAVE_MMSS_CLK_XPU_CFG,
	.buswidth = 16,
	.mas_rpm_id = -1,
	.slv_rpm_id = 13,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
};

static struct qcom_icc_node slv_mnoc_mpu_cfg = {
	.name = "slv_mnoc_mpu_cfg",
	.id = MSM8994_SLAVE_MNOC_MPU_CFG,
	.buswidth = 16,
	.mas_rpm_id = -1,
	.slv_rpm_id = 14,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
};

static struct qcom_icc_node slv_display_cfg = {
	.name = "slv_display_cfg",
	.id = MSM8994_SLAVE_DISPLAY_CFG,
	.buswidth = 16,
	.mas_rpm_id = -1,
	.slv_rpm_id = 4,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
};

static struct qcom_icc_node slv_avsync_cfg = {
	.name = "slv_avsync_cfg",
	.id = MSM8994_SLAVE_AVSYNC_CFG,
	.buswidth = 16,
	.mas_rpm_id = -1,
	.slv_rpm_id = 93,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
};

static struct qcom_icc_node slv_vpu_cfg = {
	.name = "slv_vpu_cfg",
	.id = MSM8994_SLAVE_VPU_CFG,
	.buswidth = 16,
	.mas_rpm_id = -1,
	.slv_rpm_id = 94,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
};

static const u16 slv_mnoc_bimc_links[] = {
	MSM8994_MASTER_MNOC_BIMC,
};

static struct qcom_icc_node slv_mnoc_bimc = {
	.name = "slv_mnoc_bimc",
	.id = MSM8994_SLAVE_MNOC_BIMC,
	.buswidth = 16,
	.mas_rpm_id = -1,
	.slv_rpm_id = -1,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = 16, /* [16, 17] */
	.num_links = ARRAY_SIZE(slv_mnoc_bimc_links),
	.links = slv_mnoc_bimc_links
};

static struct qcom_icc_node slv_srvc_mnoc = {
	.name = "slv_srvc_mnoc",
	.id = MSM8994_SLAVE_SRVC_MNOC,
	.buswidth = 16,
	.mas_rpm_id = -1,
	.slv_rpm_id = 17,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
};

static const u16 mas_lpass_ahb_links[] = {
	MSM8994_SLAVE_OCIMEM,
	MSM8994_SLAVE_SNOC_BIMC,
	MSM8994_SLAVE_SNOC_PNOC,
};

static struct qcom_icc_node mas_lpass_ahb = {
	.name = "mas_lpass_ahb",
	.id = MSM8994_MASTER_LPASS_AHB,
	.buswidth = 8,
	.mas_rpm_id = 18,
	.slv_rpm_id = -1,
	.qos.qos_mode = NOC_QOS_MODE_FIXED,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = 0,
	.num_links = ARRAY_SIZE(mas_lpass_ahb_links),
	.links = mas_lpass_ahb_links
};

static const u16 mas_qdss_bam_links[] = {
	MSM8994_SLAVE_SNOC_BIMC,
	MSM8994_SLAVE_SNOC_PNOC,
	MSM8994_SLAVE_USB3,
};

static struct qcom_icc_node mas_qdss_bam = {
	.name = "mas_qdss_bam",
	.id = MSM8994_MASTER_QDSS_BAM,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = -1,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_FIXED,
	.qos.areq_prio = 1,
	.qos.prio_level = 1,
	.qos.qos_port = 1,
	.num_links = ARRAY_SIZE(mas_qdss_bam_links),
	.links = mas_qdss_bam_links
};

static const u16 mas_bimc_snoc_links[] = {
	MSM8994_SLAVE_SNOC_OVNOC,
	MSM8994_SLAVE_QDSS_STM,
	MSM8994_SLAVE_LPASS,
	MSM8994_SLAVE_SNOC_CNOC,
	MSM8994_SLAVE_AMPSS,
	MSM8994_SLAVE_OCIMEM,
	MSM8994_SLAVE_SNOC_PNOC
};

static struct qcom_icc_node mas_bimc_snoc = {
	.name = "mas_bimc_snoc",
	.id = MSM8994_MASTER_BIMC_SNOC,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = -1,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(mas_bimc_snoc_links),
	.links = mas_bimc_snoc_links
};

static const u16 mas_cnoc_snoc_links[] = {
	MSM8994_SLAVE_SNOC_BIMC,
	MSM8994_SLAVE_SNOC_OVNOC,
	MSM8994_SLAVE_QDSS_STM,
	MSM8994_SLAVE_LPASS,
	MSM8994_SLAVE_AMPSS,
	MSM8994_SLAVE_OCIMEM,
	MSM8994_SLAVE_SNOC_PNOC
};

static struct qcom_icc_node mas_cnoc_snoc = {
	.name = "mas_cnoc_snoc",
	.id = MSM8994_MASTER_CNOC_SNOC,
	.buswidth = 8,
	.mas_rpm_id = 22,
	.slv_rpm_id = -1,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(mas_cnoc_snoc_links),
	.links = mas_cnoc_snoc_links
};

static const u16 mas_crypto_c0_links[] = {
	MSM8994_SLAVE_SNOC_BIMC,
	MSM8994_SLAVE_SNOC_OVNOC,
	MSM8994_SLAVE_LPASS,
	MSM8994_SLAVE_USB3,
	MSM8994_SLAVE_OCIMEM,
	MSM8994_SLAVE_SNOC_PNOC
};

static struct qcom_icc_node mas_crypto_c0 = {
	.name = "mas_crypto_c0",
	.id = MSM8994_MASTER_CRYPTO_CORE0,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = -1,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_FIXED,
	.qos.areq_prio = 1,
	.qos.prio_level = 1,
	.qos.qos_port = 2,
	.num_links = ARRAY_SIZE(mas_crypto_c0_links),
	.links = mas_crypto_c0_links
};

static const u16 mas_crypto_c1_links[] = {
	MSM8994_SLAVE_SNOC_BIMC,
	MSM8994_SLAVE_SNOC_OVNOC,
	MSM8994_SLAVE_LPASS,
	MSM8994_SLAVE_USB3,
	MSM8994_SLAVE_OCIMEM,
	MSM8994_SLAVE_SNOC_PNOC
};

static struct qcom_icc_node mas_crypto_c1 = {
	.name = "mas_crypto_c1",
	.id = MSM8994_MASTER_CRYPTO_CORE1,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = -1,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_FIXED,
	.qos.areq_prio = 1,
	.qos.prio_level = 1,
	.qos.qos_port = 3,
	.num_links = ARRAY_SIZE(mas_crypto_c1_links),
	.links = mas_crypto_c1_links
};

static const u16 mas_crypto_c2_links[] = {
	MSM8994_SLAVE_SNOC_BIMC,
	MSM8994_SLAVE_SNOC_OVNOC,
	MSM8994_SLAVE_LPASS,
	MSM8994_SLAVE_USB3,
	MSM8994_SLAVE_OCIMEM,
	MSM8994_SLAVE_SNOC_PNOC
};

static struct qcom_icc_node mas_crypto_c2 = {
	.name = "mas_crypto_c2",
	.id = MSM8994_MASTER_CRYPTO_CORE2,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = -1,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_FIXED,
	.qos.areq_prio = 1,
	.qos.prio_level = 1,
	.qos.qos_port = 9,
	.num_links = ARRAY_SIZE(mas_crypto_c2_links),
	.links = mas_crypto_c2_links
};

static const u16 mas_lpass_proc_links[] = {
	MSM8994_SLAVE_SNOC_BIMC,
	MSM8994_SLAVE_SNOC_OVNOC,
	MSM8994_SLAVE_QDSS_STM,
	MSM8994_SLAVE_SNOC_CNOC,
	MSM8994_SLAVE_OCIMEM,
	MSM8994_SLAVE_SNOC_PNOC,
};

static struct qcom_icc_node mas_lpass_proc = {
	.name = "mas_lpass_proc",
	.id = MSM8994_MASTER_LPASS_PROC,
	.buswidth = 8,
	.mas_rpm_id = 25,
	.slv_rpm_id = -1,
	.qos.qos_mode = NOC_QOS_MODE_FIXED,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = 4,
	.num_links = ARRAY_SIZE(mas_lpass_proc_links),
	.links = mas_lpass_proc_links
};

static const u16 mas_ovirt_snoc_links[] = {
	MSM8994_SLAVE_SNOC_BIMC,
};

static struct qcom_icc_node mas_ovirt_snoc = {
	.name = "mas_ovirt_snoc",
	.id = MSM8994_MASTER_OVNOC_SNOC,
	.buswidth = 8,
	.mas_rpm_id = 54,
	.slv_rpm_id = -1,
	.qos.qos_mode = NOC_QOS_MODE_FIXED,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = 5,
	.num_links = ARRAY_SIZE(mas_ovirt_snoc_links),
	.links = mas_ovirt_snoc_links
};

static const u16 mas_periph_snoc_links[] = {
	MSM8994_SLAVE_SNOC_BIMC,
	MSM8994_SLAVE_QDSS_STM,
	MSM8994_SLAVE_LPASS,
	MSM8994_SLAVE_SNOC_CNOC,
	MSM8994_SLAVE_OCIMEM,
};

static struct qcom_icc_node mas_periph_snoc = {
	.name = "mas_periph_snoc",
	.id = MSM8994_MASTER_PNOC_SNOC,
	.buswidth = 8,
	.mas_rpm_id = 29,
	.slv_rpm_id = -1,
	.qos.qos_mode = NOC_QOS_MODE_FIXED,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = 13,
	.num_links = ARRAY_SIZE(mas_periph_snoc_links),
	.links = mas_periph_snoc_links
};

static const u16 mas_pcie_0_links[] = {
	MSM8994_SLAVE_OCIMEM,
	MSM8994_SLAVE_SNOC_BIMC,
	MSM8994_SLAVE_AMPSS,
};

static struct qcom_icc_node mas_pcie_0 = {
	.name = "mas_pcie_0",
	.id = MSM8994_MASTER_PCIE,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = -1,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_FIXED,
	.qos.areq_prio = 1,
	.qos.prio_level = 1,
	.qos.qos_port = 7,
	.num_links = ARRAY_SIZE(mas_pcie_0_links),
	.links = mas_pcie_0_links
};

static const u16 mas_pcie_1_links[] = {
	MSM8994_SLAVE_OCIMEM,
	MSM8994_SLAVE_SNOC_BIMC,
	MSM8994_SLAVE_AMPSS,
};

static struct qcom_icc_node mas_pcie_1 = {
	.name = "mas_pcie_1",
	.id = MSM8994_MASTER_PCIE_1,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = -1,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_FIXED,
	.qos.areq_prio = 1,
	.qos.prio_level = 1,
	.qos.qos_port = 16,
	.num_links = ARRAY_SIZE(mas_pcie_1_links),
	.links = mas_pcie_1_links
};

static const u16 mas_qdss_etr_links[] = {
	MSM8994_SLAVE_SNOC_BIMC,
	MSM8994_SLAVE_SNOC_PNOC,
};

static struct qcom_icc_node mas_qdss_etr = {
	.name = "mas_qdss_etr",
	.id = MSM8994_MASTER_QDSS_ETR,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = -1,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_FIXED,
	.qos.areq_prio = 1,
	.qos.prio_level = 1,
	.qos.qos_port = 8,
	.num_links = ARRAY_SIZE(mas_qdss_etr_links),
	.links = mas_qdss_etr_links
};

static const u16 mas_ufs_links[] = {
	MSM8994_SLAVE_SNOC_BIMC,
	MSM8994_SLAVE_SNOC_OVNOC,
	MSM8994_SLAVE_SNOC_CNOC,
	MSM8994_SLAVE_OCIMEM,
};

static struct qcom_icc_node mas_ufs = {
	.name = "mas_ufs",
	.id = MSM8994_MASTER_UFS,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = -1,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_FIXED,
	.qos.areq_prio = 1,
	.qos.prio_level = 1,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(mas_ufs_links),
	.links = mas_ufs_links
};

static const u16 mas_usb3_links[] = {
	MSM8994_SLAVE_SNOC_BIMC,
	MSM8994_SLAVE_LPASS,
	MSM8994_SLAVE_SNOC_CNOC,
	MSM8994_SLAVE_USB3,
	MSM8994_SLAVE_OCIMEM,
	MSM8994_SLAVE_SNOC_PNOC,
};

static struct qcom_icc_node mas_usb3 = {
	.name = "mas_usb3",
	.id = MSM8994_MASTER_USB3,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = -1,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_FIXED,
	.qos.areq_prio = 1,
	.qos.prio_level = 1,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(mas_usb3_links),
	.links = mas_usb3_links
};

static const u16 mas_ipa_links[] = {
	MSM8994_SLAVE_SNOC_BIMC,
	MSM8994_SLAVE_QDSS_STM,
	MSM8994_SLAVE_SNOC_CNOC,
	MSM8994_SLAVE_SNOC_PNOC,
	MSM8994_SLAVE_USB3,
	MSM8994_SLAVE_OCIMEM,
	MSM8994_SLAVE_PCIE_0,
};

static struct qcom_icc_node mas_ipa = {
	.name = "mas_ipa",
	.id = MSM8994_MASTER_IPA,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = -1,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_FIXED,
	.qos.areq_prio = 1,
	.qos.prio_level = 1,
	.qos.qos_port = 15,
	.num_links = ARRAY_SIZE(mas_ipa_links),
	.links = mas_ipa_links
};

static struct qcom_icc_node slv_kpss_ahb = {
	.name = "slv_kpss_ahb",
	.id = MSM8994_SLAVE_AMPSS,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 20,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
};

static struct qcom_icc_node slv_lpass = {
	.name = "slv_lpass",
	.id = MSM8994_SLAVE_LPASS,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 21,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
};

static const u16 slv_snoc_bimc_links[] = {
	MSM8994_MASTER_SNOC_BIMC,
};

static struct qcom_icc_node slv_snoc_bimc = {
	.name = "slv_snoc_bimc",
	.id = MSM8994_SLAVE_SNOC_BIMC,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 24,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(slv_snoc_bimc_links),
	.links = slv_snoc_bimc_links
};

static const u16 slv_snoc_cnoc_links[] = {
	MSM8994_MASTER_SNOC_CNOC,
};

static struct qcom_icc_node slv_snoc_cnoc = {
	.name = "slv_snoc_cnoc",
	.id = MSM8994_SLAVE_SNOC_CNOC,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 25,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(slv_snoc_cnoc_links),
	.links = slv_snoc_cnoc_links
};

static struct qcom_icc_node slv_ocimem = {
	.name = "slv_ocimem",
	.id = MSM8994_SLAVE_OCIMEM,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 26,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
};

static const u16 slv_snoc_ovirt_links[] = {
	MSM8994_MASTER_SNOC_OVNOC,
};

static struct qcom_icc_node slv_snoc_ovirt = {
	.name = "slv_snoc_ovirt",
	.id = MSM8994_SLAVE_SNOC_OVNOC,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 27,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(slv_snoc_ovirt_links),
	.links = slv_snoc_ovirt_links
};

static const u16 slv_snoc_pnoc_links[] = {
	MSM8994_MASTER_SNOC_PNOC,
};

static struct qcom_icc_node slv_snoc_pnoc = {
	.name = "slv_snoc_pnoc",
	.id = MSM8994_SLAVE_SNOC_PNOC,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 28,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(slv_snoc_pnoc_links),
	.links = slv_snoc_pnoc_links
};

static struct qcom_icc_node slv_qdss_stm = {
	.name = "slv_qdss_stm",
	.id = MSM8994_SLAVE_QDSS_STM,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 30,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
};

static struct qcom_icc_node slv_usb3 = {
	.name = "slv_usb3",
	.id = MSM8994_SLAVE_USB3,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 22,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
};

static struct qcom_icc_node slv_pcie_0 = {
	.name = "slv_pcie_0",
	.id = MSM8994_SLAVE_PCIE_0,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 84,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
};

static struct qcom_icc_node slv_pcie_1 = {
	.name = "slv_pcie_1",
	.id = MSM8994_SLAVE_PCIE_1,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 85,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
};

static const u16 mas_snoc_ovirt_links[] = {
	MSM8994_SLAVE_OCMEM,
};

static struct qcom_icc_node mas_snoc_ovirt = {
	.name = "mas_snoc_ovirt",
	.id = MSM8994_MASTER_SNOC_OVNOC,
	.buswidth = 32,
	.mas_rpm_id = -1,
	.slv_rpm_id = -1,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(mas_snoc_ovirt_links),
	.links = mas_snoc_ovirt_links
};

static const u16 mas_ocmem_dma_links[] = {
	MSM8994_SLAVE_OVNOC_SNOC,
	MSM8994_SLAVE_OCMEM,
};

static struct qcom_icc_node mas_ocmem_dma = {
	.name = "mas_ocmem_dma",
	.id = MSM8994_MASTER_OCMEM_DMA,
	.buswidth = 32,
	.mas_rpm_id = -1,
	.slv_rpm_id = -1,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(mas_ocmem_dma_links),
	.links = mas_ocmem_dma_links
};

static const u16 mas_oxili_ocmem_links[] = {
	MSM8994_SLAVE_OCMEM_GFX,
};

static struct qcom_icc_node mas_oxili_ocmem = {
	.name = "mas_oxili_ocmem",
	.id = MSM8994_MASTER_V_OCMEM_GFX3D,
	.buswidth = 32,
	.mas_rpm_id = -1,
	.slv_rpm_id = -1,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(mas_oxili_ocmem_links),
	.links = mas_oxili_ocmem_links
};

static const u16 mas_venus_ocmem_links[] = {
	MSM8994_SLAVE_OCMEM,
};

static struct qcom_icc_node mas_venus_ocmem = {
	.name = "mas_venus_ocmem",
	.id = MSM8994_MASTER_VIDEO_P0_OCMEM,
	.buswidth = 32,
	.mas_rpm_id = -1,
	.slv_rpm_id = -1,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(mas_venus_ocmem_links),
	.links = mas_venus_ocmem_links
};

static struct qcom_icc_node slv_ocmem = {
	.name = "slv_ocmem",
	.id = MSM8994_SLAVE_OCMEM,
	.buswidth = 16,
	.mas_rpm_id = -1,
	.slv_rpm_id = -1,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
};

static struct qcom_icc_node slv_ocmem_gfx = {
	.name = "slv_ocmem_gfx",
	.id = MSM8994_SLAVE_OCMEM_GFX,
	.buswidth = 32,
	.mas_rpm_id = -1,
	.slv_rpm_id = -1,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
};

static const u16 slv_ovirt_snoc_links[] = {
	MSM8994_MASTER_OVNOC_SNOC
};

static struct qcom_icc_node slv_ovirt_snoc = {
	.name = "slv_ovirt_snoc",
	.id = MSM8994_SLAVE_OVNOC_SNOC,
	.buswidth = 32,
	.mas_rpm_id = -1,
	.slv_rpm_id = 77,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(slv_ovirt_snoc_links),
	.links = slv_ovirt_snoc_links
};

static const u16 mas_bam_dma_links[] = {
	MSM8994_SLAVE_SDCC_4,
	MSM8994_SLAVE_SDCC_2,
	MSM8994_SLAVE_SDCC_3,
	MSM8994_SLAVE_USB_HS,
	MSM8994_SLAVE_SDCC_1,
	MSM8994_SLAVE_TSIF,
	MSM8994_SLAVE_BLSP_1,
	MSM8994_SLAVE_BLSP_2,
	MSM8994_SLAVE_PNOC_SNOC,
};

static struct qcom_icc_node mas_bam_dma = {
	.name = "mas_bam_dma",
	.id = MSM8994_MASTER_BAM_DMA,
	.buswidth = 8,
	.mas_rpm_id = 38,
	.slv_rpm_id = -1,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(mas_bam_dma_links),
	.links = mas_bam_dma_links
};

static const u16 mas_blsp_2_links[] = {
	MSM8994_SLAVE_SDCC_4,
	MSM8994_SLAVE_SDCC_2,
	MSM8994_SLAVE_SDCC_3,
	MSM8994_SLAVE_USB_HS,
	MSM8994_SLAVE_SDCC_1,
	MSM8994_SLAVE_TSIF,
	MSM8994_SLAVE_BLSP_1,
	MSM8994_SLAVE_BAM_DMA,
	MSM8994_SLAVE_PNOC_SNOC,
};

static struct qcom_icc_node mas_blsp_2 = {
	.name = "mas_blsp_2",
	.id = MSM8994_MASTER_BLSP_2,
	.buswidth = 8,
	.mas_rpm_id = 39,
	.slv_rpm_id = -1,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(mas_blsp_2_links),
	.links = mas_blsp_2_links
};

static const u16 mas_blsp_1_links[] = {
	MSM8994_SLAVE_SDCC_4,
	MSM8994_SLAVE_SDCC_2,
	MSM8994_SLAVE_SDCC_3,
	MSM8994_SLAVE_USB_HS,
	MSM8994_SLAVE_SDCC_1,
	MSM8994_SLAVE_TSIF,
	MSM8994_SLAVE_BAM_DMA,
	MSM8994_SLAVE_BLSP_2,
	MSM8994_SLAVE_PNOC_SNOC,
};

static struct qcom_icc_node mas_blsp_1 = {
	.name = "mas_blsp_1",
	.id = MSM8994_MASTER_BLSP_1,
	.buswidth = 8,
	.mas_rpm_id = 41,
	.slv_rpm_id = -1,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(mas_blsp_1_links),
	.links = mas_blsp_1_links
};

static const u16 mas_tsif_links[] = {
	MSM8994_SLAVE_SDCC_4,
	MSM8994_SLAVE_SDCC_2,
	MSM8994_SLAVE_SDCC_3,
	MSM8994_SLAVE_USB_HS,
	MSM8994_SLAVE_SDCC_1,
	MSM8994_SLAVE_BLSP_1,
	MSM8994_SLAVE_BAM_DMA,
	MSM8994_SLAVE_BLSP_2,
	MSM8994_SLAVE_PNOC_SNOC,
};

static struct qcom_icc_node mas_tsif = {
	.name = "mas_tsif",
	.id = MSM8994_MASTER_TSIF,
	.buswidth = 8,
	.mas_rpm_id = 37,
	.slv_rpm_id = -1,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(mas_tsif_links),
	.links = mas_tsif_links
};

static const u16 mas_usb_hs_links[] = {
	MSM8994_SLAVE_SDCC_4,
	MSM8994_SLAVE_SDCC_2,
	MSM8994_SLAVE_SDCC_3,
	MSM8994_SLAVE_SDCC_1,
	MSM8994_SLAVE_TSIF,
	MSM8994_SLAVE_BLSP_1,
	MSM8994_SLAVE_BAM_DMA,
	MSM8994_SLAVE_BLSP_2,
	MSM8994_SLAVE_PNOC_SNOC,
};

static struct qcom_icc_node mas_usb_hs = {
	.name = "mas_usb_hs",
	.id = MSM8994_MASTER_USB_HS,
	.buswidth = 8,
	.mas_rpm_id = 42,
	.slv_rpm_id = -1,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(mas_usb_hs_links),
	.links = mas_usb_hs_links
};

static const u16 mas_pnoc_cfg_links[] = {
	MSM8994_SLAVE_PRNG,
};

static struct qcom_icc_node mas_pnoc_cfg = {
	.name = "mas_pnoc_cfg",
	.id = MSM8994_MASTER_PNOC_CFG,
	.buswidth = 8,
	.mas_rpm_id = 43,
	.slv_rpm_id = -1,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(mas_pnoc_cfg_links),
	.links = mas_pnoc_cfg_links
};

static const u16 mas_sdcc_1_links[] = {
	MSM8994_SLAVE_SDCC_4,
	MSM8994_SLAVE_SDCC_2,
	MSM8994_SLAVE_SDCC_3,
	MSM8994_SLAVE_USB_HS,
	MSM8994_SLAVE_TSIF,
	MSM8994_SLAVE_BLSP_1,
	MSM8994_SLAVE_BAM_DMA,
	MSM8994_SLAVE_BLSP_2,
	MSM8994_SLAVE_PNOC_SNOC,
};

static struct qcom_icc_node mas_sdcc_1 = {
	.name = "mas_sdcc_1",
	.id = MSM8994_MASTER_SDCC_1,
	.buswidth = 8,
	.mas_rpm_id = 33,
	.slv_rpm_id = -1,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(mas_sdcc_1_links),
	.links = mas_sdcc_1_links
};

static const u16 mas_sdcc_2_links[] = {
	MSM8994_SLAVE_SDCC_4,
	MSM8994_SLAVE_SDCC_3,
	MSM8994_SLAVE_USB_HS,
	MSM8994_SLAVE_SDCC_1,
	MSM8994_SLAVE_TSIF,
	MSM8994_SLAVE_BLSP_1,
	MSM8994_SLAVE_BAM_DMA,
	MSM8994_SLAVE_BLSP_2,
	MSM8994_SLAVE_PNOC_SNOC,
};

static struct qcom_icc_node mas_sdcc_2 = {
	.name = "mas_sdcc_2",
	.id = MSM8994_MASTER_SDCC_2,
	.buswidth = 8,
	.mas_rpm_id = 35,
	.slv_rpm_id = -1,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(mas_sdcc_2_links),
	.links = mas_sdcc_2_links
};

static const u16 mas_sdcc_3_links[] = {
	MSM8994_SLAVE_SDCC_4,
	MSM8994_SLAVE_SDCC_2,
	MSM8994_SLAVE_BLSP_2,
	MSM8994_SLAVE_USB_HS,
	MSM8994_SLAVE_SDCC_1,
	MSM8994_SLAVE_TSIF,
	MSM8994_SLAVE_BLSP_1,
	MSM8994_SLAVE_BAM_DMA,
	MSM8994_SLAVE_PNOC_SNOC,
};

static struct qcom_icc_node mas_sdcc_3 = {
	.name = "mas_sdcc_3",
	.id = MSM8994_MASTER_SDCC_3,
	.buswidth = 8,
	.mas_rpm_id = 34,
	.slv_rpm_id = -1,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(mas_sdcc_3_links),
	.links = mas_sdcc_3_links
};

static const u16 mas_sdcc_4_links[] = {
	MSM8994_SLAVE_SDCC_2,
	MSM8994_SLAVE_SDCC_3,
	MSM8994_SLAVE_USB_HS,
	MSM8994_SLAVE_SDCC_1,
	MSM8994_SLAVE_TSIF,
	MSM8994_SLAVE_BLSP_1,
	MSM8994_SLAVE_BAM_DMA,
	MSM8994_SLAVE_BLSP_2,
	MSM8994_SLAVE_PNOC_SNOC,
};

static struct qcom_icc_node mas_sdcc_4 = {
	.name = "mas_sdcc_4",
	.id = MSM8994_MASTER_SDCC_4,
	.buswidth = 8,
	.mas_rpm_id = 36,
	.slv_rpm_id = -1,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(mas_sdcc_4_links),
	.links = mas_sdcc_4_links
};

static const u16 mas_snoc_pnoc_links[] = {
	MSM8994_SLAVE_SDCC_4,
	MSM8994_SLAVE_SDCC_2,
	MSM8994_SLAVE_SDCC_3,
	MSM8994_SLAVE_USB_HS,
	MSM8994_SLAVE_PRNG,
	MSM8994_SLAVE_SDCC_1,
	MSM8994_SLAVE_TSIF,
	MSM8994_SLAVE_PDM,
	MSM8994_SLAVE_BLSP_1,
	MSM8994_SLAVE_BAM_DMA,
	MSM8994_SLAVE_BLSP_2
};

static struct qcom_icc_node mas_snoc_pnoc = {
	.name = "mas_snoc_pnoc",
	.id = MSM8994_MASTER_SNOC_PNOC,
	.buswidth = 8,
	.mas_rpm_id = 44,
	.slv_rpm_id = -1,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(mas_snoc_pnoc_links),
	.links = mas_snoc_pnoc_links
};

static struct qcom_icc_node slv_bam_dma = {
	.name = "slv_bam_dma",
	.id = MSM8994_SLAVE_BAM_DMA,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 36,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
};

static struct qcom_icc_node slv_sdcc_1 = {
	.name = "slv_sdcc_1",
	.id = MSM8994_SLAVE_SDCC_1,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 31,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
};

static struct qcom_icc_node slv_sdcc_3 = {
	.name = "slv_sdcc_3",
	.id = MSM8994_SLAVE_SDCC_3,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 32,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
};

static struct qcom_icc_node slv_blsp_2 = {
	.name = "slv_blsp_2",
	.id = MSM8994_SLAVE_BLSP_2,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 37,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
};

static struct qcom_icc_node slv_sdcc_2 = {
	.name = "slv_sdcc_2",
	.id = MSM8994_SLAVE_SDCC_2,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 33,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
};

static struct qcom_icc_node slv_sdcc_4 = {
	.name = "slv_sdcc_4",
	.id = MSM8994_SLAVE_SDCC_4,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 34,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
};

static struct qcom_icc_node slv_blsp_1 = {
	.name = "slv_blsp_1",
	.id = MSM8994_SLAVE_BLSP_1,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 39,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
};

static struct qcom_icc_node slv_tsif = {
	.name = "slv_tsif",
	.id = MSM8994_SLAVE_TSIF,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 35,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
};

static struct qcom_icc_node slv_usb_hs = {
	.name = "slv_usb_hs",
	.id = MSM8994_SLAVE_USB_HS,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 40,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
};

static struct qcom_icc_node slv_pdm = {
	.name = "slv_pdm",
	.id = MSM8994_SLAVE_PDM,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 41,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
};

static struct qcom_icc_node slv_prng = {
	.name = "slv_prng",
	.id = MSM8994_SLAVE_PRNG,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 44,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
};

static const u16 slv_periph_snoc_links[] = {
	MSM8994_MASTER_PNOC_SNOC,
};

static struct qcom_icc_node slv_periph_snoc = {
	.name = "slv_periph_snoc",
	.id = MSM8994_SLAVE_PNOC_SNOC,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 45,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(slv_periph_snoc_links),
	.links = slv_periph_snoc_links
};

static const u16 mas_rpm_inst_links[] = {
	MSM8994_SLAVE_BOOT_ROM,
};

static struct qcom_icc_node mas_rpm_inst = {
	.name = "mas_rpm_inst",
	.id = MSM8994_MASTER_RPM_INST,
	.buswidth = 8,
	.mas_rpm_id = 45,
	.slv_rpm_id = -1,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(mas_rpm_inst_links),
	.links = mas_rpm_inst_links
};

static const u16 mas_rpm_sys_links[] = {
	MSM8994_SLAVE_PCIE_1_CFG,
	MSM8994_SLAVE_EBI1_PHY_CFG,
	MSM8994_SLAVE_PCIE_0_CFG,
	MSM8994_SLAVE_TLMM,
	MSM8994_SLAVE_PNOC_CFG,
	MSM8994_SLAVE_UFS_CFG,
	MSM8994_SLAVE_CLK_CTL,
	MSM8994_SLAVE_SECURITY,
	MSM8994_SLAVE_TCSR,
	MSM8994_SLAVE_BIMC_CFG,
	MSM8994_SLAVE_QDSS_CFG,
	MSM8994_SLAVE_PHY_APU_CFG,
	MSM8994_SLAVE_CRYPTO_0_CFG,
	MSM8994_SLAVE_SNOC_CFG,
	MSM8994_SLAVE_SNOC_MPU_CFG,
	MSM8994_SLAVE_CRYPTO_2_CFG,
	MSM8994_SLAVE_SPDM_WRAPPER,
	MSM8994_SLAVE_CRYPTO_1_CFG,
	MSM8994_SLAVE_CNOC_SNOC,
	MSM8994_SLAVE_DEHR_CFG,
	MSM8994_SLAVE_MPM,
	MSM8994_SLAVE_GENI_IR_CFG,
	MSM8994_SLAVE_IMEM_CFG,
	MSM8994_SLAVE_CNOC_MNOC_MMSS_CFG,
	MSM8994_SLAVE_EBI1_DLL_CFG,
	MSM8994_SLAVE_MESSAGE_RAM
};

static struct qcom_icc_node mas_rpm_sys = {
	.name = "mas_rpm_sys",
	.id = MSM8994_MASTER_RPM_SYS,
	.buswidth = 8,
	.mas_rpm_id = 47,
	.slv_rpm_id = -1,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(mas_rpm_sys_links),
	.links = mas_rpm_sys_links
};

static const u16 mas_dehr_links[] = {
	MSM8994_SLAVE_BIMC_CFG,
};

static struct qcom_icc_node mas_dehr = {
	.name = "mas_dehr",
	.id = MSM8994_MASTER_DEHR,
	.buswidth = 8,
	.mas_rpm_id = 48,
	.slv_rpm_id = -1,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(mas_dehr_links),
	.links = mas_dehr_links
};

static const u16 mas_spdm_links[] = {
	MSM8994_SLAVE_CNOC_SNOC,
};

static struct qcom_icc_node mas_spdm = {
	.name = "mas_spdm",
	.id = MSM8994_MASTER_SPDM,
	.buswidth = 8,
	.mas_rpm_id = 50,
	.slv_rpm_id = -1,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(mas_spdm_links),
	.links = mas_spdm_links
};

static const u16 mas_tic_links[] = {
	MSM8994_SLAVE_PCIE_1_CFG,
	MSM8994_SLAVE_EBI1_PHY_CFG,
	MSM8994_SLAVE_PCIE_0_CFG,
	MSM8994_SLAVE_BOOT_ROM,
	MSM8994_SLAVE_TLMM,
	MSM8994_SLAVE_RPM,
	MSM8994_SLAVE_PNOC_CFG,
	MSM8994_SLAVE_UFS_CFG,
	MSM8994_SLAVE_CLK_CTL,
	MSM8994_SLAVE_SECURITY,
	MSM8994_SLAVE_TCSR,
	MSM8994_SLAVE_BIMC_CFG,
	MSM8994_SLAVE_QDSS_CFG,
	MSM8994_SLAVE_PHY_APU_CFG,
	MSM8994_SLAVE_CRYPTO_0_CFG,
	MSM8994_SLAVE_SNOC_CFG,
	MSM8994_SLAVE_SNOC_MPU_CFG,
	MSM8994_SLAVE_CRYPTO_2_CFG,
	MSM8994_SLAVE_SPDM_WRAPPER,
	MSM8994_SLAVE_CRYPTO_1_CFG,
	MSM8994_SLAVE_CNOC_SNOC,
	MSM8994_SLAVE_DEHR_CFG,
	MSM8994_SLAVE_MPM,
	MSM8994_SLAVE_GENI_IR_CFG,
	MSM8994_SLAVE_IMEM_CFG,
	MSM8994_SLAVE_CNOC_MNOC_MMSS_CFG,
	MSM8994_SLAVE_EBI1_DLL_CFG,
	MSM8994_SLAVE_MESSAGE_RAM,
};

static struct qcom_icc_node mas_tic = {
	.name = "mas_tic",
	.id = MSM8994_MASTER_TIC,
	.buswidth = 8,
	.mas_rpm_id = 51,
	.slv_rpm_id = -1,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(mas_tic_links),
	.links = mas_tic_links
};

static const u16 mas_snoc_cnoc_links[] = {
	MSM8994_SLAVE_PCIE_1_CFG,
	MSM8994_SLAVE_EBI1_PHY_CFG,
	MSM8994_SLAVE_PCIE_0_CFG,
	MSM8994_SLAVE_BOOT_ROM,
	MSM8994_SLAVE_TLMM,
	MSM8994_SLAVE_RPM,
	MSM8994_SLAVE_PNOC_CFG,
	MSM8994_SLAVE_UFS_CFG,
	MSM8994_SLAVE_CLK_CTL,
	MSM8994_SLAVE_SECURITY,
	MSM8994_SLAVE_TCSR,
	MSM8994_SLAVE_BIMC_CFG,
	MSM8994_SLAVE_QDSS_CFG,
	MSM8994_SLAVE_PHY_APU_CFG,
	MSM8994_SLAVE_CRYPTO_0_CFG,
	MSM8994_SLAVE_SNOC_CFG,
	MSM8994_SLAVE_SNOC_MPU_CFG,
	MSM8994_SLAVE_CRYPTO_2_CFG,
	MSM8994_SLAVE_SPDM_WRAPPER,
	MSM8994_SLAVE_CRYPTO_1_CFG,
	MSM8994_SLAVE_DEHR_CFG,
	MSM8994_SLAVE_MPM,
	MSM8994_SLAVE_GENI_IR_CFG,
	MSM8994_SLAVE_IMEM_CFG,
	MSM8994_SLAVE_CNOC_MNOC_MMSS_CFG,
	MSM8994_SLAVE_EBI1_DLL_CFG,
	MSM8994_SLAVE_MESSAGE_RAM,
};

static struct qcom_icc_node mas_snoc_cnoc = {
	.name = "mas_snoc_cnoc",
	.id = MSM8994_MASTER_SNOC_CNOC,
	.buswidth = 8,
	.mas_rpm_id = 52,
	.slv_rpm_id = -1,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(mas_snoc_cnoc_links),
	.links = mas_snoc_cnoc_links
};

static const u16 mas_qdss_dap_links[] = {
	MSM8994_SLAVE_PCIE_1_CFG,
	MSM8994_SLAVE_EBI1_PHY_CFG,
	MSM8994_SLAVE_PCIE_0_CFG,
	MSM8994_SLAVE_BOOT_ROM,
	MSM8994_SLAVE_TLMM,
	MSM8994_SLAVE_RPM,
	MSM8994_SLAVE_PNOC_CFG,
	MSM8994_SLAVE_UFS_CFG,
	MSM8994_SLAVE_CLK_CTL,
	MSM8994_SLAVE_SECURITY,
	MSM8994_SLAVE_TCSR,
	MSM8994_SLAVE_BIMC_CFG,
	MSM8994_SLAVE_QDSS_CFG,
	MSM8994_SLAVE_PHY_APU_CFG,
	MSM8994_SLAVE_CRYPTO_0_CFG,
	MSM8994_SLAVE_SNOC_CFG,
	MSM8994_SLAVE_SNOC_MPU_CFG,
	MSM8994_SLAVE_CRYPTO_2_CFG,
	MSM8994_SLAVE_SPDM_WRAPPER,
	MSM8994_SLAVE_CRYPTO_1_CFG,
	MSM8994_SLAVE_CNOC_SNOC,
	MSM8994_SLAVE_DEHR_CFG,
	MSM8994_SLAVE_MPM,
	MSM8994_SLAVE_GENI_IR_CFG,
	MSM8994_SLAVE_IMEM_CFG,
	MSM8994_SLAVE_CNOC_MNOC_MMSS_CFG,
	MSM8994_SLAVE_EBI1_DLL_CFG,
	MSM8994_SLAVE_MESSAGE_RAM,
};

static struct qcom_icc_node mas_qdss_dap = {
	.name = "mas_qdss_dap",
	.id = MSM8994_MASTER_QDSS_DAP,
	.buswidth = 8,
	.mas_rpm_id = 49,
	.slv_rpm_id = -1,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(mas_qdss_dap_links),
	.links = mas_qdss_dap_links
};

static struct qcom_icc_node slv_clk_ctl = {
	.name = "slv_clk_ctl",
	.id = MSM8994_SLAVE_CLK_CTL,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 47,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
};

static struct qcom_icc_node slv_crypto_2_cfg = {
	.name = "slv_crypto_2_cfg",
	.id = MSM8994_SLAVE_CRYPTO_2_CFG,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 87,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
};

static struct qcom_icc_node slv_security = {
	.name = "slv_security",
	.id = MSM8994_SLAVE_SECURITY,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 49,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
};

static struct qcom_icc_node slv_tcsr = {
	.name = "slv_tcsr",
	.id = MSM8994_SLAVE_TCSR,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 50,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
};

static struct qcom_icc_node slv_tlmm = {
	.name = "slv_tlmm",
	.id = MSM8994_SLAVE_TLMM,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 51,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
};

static struct qcom_icc_node slv_crypto_0_cfg = {
	.name = "slv_crypto_0_cfg",
	.id = MSM8994_SLAVE_CRYPTO_0_CFG,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 52,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
};

static struct qcom_icc_node slv_crypto_1_cfg = {
	.name = "slv_crypto_1_cfg",
	.id = MSM8994_SLAVE_CRYPTO_1_CFG,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 53,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
};

static struct qcom_icc_node slv_imem_cfg = {
	.name = "slv_imem_cfg",
	.id = MSM8994_SLAVE_IMEM_CFG,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 54,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
};

static struct qcom_icc_node slv_message_ram = {
	.name = "slv_message_ram",
	.id = MSM8994_SLAVE_MESSAGE_RAM,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 55,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
};

static struct qcom_icc_node slv_bimc_cfg = {
	.name = "slv_bimc_cfg",
	.id = MSM8994_SLAVE_BIMC_CFG,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 56,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
};

static struct qcom_icc_node slv_boot_rom = {
	.name = "slv_boot_rom",
	.id = MSM8994_SLAVE_BOOT_ROM,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 57,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
};

static const u16 slv_cnoc_mnoc_mmss_cfg_links[] = {
	MSM8994_MASTER_CNOC_MNOC_MMSS_CFG,
};

static struct qcom_icc_node slv_cnoc_mnoc_mmss_cfg = {
	.name = "slv_cnoc_mnoc_mmss_cfg",
	.id = MSM8994_SLAVE_CNOC_MNOC_MMSS_CFG,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 58,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(slv_cnoc_mnoc_mmss_cfg_links),
	.links = slv_cnoc_mnoc_mmss_cfg_links
};

static struct qcom_icc_node slv_pmic_arb = {
	.name = "slv_pmic_arb",
	.id = MSM8994_SLAVE_PMIC_ARB,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 59,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
};

static struct qcom_icc_node slv_spdm_wrapper = {
	.name = "slv_spdm_wrapper",
	.id = MSM8994_SLAVE_SPDM_WRAPPER,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 60,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
};

static struct qcom_icc_node slv_dehr_cfg = {
	.name = "slv_dehr_cfg",
	.id = MSM8994_SLAVE_DEHR_CFG,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 61,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
};

static struct qcom_icc_node slv_mpm = {
	.name = "slv_mpm",
	.id = MSM8994_SLAVE_MPM,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 62,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
};

static struct qcom_icc_node slv_qdss_cfg = {
	.name = "slv_qdss_cfg",
	.id = MSM8994_SLAVE_QDSS_CFG,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 63,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
};

static struct qcom_icc_node slv_rbcpr_qdss_apu_cfg = {
	.name = "slv_rbcpr_qdss_apu_cfg",
	.id = MSM8994_SLAVE_RBCPR_QDSS_APU_CFG,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 65,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
};

static struct qcom_icc_node slv_rbcpr_cfg = {
	.name = "slv_rbcpr_cfg",
	.id = MSM8994_SLAVE_RBCPR_CFG,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 64,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
};

static const u16 slv_cnoc_mnoc_cfg_links[] = {
	MSM8994_MASTER_CNOC_MNOC_CFG,
};

static struct qcom_icc_node slv_cnoc_mnoc_cfg = {
	.name = "slv_cnoc_mnoc_cfg",
	.id = MSM8994_SLAVE_CNOC_MNOC_CFG,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 66,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(slv_cnoc_mnoc_cfg_links),
	.links = slv_cnoc_mnoc_cfg_links
};

static const u16 slv_pnoc_cfg_links[] = {
	MSM8994_MASTER_PNOC_CFG,
};

static struct qcom_icc_node slv_pnoc_cfg = {
	.name = "slv_pnoc_cfg",
	.id = MSM8994_SLAVE_PNOC_CFG,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 69,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(slv_pnoc_cfg_links),
	.links = slv_pnoc_cfg_links
};

static struct qcom_icc_node slv_snoc_cfg = {
	.name = "slv_snoc_cfg",
	.id = MSM8994_SLAVE_SNOC_CFG,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 70,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
};

static struct qcom_icc_node slv_snoc_mpu_cfg = {
	.name = "slv_snoc_mpu_cfg",
	.id = MSM8994_SLAVE_SNOC_MPU_CFG,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 67,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
};

static struct qcom_icc_node slv_ebi1_dll_cfg = {
	.name = "slv_ebi1_dll_cfg",
	.id = MSM8994_SLAVE_EBI1_DLL_CFG,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 71,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
};

static struct qcom_icc_node slv_phy_apu_cfg = {
	.name = "slv_phy_apu_cfg",
	.id = MSM8994_SLAVE_PHY_APU_CFG,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 72,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
};

static struct qcom_icc_node slv_ebi1_phy_cfg = {
	.name = "slv_ebi1_phy_cfg",
	.id = MSM8994_SLAVE_EBI1_PHY_CFG,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 73,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
};

static struct qcom_icc_node slv_rpm = {
	.name = "slv_rpm",
	.id = MSM8994_SLAVE_RPM,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 74,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
};

static struct qcom_icc_node slv_pcie_0_cfg = {
	.name = "slv_pcie_0_cfg",
	.id = MSM8994_SLAVE_PCIE_0_CFG,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 88,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
};

static struct qcom_icc_node slv_pcie_1_cfg = {
	.name = "slv_pcie_1_cfg",
	.id = MSM8994_SLAVE_PCIE_1_CFG,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 89,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
};

static struct qcom_icc_node slv_spss_geni_ir = {
	.name = "slv_spss_geni_ir",
	.id = MSM8994_SLAVE_GENI_IR_CFG,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 91,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
};

static struct qcom_icc_node slv_ufs_cfg = {
	.name = "slv_ufs_cfg",
	.id = MSM8994_SLAVE_UFS_CFG,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 92,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
};

static const u16 slv_cnoc_snoc_links[] = {
	MSM8994_MASTER_CNOC_SNOC,
};

static struct qcom_icc_node slv_cnoc_snoc = {
	.name = "slv_cnoc_snoc",
	.id = MSM8994_SLAVE_CNOC_SNOC,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 75,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(slv_cnoc_snoc_links),
	.links = slv_cnoc_snoc_links,
};

static struct qcom_icc_node * const snoc_nodes[] = {
	[MASTER_LPASS_AHB] = &mas_lpass_ahb,
	[MASTER_QDSS_BAM] = &mas_qdss_bam,
	[MASTER_BIMC_SNOC] = &mas_bimc_snoc,
	[MASTER_CNOC_SNOC] = &mas_cnoc_snoc,
	[MASTER_CRYPTO_CORE0] = &mas_crypto_c0,
	[MASTER_CRYPTO_CORE1] = &mas_crypto_c1,
	[MASTER_CRYPTO_CORE2] = &mas_crypto_c2,
	[MASTER_LPASS_PROC] = &mas_lpass_proc,
	[MASTER_OVNOC_SNOC] = &mas_ovirt_snoc,
	[MASTER_PNOC_SNOC] = &mas_periph_snoc,
	[MASTER_PCIE] = &mas_pcie_0,
	[MASTER_PCIE_1] = &mas_pcie_1,
	[MASTER_QDSS_ETR] = &mas_qdss_etr,
	[MASTER_UFS] = &mas_ufs,
	[MASTER_USB3] = &mas_usb3,
	[MASTER_IPA] = &mas_ipa,
	[SLAVE_AMPSS] = &slv_kpss_ahb,
	[SLAVE_LPASS] = &slv_lpass,
	[SLAVE_SNOC_BIMC] = &slv_snoc_bimc,
	[SLAVE_SNOC_CNOC] = &slv_snoc_cnoc,
	[SLAVE_OCIMEM] = &slv_ocimem,
	[SLAVE_SNOC_OVNOC] = &slv_snoc_ovirt,
	[SLAVE_SNOC_PNOC] = &slv_snoc_pnoc,
	[SLAVE_QDSS_STM] = &slv_qdss_stm,
	[SLAVE_USB3] = &slv_usb3,
	[SLAVE_PCIE_0] = &slv_pcie_0,
	[SLAVE_PCIE_1] = &slv_pcie_1,
};

static const struct regmap_config msm8994_snoc_regmap_config = {
	.reg_bits	= 32,
	.reg_stride	= 4,
	.val_bits	= 32,
	.max_register	= 0x5880,
	.fast_io	= true
};

static const struct qcom_icc_desc msm8994_snoc = {
	.nodes = snoc_nodes,
	.num_nodes = ARRAY_SIZE(snoc_nodes),
	.regmap_cfg = &msm8994_snoc_regmap_config,
	.type = QCOM_ICC_NOC,
	.qos_offset = 0x5000,
};

static struct qcom_icc_node * const bimc_nodes[] = {
	[MASTER_AMPSS_M0] = &mas_apps_proc,
	[BIMC_INT_APPS_EBI] = &bimc_int_apps_ebi,
	[BIMC_INT_APPS_SNOC] = &bimc_int_apps_snoc,
	[MASTER_GRAPHICS_3D] = &mas_oxili,
	[MASTER_MNOC_BIMC] = &mas_mnoc_bimc,
	[MASTER_SNOC_BIMC] = &mas_snoc_bimc,
	[SLAVE_EBI_CH0] = &slv_ebi,
	[SLAVE_AMPSS_L2] = &slv_appss_l2,
	[SLAVE_BIMC_SNOC] = &slv_bimc_snoc,
};

static const struct regmap_config msm8994_bimc_regmap_config = {
	.reg_bits	= 32,
	.reg_stride	= 4,
	.val_bits	= 32,
	.max_register	= 0x80000,
	.fast_io	= true
};

static const struct qcom_icc_desc msm8994_bimc = {
	.nodes = bimc_nodes,
	.num_nodes = ARRAY_SIZE(bimc_nodes),
	.regmap_cfg = &msm8994_bimc_regmap_config,
	.type = QCOM_ICC_BIMC,
	.qos_offset = 0x8000,
};

static struct qcom_icc_node * const pnoc_nodes[] = {
	[MASTER_BAM_DMA] = &mas_bam_dma,
	[MASTER_BLSP_2] = &mas_blsp_2,
	[MASTER_BLSP_1] = &mas_blsp_1,
	[MASTER_TSIF] = &mas_tsif,
	[MASTER_USB_HS] = &mas_usb_hs,
	[MASTER_PNOC_CFG] = &mas_pnoc_cfg,
	[MASTER_SDCC_1] = &mas_sdcc_1,
	[MASTER_SDCC_2] = &mas_sdcc_2,
	[MASTER_SDCC_3] = &mas_sdcc_3,
	[MASTER_SDCC_4] = &mas_sdcc_4,
	[MASTER_SNOC_PNOC] = &mas_snoc_pnoc,
	[SLAVE_BAM_DMA] = &slv_bam_dma,
	[SLAVE_SDCC_1] = &slv_sdcc_1,
	[SLAVE_SDCC_3] = &slv_sdcc_3,
	[SLAVE_BLSP_2] = &slv_blsp_2,
	[SLAVE_SDCC_2] = &slv_sdcc_2,
	[SLAVE_SDCC_4] = &slv_sdcc_4,
	[SLAVE_BLSP_1] = &slv_blsp_1,
	[SLAVE_TSIF] = &slv_tsif,
	[SLAVE_USB_HS] = &slv_usb_hs,
	[SLAVE_PDM] = &slv_pdm,
	[SLAVE_PRNG] = &slv_prng,
	[SLAVE_PNOC_SNOC] = &slv_periph_snoc,
};

static const struct regmap_config msm8994_pnoc_regmap_config = {
	.reg_bits	= 32,
	.reg_stride	= 4,
	.val_bits	= 32,
	.max_register	= 0x10000,
	.fast_io	= true
};

static const struct qcom_icc_desc msm8994_pnoc = {
	.nodes = pnoc_nodes,
	.num_nodes = ARRAY_SIZE(pnoc_nodes),
	.regmap_cfg = &msm8994_pnoc_regmap_config,
	.type = QCOM_ICC_NOC,
	.qos_offset = 0x3000,
};

static struct qcom_icc_node * const mnoc_nodes[] = {
	[MASTER_CNOC_MNOC_MMSS_CFG] = &mas_cnoc_mnoc_mmss_cfg,
	[MASTER_CNOC_MNOC_CFG] = &mas_cnoc_mnoc_cfg,
	[MASTER_JPEG] = &mas_jpeg,
	[MASTER_MDP_PORT0] = &mas_mdp_p0,
	[MASTER_MDP_PORT1] = &mas_mdp_p1,
	[MASTER_VIDEO_P0] = &mas_venus_p0,
	[MASTER_VIDEO_P1] = &mas_venus_p1,
	[MASTER_VFE] = &mas_vfe,
	[MASTER_CPP] = &mas_cpp,
	[MASTER_VPU] = &mas_vpu,
	[SLAVE_CAMERA_CFG] = &slv_camera_cfg,
	[SLAVE_CPR_CFG] = &slv_cpr_cfg,
	[SLAVE_CPR_XPU_CFG] = &slv_cpr_xpu_cfg,
	[SLAVE_OCMEM_CFG] = &slv_ocmem_cfg,
	[SLAVE_MISC_CFG] = &slv_misc_cfg,
	[SLAVE_MISC_XPU_CFG] = &slv_misc_xpu_cfg,
	[SLAVE_GRAPHICS_3D_CFG] = &slv_oxili_cfg,
	[SLAVE_MISC_VENUS_CFG] = &slv_venus_cfg,
	[SLAVE_MMSS_CLK_CFG] = &slv_mnoc_clocks_cfg,
	[SLAVE_MMSS_CLK_XPU_CFG] = &slv_mnoc_clocks_xpu_cfg,
	[SLAVE_MNOC_MPU_CFG] = &slv_mnoc_mpu_cfg,
	[SLAVE_DISPLAY_CFG] = &slv_display_cfg,
	[SLAVE_AVSYNC_CFG] = &slv_avsync_cfg,
	[SLAVE_VPU_CFG] = &slv_vpu_cfg,
	[SLAVE_MNOC_BIMC] = &slv_mnoc_bimc,
	[SLAVE_SRVC_MNOC] = &slv_srvc_mnoc,
};

static const struct regmap_config msm8994_mnoc_regmap_config = {
	.reg_bits	= 32,
	.reg_stride	= 4,
	.val_bits	= 32,
	.max_register	= 0x8000,
	.fast_io	= true
};

static const struct qcom_icc_desc msm8994_mnoc = {
	.nodes = mnoc_nodes,
	.num_nodes = ARRAY_SIZE(mnoc_nodes),
	.intf_clocks = mnoc_intf_clocks,
	.num_intf_clocks = ARRAY_SIZE(mnoc_intf_clocks),
	.regmap_cfg = &msm8994_mnoc_regmap_config,
	.type = QCOM_ICC_NOC,
	.qos_offset = 0x3000,
};

static struct qcom_icc_node * const cnoc_nodes[] = {
	[MASTER_RPM_INST] = &mas_rpm_inst,
	[MASTER_RPM_SYS] = &mas_rpm_sys,
	[MASTER_DEHR] = &mas_dehr,
	[MASTER_SPDM] = &mas_spdm,
	[MASTER_TIC] = &mas_tic,
	[MASTER_SNOC_CNOC] = &mas_snoc_cnoc,
	[MASTER_QDSS_DAP] = &mas_qdss_dap,
	[SLAVE_CLK_CTL] = &slv_clk_ctl,
	[SLAVE_CRYPTO_2_CFG] = &slv_crypto_2_cfg,
	[SLAVE_SECURITY] = &slv_security,
	[SLAVE_TCSR] = &slv_tcsr,
	[SLAVE_TLMM] = &slv_tlmm,
	[SLAVE_CRYPTO_0_CFG] = &slv_crypto_0_cfg,
	[SLAVE_CRYPTO_1_CFG] = &slv_crypto_1_cfg,
	[SLAVE_IMEM_CFG] = &slv_imem_cfg,
	[SLAVE_MESSAGE_RAM] = &slv_message_ram,
	[SLAVE_BIMC_CFG] = &slv_bimc_cfg,
	[SLAVE_BOOT_ROM] = &slv_boot_rom,
	[SLAVE_CNOC_MNOC_MMSS_CFG] = &slv_cnoc_mnoc_mmss_cfg,
	[SLAVE_PMIC_ARB] = &slv_pmic_arb,
	[SLAVE_SPDM_WRAPPER] = &slv_spdm_wrapper,
	[SLAVE_DEHR_CFG] = &slv_dehr_cfg,
	[SLAVE_MPM] = &slv_mpm,
	[SLAVE_QDSS_CFG] = &slv_qdss_cfg,
	[SLAVE_RBCPR_QDSS_APU_CFG] = &slv_rbcpr_qdss_apu_cfg,
	[SLAVE_RBCPR_CFG] = &slv_rbcpr_cfg,
	[SLAVE_CNOC_MNOC_CFG] = &slv_cnoc_mnoc_cfg,
	[SLAVE_PNOC_CFG] = &slv_pnoc_cfg,
	[SLAVE_SNOC_CFG] = &slv_snoc_cfg,
	[SLAVE_SNOC_MPU_CFG] = &slv_snoc_mpu_cfg,
	[SLAVE_EBI1_DLL_CFG] = &slv_ebi1_dll_cfg,
	[SLAVE_PHY_APU_CFG] = &slv_phy_apu_cfg,
	[SLAVE_EBI1_PHY_CFG] = &slv_ebi1_phy_cfg,
	[SLAVE_RPM] = &slv_rpm,
	[SLAVE_PCIE_0_CFG] = &slv_pcie_0_cfg,
	[SLAVE_PCIE_1_CFG] = &slv_pcie_1_cfg,
	[SLAVE_GENI_IR_CFG] = &slv_spss_geni_ir,
	[SLAVE_UFS_CFG] = &slv_ufs_cfg,
	[SLAVE_CNOC_SNOC] = &slv_cnoc_snoc,
};

static const struct regmap_config msm8994_cnoc_regmap_config = {
	.reg_bits	= 32,
	.reg_stride	= 4,
	.val_bits	= 32,
	.max_register	= 0x4000,
	.fast_io	= true
};

static const struct qcom_icc_desc msm8994_cnoc = {
	.nodes = cnoc_nodes,
	.num_nodes = ARRAY_SIZE(cnoc_nodes),
	.regmap_cfg = &msm8994_cnoc_regmap_config,
	.type = QCOM_ICC_NOC,
	.qos_offset = 0x3000,
};

static struct qcom_icc_node * const ovirt_nodes[] = {
	[MASTER_SNOC_OVNOC] = &mas_snoc_ovirt,
	[MASTER_OCMEM_DMA] = &mas_ocmem_dma,
	[MASTER_V_OCMEM_GFX3D] = &mas_oxili_ocmem,
	[MASTER_VIDEO_P0_OCMEM] = &mas_venus_ocmem,
	[SLAVE_OCMEM] = &slv_ocmem,
	[SLAVE_OCMEM_GFX] = &slv_ocmem_gfx,
	[SLAVE_OVNOC_SNOC] = &slv_ovirt_snoc,
};

static const struct qcom_icc_desc msm8994_ovirt = {
	.nodes = ovirt_nodes,
	.num_nodes = ARRAY_SIZE(ovirt_nodes),
	.type = QCOM_ICC_NOC,
};

static const struct of_device_id qnoc_of_match[] = {
	{ .compatible = "qcom,msm8994-bimc", .data = &msm8994_bimc },
	{ .compatible = "qcom,msm8994-cnoc", .data = &msm8994_cnoc },
	{ .compatible = "qcom,msm8994-mnoc", .data = &msm8994_mnoc },
	{ .compatible = "qcom,msm8994-ovirt", .data = &msm8994_ovirt },
	{ .compatible = "qcom,msm8994-pnoc", .data = &msm8994_pnoc },
	{ .compatible = "qcom,msm8994-snoc", .data = &msm8994_snoc },
	{ }
};
MODULE_DEVICE_TABLE(of, qnoc_of_match);

static struct platform_driver qnoc_driver = {
	.probe = qnoc_probe,
	.remove = qnoc_remove,
	.driver = {
		.name = "qnoc-msm8994",
		.of_match_table = qnoc_of_match,
		.sync_state = icc_sync_state,
	},
};
module_platform_driver(qnoc_driver);

MODULE_DESCRIPTION("Qualcomm MSM8994 NoC driver");
MODULE_LICENSE("GPL");
