// SPDX-License-Identifier: GPL-2.0-only
/*
 * Qualcomm MSM8998 Network-on-Chip (NoC) QoS driver
 * Copyright (c) 2020, AngeloGioacchino Del Regno
 *                     <angelogioacchino.delregno@somainline.org>
 * Copyright (C) 2020, Konrad Dybcio <konrad.dybcio@somainline.org>
 * Copyright (c) 2023, Linaro Limited
 */

#include <dt-bindings/interconnect/qcom,msm8998.h>
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/interconnect-provider.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/slab.h>

#include "icc-rpm.h"
#include "smd-rpm.h"

static const char * const a1noc_intf_clocks[] = {
	"aggre1_ufs_axi",
	"usb_axi",
	"blsp2_ahb",
};

static const char * const a2noc_intf_clocks[] = {
	"sdcc2_ahb",
	"sdcc4_ahb",
	"blsp1_ahb",
	"ipa", /* mas_ipa */
};

static const char * const mnoc_intf_clocks[] = {
	"iface",
	"mnoc_bus",
};

static const char * const cnoc_snoc_intf_clocks[] = {
	"iface",
};

enum {
	MSM8998_MASTER_IPA,
	MSM8998_MASTER_CNOC_A2NOC,
	MSM8998_MASTER_SDCC_2,
	MSM8998_MASTER_SDCC_4,
	MSM8998_MASTER_BLSP_1,
	MSM8998_MASTER_BLSP_2,
	MSM8998_MASTER_UFS,
	MSM8998_MASTER_USB_HS,
	MSM8998_MASTER_USB3,
	MSM8998_MASTER_CRYPTO_C0,
	MSM8998_MASTER_GNOC_BIMC,
	MSM8998_MASTER_OXILI,
	MSM8998_MASTER_MNOC_BIMC,
	MSM8998_MASTER_SNOC_BIMC,
	MSM8998_MASTER_PIMEM,
	MSM8998_MASTER_SNOC_CNOC,
	MSM8998_MASTER_QDSS_DAP,
	MSM8998_MASTER_APPS_PROC,
	MSM8998_MASTER_CNOC_MNOC_MMSS_CFG,
	MSM8998_MASTER_CNOC_MNOC_CFG,
	MSM8998_MASTER_CPP,
	MSM8998_MASTER_JPEG,
	MSM8998_MASTER_MDP_P0,
	MSM8998_MASTER_MDP_P1,
	MSM8998_MASTER_VENUS,
	MSM8998_MASTER_VFE,
	MSM8998_MASTER_QDSS_ETR,
	MSM8998_MASTER_QDSS_BAM,
	MSM8998_MASTER_SNOC_CFG,
	MSM8998_MASTER_BIMC_SNOC,
	MSM8998_MASTER_A1NOC_SNOC,
	MSM8998_MASTER_A2NOC_SNOC,
	MSM8998_MASTER_GNOC_SNOC,
	MSM8998_MASTER_PCIE_0,
	MSM8998_MASTER_A2NOC_TSIF,
	MSM8998_MASTER_CRVIRT_A2NOC,
	MSM8998_MASTER_ROTATOR,
	MSM8998_MASTER_VENUS_VMEM,
	MSM8998_MASTER_HMSS,
	MSM8998_MASTER_BIMC_SNOC_0,
	MSM8998_MASTER_BIMC_SNOC_1,

	MSM8998_SLAVE_A1NOC_SNOC,
	MSM8998_SLAVE_A2NOC_SNOC,
	MSM8998_SLAVE_EBI,
	MSM8998_SLAVE_HMSS_L3,
	MSM8998_SLAVE_CNOC_A2NOC,
	MSM8998_SLAVE_MPM,
	MSM8998_SLAVE_PMIC_ARB,
	MSM8998_SLAVE_TLMM_NORTH,
	MSM8998_SLAVE_TCSR,
	MSM8998_SLAVE_PIMEM_CFG,
	MSM8998_SLAVE_IMEM_CFG,
	MSM8998_SLAVE_MESSAGE_RAM,
	MSM8998_SLAVE_GLM,
	MSM8998_SLAVE_BIMC_CFG,
	MSM8998_SLAVE_PRNG,
	MSM8998_SLAVE_SPDM,
	MSM8998_SLAVE_QDSS_CFG,
	MSM8998_SLAVE_CNOC_MNOC_CFG,
	MSM8998_SLAVE_SNOC_CFG,
	MSM8998_SLAVE_QM_CFG,
	MSM8998_SLAVE_CLK_CTL,
	MSM8998_SLAVE_MSS_CFG,
	MSM8998_SLAVE_UFS_CFG,
	MSM8998_SLAVE_A2NOC_CFG,
	MSM8998_SLAVE_A2NOC_SMMU_CFG,
	MSM8998_SLAVE_GPUSS_CFG,
	MSM8998_SLAVE_AHB2PHY,
	MSM8998_SLAVE_BLSP_1,
	MSM8998_SLAVE_SDCC_2,
	MSM8998_SLAVE_SDCC_4,
	MSM8998_SLAVE_BLSP_2,
	MSM8998_SLAVE_PDM,
	MSM8998_SLAVE_CNOC_MNOC_MMSS_CFG,
	MSM8998_SLAVE_USB_HS,
	MSM8998_SLAVE_USB3_0,
	MSM8998_SLAVE_SRVC_CNOC,
	MSM8998_SLAVE_GNOC_BIMC,
	MSM8998_SLAVE_GNOC_SNOC,
	MSM8998_SLAVE_CAMERA_CFG,
	MSM8998_SLAVE_CAMERA_THROTTLE_CFG,
	MSM8998_SLAVE_MISC_CFG,
	MSM8998_SLAVE_VENUS_THROTTLE_CFG,
	MSM8998_SLAVE_VENUS_CFG,
	MSM8998_SLAVE_MMSS_CLK_XPU_CFG,
	MSM8998_SLAVE_MMSS_CLK_CFG,
	MSM8998_SLAVE_MNOC_MPU_CFG,
	MSM8998_SLAVE_DISPLAY_CFG,
	MSM8998_SLAVE_CSI_PHY_CFG,
	MSM8998_SLAVE_DISPLAY_THROTTLE_CFG,
	MSM8998_SLAVE_SMMU_CFG,
	MSM8998_SLAVE_MNOC_BIMC,
	MSM8998_SLAVE_SRVC_MNOC,
	MSM8998_SLAVE_HMSS,
	MSM8998_SLAVE_LPASS,
	MSM8998_SLAVE_WLAN,
	MSM8998_SLAVE_CDSP,
	MSM8998_SLAVE_IPA,
	MSM8998_SLAVE_SNOC_BIMC,
	MSM8998_SLAVE_SNOC_CNOC,
	MSM8998_SLAVE_IMEM,
	MSM8998_SLAVE_PIMEM,
	MSM8998_SLAVE_QDSS_STM,
	MSM8998_SLAVE_SRVC_SNOC,
	MSM8998_SLAVE_BIMC_SNOC_0,
	MSM8998_SLAVE_BIMC_SNOC_1,
	MSM8998_SLAVE_SSC_CFG,
	MSM8998_SLAVE_SKL,
	MSM8998_SLAVE_TLMM_WEST,
	MSM8998_SLAVE_A1NOC_CFG,
	MSM8998_SLAVE_A1NOC_SMMU_CFG,
	MSM8998_SLAVE_TSIF,
	MSM8998_SLAVE_TLMM_EAST,
	MSM8998_SLAVE_CRVIRT_A2NOC,
	MSM8998_SLAVE_VMEM_CFG,
	MSM8998_SLAVE_VMEM,
	MSM8998_SLAVE_PCIE_0,
};

static const u16 master_pcie_0_links[] = {
	MSM8998_SLAVE_A1NOC_SNOC,
};

static struct qcom_icc_node mas_pcie_0 = {
	.name = "master_pcie_0",
	.id = MSM8998_MASTER_PCIE_0,
	.buswidth = 16,
	.mas_rpm_id = 45,
	.slv_rpm_id = -1,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_FIXED,
	.qos.areq_prio = 1,
	.qos.prio_level = 1,
	.qos.qos_port = 1,
	.links = master_pcie_0_links,
	.num_links = ARRAY_SIZE(master_pcie_0_links),
};

static const u16 master_usb3_links[] = {
	MSM8998_SLAVE_A1NOC_SNOC,
};

static struct qcom_icc_node mas_usb3 = {
	.name = "master_usb3",
	.id = MSM8998_MASTER_USB3,
	.buswidth = 16,
	.mas_rpm_id = 32,
	.slv_rpm_id = -1,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_FIXED,
	.qos.areq_prio = 1,
	.qos.prio_level = 1,
	.qos.qos_port = 2,
	.links = master_usb3_links,
	.num_links = ARRAY_SIZE(master_usb3_links),
};

static const u16 master_ufs_links[] = {
	MSM8998_SLAVE_A1NOC_SNOC,
};

static struct qcom_icc_node mas_ufs = {
	.name = "master_ufs",
	.id = MSM8998_MASTER_UFS,
	.buswidth = 16,
	.mas_rpm_id = 68,
	.slv_rpm_id = -1,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_FIXED,
	.qos.areq_prio = 1,
	.qos.prio_level = 1,
	.qos.qos_port = 2,
	.links = master_ufs_links,
	.num_links = ARRAY_SIZE(master_ufs_links),
};

static const u16 master_blsp_2_links[] = {
	MSM8998_SLAVE_A1NOC_SNOC,
};

static struct qcom_icc_node mas_blsp_2 = {
	.name = "master_blsp_2",
	.id = MSM8998_MASTER_BLSP_2,
	.buswidth = 16,
	.mas_rpm_id = 39,
	.slv_rpm_id = -1,
	.links = master_blsp_2_links,
	.num_links = ARRAY_SIZE(master_blsp_2_links),
};

static const u16 master_cnoc_a2noc_links[] = {
	MSM8998_SLAVE_A2NOC_SNOC,
};

static struct qcom_icc_node mas_cnoc_a2noc = {
	.name = "master_cnoc_a2noc",
	.id = MSM8998_MASTER_CNOC_A2NOC,
	.buswidth = 8,
	.mas_rpm_id = 146,
	.slv_rpm_id = -1,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.links = master_cnoc_a2noc_links,
	.num_links = ARRAY_SIZE(master_cnoc_a2noc_links),
};

static const u16 master_ipa_links[] = {
	MSM8998_SLAVE_A2NOC_SNOC,
};

static struct qcom_icc_node mas_ipa = {
	.name = "master_ipa",
	.id = MSM8998_MASTER_IPA,
	.buswidth = 8,
	.mas_rpm_id = 59,
	.slv_rpm_id = -1,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_FIXED,
	.qos.areq_prio = 1,
	.qos.prio_level = 1,
	.qos.qos_port = 1,
	.links = master_ipa_links,
	.num_links = ARRAY_SIZE(master_ipa_links),
};

static const u16 master_sdcc_2_links[] = {
	MSM8998_SLAVE_A2NOC_SNOC,
};

static struct qcom_icc_node mas_sdcc_2 = {
	.name = "master_sdcc_2",
	.id = MSM8998_MASTER_SDCC_2,
	.buswidth = 8,
	.mas_rpm_id = 35,
	.slv_rpm_id = -1,
	.links = master_sdcc_2_links,
	.num_links = ARRAY_SIZE(master_sdcc_2_links),
};

static const u16 master_sdcc_4_links[] = {
	MSM8998_SLAVE_A2NOC_SNOC,
};

static struct qcom_icc_node mas_sdcc_4 = {
	.name = "master_sdcc_4",
	.id = MSM8998_MASTER_SDCC_4,
	.buswidth = 8,
	.mas_rpm_id = 36,
	.slv_rpm_id = -1,
	.links = master_sdcc_4_links,
	.num_links = ARRAY_SIZE(master_sdcc_4_links),
};

static const u16 master_a2noc_tsif_links[] = {
	MSM8998_SLAVE_A2NOC_SNOC,
};

static struct qcom_icc_node mas_tsif = {
	.name = "master_a2noc_tsif",
	.id = MSM8998_MASTER_A2NOC_TSIF,
	.buswidth = 4,
	.mas_rpm_id = 37,
	.slv_rpm_id = -1,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.links = master_a2noc_tsif_links,
	.num_links = ARRAY_SIZE(master_a2noc_tsif_links),
};

static const u16 master_blsp_1_links[] = {
	MSM8998_SLAVE_A2NOC_SNOC,
};

static struct qcom_icc_node mas_blsp_1 = {
	.name = "master_blsp_1",
	.id = MSM8998_MASTER_BLSP_1,
	.buswidth = 16,
	.mas_rpm_id = 41,
	.slv_rpm_id = -1,
	.links = master_blsp_1_links,
	.num_links = ARRAY_SIZE(master_blsp_1_links),
};

static const u16 master_crvirt_a2noc_links[] = {
	MSM8998_SLAVE_A2NOC_SNOC,
};

static struct qcom_icc_node mas_crvirt_a2noc = {
	.name = "master_crvirt_a2noc",
	.id = MSM8998_MASTER_CRVIRT_A2NOC,
	.buswidth = 8,
	.mas_rpm_id = 145,
	.slv_rpm_id = -1,
	.links = master_crvirt_a2noc_links,
	.num_links = ARRAY_SIZE(master_crvirt_a2noc_links),
};

static const u16 master_gnoc_bimc_links[] = {
	MSM8998_SLAVE_EBI,
	MSM8998_SLAVE_BIMC_SNOC_0,
};

static struct qcom_icc_node mas_gnoc_bimc = {
	.name = "master_gnoc_bimc",
	.id = MSM8998_MASTER_GNOC_BIMC,
	.channels = 2,
	.buswidth = 8,
	.mas_rpm_id = 144,
	.slv_rpm_id = -1,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_FIXED,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = 0,
	.links = master_gnoc_bimc_links,
	.num_links = ARRAY_SIZE(master_gnoc_bimc_links),
};

static const u16 master_oxili_links[] = {
	MSM8998_SLAVE_BIMC_SNOC_1,
	MSM8998_SLAVE_HMSS_L3,
	MSM8998_SLAVE_EBI,
	MSM8998_SLAVE_BIMC_SNOC_0,
};

static struct qcom_icc_node mas_oxili = {
	.name = "master_oxili",
	.id = MSM8998_MASTER_OXILI,
	.channels = 2,
	.buswidth = 8,
	.mas_rpm_id = 6,
	.slv_rpm_id = -1,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_BYPASS,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = 1,
	.links = master_oxili_links,
	.num_links = ARRAY_SIZE(master_oxili_links),
};

static const u16 master_mnoc_bimc_links[] = {
	MSM8998_SLAVE_BIMC_SNOC_1,
	MSM8998_SLAVE_HMSS_L3,
	MSM8998_SLAVE_EBI,
	MSM8998_SLAVE_BIMC_SNOC_0,
};

static struct qcom_icc_node mas_mnoc_bimc = {
	.name = "master_mnoc_bimc",
	.id = MSM8998_MASTER_MNOC_BIMC,
	.channels = 2,
	.buswidth = 8,
	.mas_rpm_id = 2,
	.slv_rpm_id = -1,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_BYPASS,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = 2,
	.links = master_mnoc_bimc_links,
	.num_links = ARRAY_SIZE(master_mnoc_bimc_links),
};

static const u16 master_snoc_bimc_links[] = {
	MSM8998_SLAVE_HMSS_L3,
	MSM8998_SLAVE_EBI,
};

static struct qcom_icc_node mas_snoc_bimc = {
	.name = "master_snoc_bimc",
	.id = MSM8998_MASTER_SNOC_BIMC,
	.channels = 2,
	.buswidth = 8,
	.mas_rpm_id = 3,
	.slv_rpm_id = -1,
	.links = master_snoc_bimc_links,
	.num_links = ARRAY_SIZE(master_snoc_bimc_links),
};

static const u16 master_snoc_cnoc_links[] = {
	MSM8998_SLAVE_SKL,
	MSM8998_SLAVE_BLSP_2,
	MSM8998_SLAVE_MESSAGE_RAM,
	MSM8998_SLAVE_TLMM_WEST,
	MSM8998_SLAVE_TSIF,
	MSM8998_SLAVE_MPM,
	MSM8998_SLAVE_BIMC_CFG,
	MSM8998_SLAVE_TLMM_EAST,
	MSM8998_SLAVE_SPDM,
	MSM8998_SLAVE_PIMEM_CFG,
	MSM8998_SLAVE_A1NOC_SMMU_CFG,
	MSM8998_SLAVE_BLSP_1,
	MSM8998_SLAVE_CLK_CTL,
	MSM8998_SLAVE_PRNG,
	MSM8998_SLAVE_USB3_0,
	MSM8998_SLAVE_QDSS_CFG,
	MSM8998_SLAVE_QM_CFG,
	MSM8998_SLAVE_A2NOC_CFG,
	MSM8998_SLAVE_PMIC_ARB,
	MSM8998_SLAVE_UFS_CFG,
	MSM8998_SLAVE_SRVC_CNOC,
	MSM8998_SLAVE_AHB2PHY,
	MSM8998_SLAVE_IPA,
	MSM8998_SLAVE_GLM,
	MSM8998_SLAVE_SNOC_CFG,
	MSM8998_SLAVE_SSC_CFG,
	MSM8998_SLAVE_SDCC_2,
	MSM8998_SLAVE_SDCC_4,
	MSM8998_SLAVE_PDM,
	MSM8998_SLAVE_CNOC_MNOC_MMSS_CFG,
	MSM8998_SLAVE_CNOC_MNOC_CFG,
	MSM8998_SLAVE_MSS_CFG,
	MSM8998_SLAVE_IMEM_CFG,
	MSM8998_SLAVE_A1NOC_CFG,
	MSM8998_SLAVE_GPUSS_CFG,
	MSM8998_SLAVE_TCSR,
	MSM8998_SLAVE_TLMM_NORTH,
};

static struct qcom_icc_node mas_snoc_cnoc = {
	.name = "master_snoc_cnoc",
	.id = MSM8998_MASTER_SNOC_CNOC,
	.buswidth = 8,
	.mas_rpm_id = 52,
	.slv_rpm_id = -1,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.links = master_snoc_cnoc_links,
	.num_links = ARRAY_SIZE(master_snoc_cnoc_links),
};

static const u16 master_qdss_dap_links[] = {
	MSM8998_SLAVE_SKL,
	MSM8998_SLAVE_BLSP_2,
	MSM8998_SLAVE_MESSAGE_RAM,
	MSM8998_SLAVE_TLMM_WEST,
	MSM8998_SLAVE_TSIF,
	MSM8998_SLAVE_MPM,
	MSM8998_SLAVE_BIMC_CFG,
	MSM8998_SLAVE_TLMM_EAST,
	MSM8998_SLAVE_SPDM,
	MSM8998_SLAVE_PIMEM_CFG,
	MSM8998_SLAVE_A1NOC_SMMU_CFG,
	MSM8998_SLAVE_BLSP_1,
	MSM8998_SLAVE_CLK_CTL,
	MSM8998_SLAVE_PRNG,
	MSM8998_SLAVE_USB3_0,
	MSM8998_SLAVE_QDSS_CFG,
	MSM8998_SLAVE_QM_CFG,
	MSM8998_SLAVE_A2NOC_CFG,
	MSM8998_SLAVE_PMIC_ARB,
	MSM8998_SLAVE_UFS_CFG,
	MSM8998_SLAVE_SRVC_CNOC,
	MSM8998_SLAVE_AHB2PHY,
	MSM8998_SLAVE_IPA,
	MSM8998_SLAVE_GLM,
	MSM8998_SLAVE_SNOC_CFG,
	MSM8998_SLAVE_SDCC_2,
	MSM8998_SLAVE_SDCC_4,
	MSM8998_SLAVE_PDM,
	MSM8998_SLAVE_CNOC_MNOC_MMSS_CFG,
	MSM8998_SLAVE_CNOC_MNOC_CFG,
	MSM8998_SLAVE_MSS_CFG,
	MSM8998_SLAVE_IMEM_CFG,
	MSM8998_SLAVE_A1NOC_CFG,
	MSM8998_SLAVE_GPUSS_CFG,
	MSM8998_SLAVE_SSC_CFG,
	MSM8998_SLAVE_TCSR,
	MSM8998_SLAVE_TLMM_NORTH,
	MSM8998_SLAVE_CNOC_A2NOC,
};

static struct qcom_icc_node mas_qdss_dap = {
	.name = "master_qdss_dap",
	.id = MSM8998_MASTER_QDSS_DAP,
	.buswidth = 8,
	.mas_rpm_id = 49,
	.slv_rpm_id = -1,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.links = master_qdss_dap_links,
	.num_links = ARRAY_SIZE(master_qdss_dap_links),
};

static const u16 master_crypto_c0_links[] = {
	MSM8998_MASTER_CRVIRT_A2NOC,
};

static struct qcom_icc_node mas_crypto = {
	.name = "master_crypto_c0",
	.id = MSM8998_MASTER_CRYPTO_C0,
	.buswidth = 650,
	.mas_rpm_id = 23,
	.slv_rpm_id = -1,
	.links = master_crypto_c0_links,
	.num_links = ARRAY_SIZE(master_crypto_c0_links),
};

static const u16 master_apps_proc_links[] = {
	MSM8998_SLAVE_GNOC_BIMC,
};

static struct qcom_icc_node mas_apss_proc = {
	.name = "master_apps_proc",
	.id = MSM8998_MASTER_APPS_PROC,
	.buswidth = 32,
	.mas_rpm_id = 0,
	.slv_rpm_id = -1,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.links = master_apps_proc_links,
	.num_links = ARRAY_SIZE(master_apps_proc_links),
};

static const u16 master_cnoc_mnoc_mmss_cfg_links[] = {
	MSM8998_SLAVE_CAMERA_THROTTLE_CFG,
	MSM8998_SLAVE_VENUS_CFG,
	MSM8998_SLAVE_MISC_CFG,
	MSM8998_SLAVE_CAMERA_CFG,
	MSM8998_SLAVE_DISPLAY_THROTTLE_CFG,
	MSM8998_SLAVE_VENUS_THROTTLE_CFG,
	MSM8998_SLAVE_DISPLAY_CFG,
	MSM8998_SLAVE_MMSS_CLK_CFG,
	MSM8998_SLAVE_VMEM_CFG,
	MSM8998_SLAVE_MMSS_CLK_XPU_CFG,
	MSM8998_SLAVE_SMMU_CFG,
};

static struct qcom_icc_node mas_cnoc_mnoc_mmss_cfg = {
	.name = "master_cnoc_mnoc_mmss_cfg",
	.id = MSM8998_MASTER_CNOC_MNOC_MMSS_CFG,
	.buswidth = 8,
	.mas_rpm_id = 4,
	.slv_rpm_id = -1,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.links = master_cnoc_mnoc_mmss_cfg_links,
	.num_links = ARRAY_SIZE(master_cnoc_mnoc_mmss_cfg_links),
};

static const u16 master_cnoc_mnoc_cfg_links[] = {
	MSM8998_SLAVE_SRVC_MNOC,
};

static struct qcom_icc_node mas_cnoc_mnoc_cfg = {
	.name = "master_cnoc_mnoc_cfg",
	.id = MSM8998_MASTER_CNOC_MNOC_CFG,
	.buswidth = 8,
	.mas_rpm_id = 5,
	.slv_rpm_id = -1,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.links = master_cnoc_mnoc_cfg_links,
	.num_links = ARRAY_SIZE(master_cnoc_mnoc_cfg_links),
};

static const u16 master_cpp_links[] = {
	MSM8998_SLAVE_MNOC_BIMC,
};

static struct qcom_icc_node mas_cpp = {
	.name = "master_cpp",
	.id = MSM8998_MASTER_CPP,
	.buswidth = 32,
	.mas_rpm_id = 115,
	.slv_rpm_id = -1,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_BYPASS,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = 5,
	.links = master_cpp_links,
	.num_links = ARRAY_SIZE(master_cpp_links),
};

static const u16 master_jpeg_links[] = {
	MSM8998_SLAVE_MNOC_BIMC,
};

static struct qcom_icc_node mas_jpeg = {
	.name = "master_jpeg",
	.id = MSM8998_MASTER_JPEG,
	.buswidth = 32,
	.mas_rpm_id = 7,
	.slv_rpm_id = -1,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_BYPASS,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = 7,
	.links = master_jpeg_links,
	.num_links = ARRAY_SIZE(master_jpeg_links),
};

static const u16 master_mdp_p0_links[] = {
	MSM8998_SLAVE_MNOC_BIMC,
};

static struct qcom_icc_node mas_mdp_p0 = {
	.name = "master_mdp_p0",
	.id = MSM8998_MASTER_MDP_P0,
	.buswidth = 32,
	.mas_rpm_id = 8,
	.slv_rpm_id = -1,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_BYPASS,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = 1,
	.links = master_mdp_p0_links,
	.num_links = ARRAY_SIZE(master_mdp_p0_links),
};

static const u16 master_mdp_p1_links[] = {
	MSM8998_SLAVE_MNOC_BIMC,
};

static struct qcom_icc_node mas_mdp_p1 = {
	.name = "master_mdp_p1",
	.id = MSM8998_MASTER_MDP_P1,
	.buswidth = 32,
	.mas_rpm_id = 61,
	.slv_rpm_id = -1,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_BYPASS,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = 2,
	.links = master_mdp_p1_links,
	.num_links = ARRAY_SIZE(master_mdp_p1_links),
};

static const u16 master_rotator_links[] = {
	MSM8998_SLAVE_MNOC_BIMC,
};

static struct qcom_icc_node mas_rotator = {
	.name = "master_rotator",
	.id = MSM8998_MASTER_ROTATOR,
	.buswidth = 32,
	.mas_rpm_id = 120,
	.slv_rpm_id = -1,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_BYPASS,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = 0,
	.links = master_rotator_links,
	.num_links = ARRAY_SIZE(master_rotator_links),
};

static const u16 master_venus_links[] = {
	MSM8998_SLAVE_MNOC_BIMC,
};

static struct qcom_icc_node mas_venus = {
	.name = "master_venus",
	.id = MSM8998_MASTER_VENUS,
	.channels = 2,
	.buswidth = 32,
	.mas_rpm_id = 9,
	.slv_rpm_id = -1,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_BYPASS,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = 3, /* [3, 4] */
	.links = master_venus_links,
	.num_links = ARRAY_SIZE(master_venus_links),
};

static const u16 master_vfe_links[] = {
	MSM8998_SLAVE_MNOC_BIMC,
};

static struct qcom_icc_node mas_vfe = {
	.name = "master_vfe",
	.id = MSM8998_MASTER_VFE,
	.buswidth = 32,
	.mas_rpm_id = 11,
	.slv_rpm_id = -1,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_BYPASS,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = 6,
	.links = master_vfe_links,
	.num_links = ARRAY_SIZE(master_vfe_links),
};

static const u16 master_venus_vmem_links[] = {
	MSM8998_SLAVE_VMEM,
};

static struct qcom_icc_node mas_venus_vmem = {
	.name = "master_venus_vmem",
	.id = MSM8998_MASTER_VENUS_VMEM,
	.buswidth = 32,
	.mas_rpm_id = 121,
	.slv_rpm_id = -1,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.links = master_venus_vmem_links,
	.num_links = ARRAY_SIZE(master_venus_vmem_links),
};

static const u16 master_hmss_links[] = {
	MSM8998_SLAVE_PIMEM,
	MSM8998_SLAVE_IMEM,
	MSM8998_SLAVE_SNOC_BIMC,
};

static struct qcom_icc_node mas_hmss = {
	.name = "master_hmss",
	.id = MSM8998_MASTER_HMSS,
	.buswidth = 16,
	.mas_rpm_id = 118,
	.slv_rpm_id = -1,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_FIXED,
	.qos.areq_prio = 1,
	.qos.prio_level = 1,
	.qos.qos_port = 3,
	.links = master_hmss_links,
	.num_links = ARRAY_SIZE(master_hmss_links),
};

static const u16 master_qdss_bam_links[] = {
	MSM8998_SLAVE_IMEM,
	MSM8998_SLAVE_PIMEM,
	MSM8998_SLAVE_SNOC_CNOC,
	MSM8998_SLAVE_SNOC_BIMC,
};

static struct qcom_icc_node mas_qdss_bam = {
	.name = "master_qdss_bam",
	.id = MSM8998_MASTER_QDSS_BAM,
	.buswidth = 16,
	.mas_rpm_id = 19,
	.slv_rpm_id = -1,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_FIXED,
	.qos.areq_prio = 1,
	.qos.prio_level = 1,
	.qos.qos_port = 1,
	.links = master_qdss_bam_links,
	.num_links = ARRAY_SIZE(master_qdss_bam_links),
};

static const u16 master_snoc_cfg_links[] = {
	MSM8998_SLAVE_SRVC_SNOC,
};

static struct qcom_icc_node mas_snoc_cfg = {
	.name = "master_snoc_cfg",
	.id = MSM8998_MASTER_SNOC_CFG,
	.buswidth = 16,
	.mas_rpm_id = 20,
	.slv_rpm_id = -1,
	.links = master_snoc_cfg_links,
	.num_links = ARRAY_SIZE(master_snoc_cfg_links),
};

static const u16 master_bimc_snoc_0_links[] = {
	MSM8998_SLAVE_PIMEM,
	MSM8998_SLAVE_LPASS,
	MSM8998_SLAVE_HMSS,
	MSM8998_SLAVE_WLAN,
	MSM8998_SLAVE_SNOC_CNOC,
	MSM8998_SLAVE_IMEM,
	MSM8998_SLAVE_QDSS_STM,
};

static struct qcom_icc_node mas_bimc_snoc_0 = {
	.name = "master_bimc_snoc_0",
	.id = MSM8998_MASTER_BIMC_SNOC_0,
	.buswidth = 16,
	.mas_rpm_id = 21,
	.slv_rpm_id = -1,
	.links = master_bimc_snoc_0_links,
	.num_links = ARRAY_SIZE(master_bimc_snoc_0_links),
};

static const u16 master_bimc_snoc_1_links[] = {
	MSM8998_SLAVE_PCIE_0,
};

static struct qcom_icc_node mas_bimc_snoc_1 = {
	.name = "master_bimc_snoc_1",
	.id = MSM8998_MASTER_BIMC_SNOC_1,
	.buswidth = 16,
	.mas_rpm_id = 109,
	.slv_rpm_id = -1,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.links = master_bimc_snoc_1_links,
	.num_links = ARRAY_SIZE(master_bimc_snoc_1_links),
};

static const u16 master_a1noc_snoc_links[] = {
	MSM8998_SLAVE_PIMEM,
	MSM8998_SLAVE_PCIE_0,
	MSM8998_SLAVE_LPASS,
	MSM8998_SLAVE_HMSS,
	MSM8998_SLAVE_SNOC_BIMC,
	MSM8998_SLAVE_SNOC_CNOC,
	MSM8998_SLAVE_IMEM,
	MSM8998_SLAVE_QDSS_STM,
};

static struct qcom_icc_node mas_a1noc_snoc = {
	.name = "master_a1noc_snoc",
	.id = MSM8998_MASTER_A1NOC_SNOC,
	.buswidth = 16,
	.mas_rpm_id = 111,
	.slv_rpm_id = -1,
	.links = master_a1noc_snoc_links,
	.num_links = ARRAY_SIZE(master_a1noc_snoc_links),
};

static const u16 master_a2noc_snoc_links[] = {
	MSM8998_SLAVE_PIMEM,
	MSM8998_SLAVE_PCIE_0,
	MSM8998_SLAVE_LPASS,
	MSM8998_SLAVE_HMSS,
	MSM8998_SLAVE_SNOC_BIMC,
	MSM8998_SLAVE_WLAN,
	MSM8998_SLAVE_SNOC_CNOC,
	MSM8998_SLAVE_IMEM,
	MSM8998_SLAVE_QDSS_STM,
};

static struct qcom_icc_node mas_a2noc_snoc = {
	.name = "master_a2noc_snoc",
	.id = MSM8998_MASTER_A2NOC_SNOC,
	.buswidth = 16,
	.mas_rpm_id = 112,
	.slv_rpm_id = -1,
	.links = master_a2noc_snoc_links,
	.num_links = ARRAY_SIZE(master_a2noc_snoc_links),
};

static const u16 master_qdss_etr_links[] = {
	MSM8998_SLAVE_IMEM,
	MSM8998_MASTER_PIMEM,
	MSM8998_SLAVE_SNOC_CNOC,
	MSM8998_SLAVE_SNOC_BIMC,
};

static struct qcom_icc_node mas_qdss_etr = {
	.name = "master_qdss_etr",
	.id = MSM8998_MASTER_QDSS_ETR,
	.buswidth = 16,
	.mas_rpm_id = 31,
	.slv_rpm_id = -1,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_FIXED,
	.qos.areq_prio = 1,
	.qos.prio_level = 1,
	.qos.qos_port = 2,
	.links = master_qdss_etr_links,
	.num_links = ARRAY_SIZE(master_qdss_etr_links),
};

static const u16 slave_a1noc_snoc_links[] = {
	MSM8998_MASTER_A1NOC_SNOC,
};

static struct qcom_icc_node slv_a1noc_snoc = {
	.name = "slave_a1noc_snoc",
	.id = MSM8998_SLAVE_A1NOC_SNOC,
	.buswidth = 16,
	.mas_rpm_id = -1,
	.slv_rpm_id = 142,
	.links = slave_a1noc_snoc_links,
	.num_links = ARRAY_SIZE(slave_a1noc_snoc_links),
};

static const u16 slave_a2noc_snoc_links[] = {
	MSM8998_MASTER_A2NOC_SNOC,
};

static struct qcom_icc_node slv_a2noc_snoc = {
	.name = "slave_a2noc_snoc",
	.id = MSM8998_SLAVE_A2NOC_SNOC,
	.buswidth = 16,
	.mas_rpm_id = -1,
	.slv_rpm_id = 143,
	.links = slave_a2noc_snoc_links,
	.num_links = ARRAY_SIZE(slave_a2noc_snoc_links),
};

static struct qcom_icc_node slv_ebi = {
	.name = "slave_ebi",
	.id = MSM8998_SLAVE_EBI,
	.channels = 2,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 0,
};

static struct qcom_icc_node slv_hmss_l3 = {
	.name = "slave_hmss_l3",
	.id = MSM8998_SLAVE_HMSS_L3,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 160,
};

static const u16 slave_bimc_snoc_0_links[] = {
	MSM8998_MASTER_BIMC_SNOC_0,
};

static struct qcom_icc_node slv_bimc_snoc_0 = {
	.name = "slave_bimc_snoc_0",
	.id = MSM8998_SLAVE_BIMC_SNOC_0,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 2,
	.links = slave_bimc_snoc_0_links,
	.num_links = ARRAY_SIZE(slave_bimc_snoc_0_links),
};

static const u16 slave_bimc_snoc_1_links[] = {
	MSM8998_MASTER_BIMC_SNOC_1,
};

static struct qcom_icc_node slv_bimc_snoc_1 = {
	.name = "slave_bimc_snoc_1",
	.id = MSM8998_SLAVE_BIMC_SNOC_1,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 138,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.links = slave_bimc_snoc_1_links,
	.num_links = ARRAY_SIZE(slave_bimc_snoc_1_links),
};

static const u16 slave_cnoc_a2noc_links[] = {
	MSM8998_MASTER_CNOC_A2NOC,
};

static struct qcom_icc_node slv_cnoc_a2noc = {
	.name = "slave_cnoc_a2noc",
	.id = MSM8998_SLAVE_CNOC_A2NOC,
	.buswidth = 4,
	.mas_rpm_id = -1,
	.slv_rpm_id = 208,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.links = slave_cnoc_a2noc_links,
	.num_links = ARRAY_SIZE(slave_cnoc_a2noc_links),
};

static struct qcom_icc_node slv_ssc_cfg = {
	.name = "slave_ssc_cfg",
	.id = MSM8998_SLAVE_SSC_CFG,
	.buswidth = 4,
	.mas_rpm_id = -1,
	.slv_rpm_id = 177,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
};

static struct qcom_icc_node slv_mpm = {
	.name = "slave_mpm",
	.id = MSM8998_SLAVE_MPM,
	.buswidth = 4,
	.mas_rpm_id = -1,
	.slv_rpm_id = 62,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
};

static struct qcom_icc_node slv_pmic_arb = {
	.name = "slave_pmic_arb",
	.id = MSM8998_SLAVE_PMIC_ARB,
	.buswidth = 4,
	.mas_rpm_id = -1,
	.slv_rpm_id = 59,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
};

static struct qcom_icc_node slv_tlmm_north = {
	.name = "slave_tlmm_north",
	.id = MSM8998_SLAVE_TLMM_NORTH,
	.buswidth = 4,
	.mas_rpm_id = -1,
	.slv_rpm_id = 214,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
};

static struct qcom_icc_node slv_pimem_cfg = {
	.name = "slave_pimem_cfg",
	.id = MSM8998_SLAVE_PIMEM_CFG,
	.buswidth = 4,
	.mas_rpm_id = -1,
	.slv_rpm_id = 167,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
};

static struct qcom_icc_node slv_imem_cfg = {
	.name = "slave_imem_cfg",
	.id = MSM8998_SLAVE_IMEM_CFG,
	.buswidth = 4,
	.mas_rpm_id = -1,
	.slv_rpm_id = 54,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
};

static struct qcom_icc_node slv_message_ram = {
	.name = "slave_message_ram",
	.id = MSM8998_SLAVE_MESSAGE_RAM,
	.buswidth = 4,
	.mas_rpm_id = -1,
	.slv_rpm_id = 55,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
};

static struct qcom_icc_node slv_skl = {
	.name = "slave_skl",
	.id = MSM8998_SLAVE_SKL,
	.buswidth = 4,
	.mas_rpm_id = -1,
	.slv_rpm_id = 196,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
};

static struct qcom_icc_node slv_bimc_cfg = {
	.name = "slave_bimc_cfg",
	.id = MSM8998_SLAVE_BIMC_CFG,
	.buswidth = 4,
	.mas_rpm_id = -1,
	.slv_rpm_id = 56,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
};

static struct qcom_icc_node slv_prng = {
	.name = "slave_prng",
	.id = MSM8998_SLAVE_PRNG,
	.buswidth = 4,
	.mas_rpm_id = -1,
	.slv_rpm_id = 44,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
};

static struct qcom_icc_node slv_a2noc_cfg = {
	.name = "slave_a2noc_cfg",
	.id = MSM8998_SLAVE_A2NOC_CFG,
	.buswidth = 4,
	.mas_rpm_id = -1,
	.slv_rpm_id = 150,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
};

static struct qcom_icc_node slv_ipa = {
	.name = "slave_ipa",
	.id = MSM8998_SLAVE_IPA,
	.buswidth = 4,
	.mas_rpm_id = -1,
	.slv_rpm_id = 183,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
};

static struct qcom_icc_node slv_tcsr = {
	.name = "slave_tcsr",
	.id = MSM8998_SLAVE_TCSR,
	.buswidth = 4,
	.mas_rpm_id = -1,
	.slv_rpm_id = 50,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
};

static struct qcom_icc_node slv_snoc_cfg = {
	.name = "slave_snoc_cfg",
	.id = MSM8998_SLAVE_SNOC_CFG,
	.buswidth = 4,
	.mas_rpm_id = -1,
	.slv_rpm_id = 70,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
};

static struct qcom_icc_node slv_clk_ctl = {
	.name = "slave_clk_ctl",
	.id = MSM8998_SLAVE_CLK_CTL,
	.buswidth = 4,
	.mas_rpm_id = -1,
	.slv_rpm_id = 47,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
};

static struct qcom_icc_node slv_glm = {
	.name = "slave_glm",
	.id = MSM8998_SLAVE_GLM,
	.buswidth = 4,
	.mas_rpm_id = -1,
	.slv_rpm_id = 209,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
};

static struct qcom_icc_node slv_spdm = {
	.name = "slave_spdm",
	.id = MSM8998_SLAVE_SPDM,
	.buswidth = 4,
	.mas_rpm_id = -1,
	.slv_rpm_id = 60,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
};

static struct qcom_icc_node slv_gpuss_cfg = {
	.name = "slave_gpuss_cfg",
	.id = MSM8998_SLAVE_GPUSS_CFG,
	.buswidth = 4,
	.mas_rpm_id = -1,
	.slv_rpm_id = 11,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
};

static const u16 slv_cnoc_mnoc_cfg_links[] = {
	MSM8998_MASTER_CNOC_MNOC_CFG,
};

static struct qcom_icc_node slv_cnoc_mnoc_cfg = {
	.name = "slv_cnoc_mnoc_cfg",
	.id = MSM8998_SLAVE_CNOC_MNOC_CFG,
	.buswidth = 4,
	.mas_rpm_id = -1,
	.slv_rpm_id = 66,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.links = slv_cnoc_mnoc_cfg_links,
	.num_links = ARRAY_SIZE(slv_cnoc_mnoc_cfg_links),
};

static struct qcom_icc_node slv_qm_cfg = {
	.name = "slave_qm_cfg",
	.id = MSM8998_SLAVE_QM_CFG,
	.buswidth = 4,
	.mas_rpm_id = -1,
	.slv_rpm_id = 212,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
};

static struct qcom_icc_node slv_mss_cfg = {
	.name = "slave_mss_cfg",
	.id = MSM8998_SLAVE_MSS_CFG,
	.buswidth = 4,
	.mas_rpm_id = -1,
	.slv_rpm_id = 48,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
};

static struct qcom_icc_node slv_ufs_cfg = {
	.name = "slave_ufs_cfg",
	.id = MSM8998_SLAVE_UFS_CFG,
	.buswidth = 4,
	.mas_rpm_id = -1,
	.slv_rpm_id = 92,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
};

static struct qcom_icc_node slv_tlmm_west = {
	.name = "slave_tlmm_west",
	.id = MSM8998_SLAVE_TLMM_WEST,
	.buswidth = 4,
	.mas_rpm_id = -1,
	.slv_rpm_id = 215,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
};

static struct qcom_icc_node slv_a1noc_cfg = {
	.name = "slave_a1noc_cfg",
	.id = MSM8998_SLAVE_A1NOC_CFG,
	.buswidth = 4,
	.mas_rpm_id = -1,
	.slv_rpm_id = 147,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
};

static struct qcom_icc_node slv_ahb2phy = {
	.name = "slave_ahb2phy",
	.id = MSM8998_SLAVE_AHB2PHY,
	.buswidth = 4,
	.mas_rpm_id = -1,
	.slv_rpm_id = 163,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
};

static struct qcom_icc_node slv_blsp_2 = {
	.name = "slave_blsp_2",
	.id = MSM8998_SLAVE_BLSP_2,
	.buswidth = 4,
	.mas_rpm_id = -1,
	.slv_rpm_id = 37,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
};

static struct qcom_icc_node slv_pdm = {
	.name = "slave_pdm",
	.id = MSM8998_SLAVE_PDM,
	.buswidth = 4,
	.mas_rpm_id = -1,
	.slv_rpm_id = 41,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
};

static struct qcom_icc_node slv_usb3_0 = {
	.name = "slave_usb3_0",
	.id = MSM8998_SLAVE_USB3_0,
	.buswidth = 4,
	.mas_rpm_id = -1,
	.slv_rpm_id = 22,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
};

static struct qcom_icc_node slv_a1noc_smmu_cfg = {
	.name = "slave_a1noc_smmu_cfg",
	.id = MSM8998_SLAVE_A1NOC_SMMU_CFG,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 149,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
};

static struct qcom_icc_node slv_blsp_1 = {
	.name = "slave_blsp_1",
	.id = MSM8998_SLAVE_BLSP_1,
	.buswidth = 4,
	.mas_rpm_id = -1,
	.slv_rpm_id = 39,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
};

static struct qcom_icc_node slv_sdcc_2 = {
	.name = "slave_sdcc_2",
	.id = MSM8998_SLAVE_SDCC_2,
	.buswidth = 4,
	.mas_rpm_id = -1,
	.slv_rpm_id = 33,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
};

static struct qcom_icc_node slv_sdcc_4 = {
	.name = "slave_sdcc_4",
	.id = MSM8998_SLAVE_SDCC_4,
	.buswidth = 4,
	.mas_rpm_id = -1,
	.slv_rpm_id = 34,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
};

static struct qcom_icc_node slv_tsif = {
	.name = "slave_tsif",
	.id = MSM8998_SLAVE_TSIF,
	.buswidth = 4,
	.mas_rpm_id = -1,
	.slv_rpm_id = 35,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
};

static struct qcom_icc_node slv_qdss_cfg = {
	.name = "slave_qdss_cfg",
	.id = MSM8998_SLAVE_QDSS_CFG,
	.buswidth = 4,
	.mas_rpm_id = -1,
	.slv_rpm_id = 63,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
};

static struct qcom_icc_node slv_tlmm_east = {
	.name = "slave_tlmm_east",
	.id = MSM8998_SLAVE_TLMM_EAST,
	.buswidth = 4,
	.mas_rpm_id = -1,
	.slv_rpm_id = 213,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
};

static const u16 slave_cnoc_mnoc_mmss_cfg_links[] = {
	MSM8998_MASTER_CNOC_MNOC_MMSS_CFG,
};

static struct qcom_icc_node slv_cnoc_mnoc_mmss_cfg = {
	.name = "slave_cnoc_mnoc_mmss_cfg",
	.id = MSM8998_SLAVE_CNOC_MNOC_MMSS_CFG,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 58,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.links = slave_cnoc_mnoc_mmss_cfg_links,
	.num_links = ARRAY_SIZE(slave_cnoc_mnoc_mmss_cfg_links),
};

static struct qcom_icc_node slv_srvc_cnoc = {
	.name = "slave_srvc_cnoc",
	.id = MSM8998_SLAVE_SRVC_CNOC,
	.buswidth = 4,
	.mas_rpm_id = -1,
	.slv_rpm_id = 76,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
};

static struct qcom_icc_node slv_crvirt_a2noc = {
	.name = "slave_crvirt_a2noc",
	.id = MSM8998_SLAVE_CRVIRT_A2NOC,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 207,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
};

static const u16 slave_gnoc_bimc_links[] = {
	MSM8998_MASTER_GNOC_BIMC,
};

static struct qcom_icc_node slv_gnoc_bimc = {
	.name = "slave_gnoc_bimc",
	.id = MSM8998_SLAVE_GNOC_BIMC,
	.buswidth = 32,
	.mas_rpm_id = -1,
	.slv_rpm_id = 210,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.links = slave_gnoc_bimc_links,
	.num_links = ARRAY_SIZE(slave_gnoc_bimc_links),
};

static struct qcom_icc_node slv_camera_cfg = {
	.name = "slave_camera_cfg",
	.id = MSM8998_SLAVE_CAMERA_CFG,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 3,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
};

static struct qcom_icc_node slv_camera_throttle_cfg = {
	.name = "slave_camera_throttle_cfg",
	.id = MSM8998_SLAVE_CAMERA_THROTTLE_CFG,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 154,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
};

static struct qcom_icc_node slv_misc_cfg = {
	.name = "slave_misc_cfg",
	.id = MSM8998_SLAVE_MISC_CFG,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 8,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
};

static struct qcom_icc_node slv_venus_throttle_cfg = {
	.name = "slave_venus_throttle_cfg",
	.id = MSM8998_SLAVE_VENUS_THROTTLE_CFG,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 178,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
};

static struct qcom_icc_node slv_venus_cfg = {
	.name = "slave_venus_cfg",
	.id = MSM8998_SLAVE_VENUS_CFG,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 10,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
};

static struct qcom_icc_node slv_vmem_cfg = {
	.name = "slave_vmem_cfg",
	.id = MSM8998_SLAVE_VMEM_CFG,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 180,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
};

static struct qcom_icc_node slv_mmss_clk_xpu_cfg = {
	.name = "slave_mmss_clk_xpu_cfg",
	.id = MSM8998_SLAVE_MMSS_CLK_XPU_CFG,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 13,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
};

static struct qcom_icc_node slv_mmss_clk_cfg = {
	.name = "slave_mmss_clk_cfg",
	.id = MSM8998_SLAVE_MMSS_CLK_CFG,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 12,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
};

static struct qcom_icc_node slv_display_cfg = {
	.name = "slave_display_cfg",
	.id = MSM8998_SLAVE_DISPLAY_CFG,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 4,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
};

static struct qcom_icc_node slv_display_throttle_cfg = {
	.name = "slave_display_throttle_cfg",
	.id = MSM8998_SLAVE_DISPLAY_THROTTLE_CFG,
	.buswidth = 4,
	.mas_rpm_id = -1,
	.slv_rpm_id = 156,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
};

static struct qcom_icc_node slv_smmu_cfg = {
	.name = "slave_smmu_cfg",
	.id = MSM8998_SLAVE_SMMU_CFG,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 205,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
};

static const u16 slave_mnoc_bimc_links[] = {
	MSM8998_MASTER_MNOC_BIMC,
};

static struct qcom_icc_node slv_mnoc_bimc = {
	.name = "slave_mnoc_bimc",
	.id = MSM8998_SLAVE_MNOC_BIMC,
	.channels = 2,
	.buswidth = 32,
	.mas_rpm_id = -1,
	.slv_rpm_id = 16,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
	.links = slave_mnoc_bimc_links,
	.num_links = ARRAY_SIZE(slave_mnoc_bimc_links),
};

static struct qcom_icc_node slv_vmem = {
	.name = "slave_vmem",
	.id = MSM8998_SLAVE_VMEM,
	.buswidth = 32,
	.mas_rpm_id = -1,
	.slv_rpm_id = 179,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
};

static struct qcom_icc_node slv_srvc_mnoc = {
	.name = "slave_srvc_mnoc",
	.id = MSM8998_SLAVE_SRVC_MNOC,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 17,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
};

static struct qcom_icc_node slv_hmss = {
	.name = "slave_hmss",
	.id = MSM8998_SLAVE_HMSS,
	.buswidth = 16,
	.mas_rpm_id = -1,
	.slv_rpm_id = 20,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
};

static struct qcom_icc_node slv_lpass = {
	.name = "slave_lpass",
	.id = MSM8998_SLAVE_LPASS,
	.buswidth = 16,
	.mas_rpm_id = -1,
	.slv_rpm_id = 21,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
};

static struct qcom_icc_node slv_wlan = {
	.name = "slave_wlan",
	.id = MSM8998_SLAVE_WLAN,
	.buswidth = 16,
	.mas_rpm_id = -1,
	.slv_rpm_id = 206,
};

static const u16 slave_snoc_bimc_links[] = {
	MSM8998_MASTER_SNOC_BIMC,
};

static struct qcom_icc_node slv_snoc_bimc = {
	.name = "slave_snoc_bimc",
	.id = MSM8998_SLAVE_SNOC_BIMC,
	.channels = 2,
	.buswidth = 32,
	.mas_rpm_id = -1,
	.slv_rpm_id = 24,
	.links = slave_snoc_bimc_links,
	.num_links = ARRAY_SIZE(slave_snoc_bimc_links),
};

static const u16 slave_snoc_cnoc_links[] = {
	MSM8998_MASTER_SNOC_CNOC,
};

static struct qcom_icc_node slv_snoc_cnoc = {
	.name = "slave_snoc_cnoc",
	.id = MSM8998_SLAVE_SNOC_CNOC,
	.buswidth = 16,
	.mas_rpm_id = -1,
	.slv_rpm_id = 25,
	.links = slave_snoc_cnoc_links,
	.num_links = ARRAY_SIZE(slave_snoc_cnoc_links),
};

static struct qcom_icc_node slv_imem = {
	.name = "slave_imem",
	.id = MSM8998_SLAVE_IMEM,
	.buswidth = 16,
	.mas_rpm_id = -1,
	.slv_rpm_id = 26,
};

static struct qcom_icc_node slv_pimem = {
	.name = "slave_pimem",
	.id = MSM8998_SLAVE_PIMEM,
	.buswidth = 16,
	.mas_rpm_id = -1,
	.slv_rpm_id = 166,
};

static struct qcom_icc_node slv_qdss_stm = {
	.name = "slave_qdss_stm",
	.id = MSM8998_SLAVE_QDSS_STM,
	.buswidth = 16,
	.mas_rpm_id = -1,
	.slv_rpm_id = 30,
};

static struct qcom_icc_node slv_pcie_0 = {
	.name = "slave_pcie_0",
	.id = MSM8998_SLAVE_PCIE_0,
	.buswidth = 16,
	.mas_rpm_id = -1,
	.slv_rpm_id = 84,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_INVALID,
};

static struct qcom_icc_node slv_srvc_snoc = {
	.name = "slave_srvc_snoc",
	.id = MSM8998_SLAVE_SRVC_SNOC,
	.buswidth = 16,
	.mas_rpm_id = -1,
	.slv_rpm_id = 29,
};

static struct qcom_icc_node *msm8998_a1noc_nodes[] = {
	[MASTER_PCIE_0] = &mas_pcie_0,
	[MASTER_USB3] = &mas_usb3,
	[MASTER_UFS] = &mas_ufs,
	[MASTER_BLSP_2] = &mas_blsp_2,
	[SLAVE_A1NOC_SNOC] = &slv_a1noc_snoc,
};

static const struct regmap_config msm8998_a1noc_regmap_config = {
	.reg_bits	= 32,
	.reg_stride	= 4,
	.val_bits	= 32,
	/* 0x60000 downstream, but that would clash with SMMUs! */
	.max_register	= 0x20000,
	.fast_io	= true,
};

static struct qcom_icc_desc msm8998_a1noc = {
	.type = QCOM_ICC_NOC,
	.nodes = msm8998_a1noc_nodes,
	.num_nodes = ARRAY_SIZE(msm8998_a1noc_nodes),
	.regmap_cfg = &msm8998_a1noc_regmap_config,
	.intf_clocks = a1noc_intf_clocks,
	.num_intf_clocks = ARRAY_SIZE(a1noc_intf_clocks),
	.qos_offset = 0x9000,
};

static struct qcom_icc_node *msm8998_a2noc_nodes[] = {
	[MASTER_IPA] = &mas_ipa,
	[MASTER_CNOC_A2NOC] = &mas_cnoc_a2noc,
	[MASTER_SDCC_2] = &mas_sdcc_2,
	[MASTER_SDCC_4] = &mas_sdcc_4,
	[MASTER_TSIF] = &mas_tsif,
	[MASTER_BLSP_1] = &mas_blsp_1,
	[MASTER_CRVIRT_A2NOC] = &mas_crvirt_a2noc,
	[MASTER_CRYPTO_C0] = &mas_crypto,
	[SLAVE_A2NOC_SNOC] = &slv_a2noc_snoc,
	[SLAVE_CRVIRT_A2NOC] = &slv_crvirt_a2noc,
};

static const struct regmap_config msm8998_a2noc_regmap_config = {
	.reg_bits	= 32,
	.reg_stride	= 4,
	.val_bits	= 32,
	/* 0x60000 downstream, but that would clash with MNoC! */
	.max_register	= 0x40000,
	.fast_io	= true,
};

static struct qcom_icc_desc msm8998_a2noc = {
	.type = QCOM_ICC_NOC,
	.nodes = msm8998_a2noc_nodes,
	.num_nodes = ARRAY_SIZE(msm8998_a2noc_nodes),
	.regmap_cfg = &msm8998_a2noc_regmap_config,
	.intf_clocks = a2noc_intf_clocks,
	.num_intf_clocks = ARRAY_SIZE(a2noc_intf_clocks),
	.qos_offset = 0x5000,
};

static struct qcom_icc_node *msm8998_bimc_nodes[] = {
	[MASTER_GNOC_BIMC] = &mas_gnoc_bimc,
	[MASTER_OXILI] = &mas_oxili,
	[MASTER_MNOC_BIMC] = &mas_mnoc_bimc,
	[MASTER_SNOC_BIMC] = &mas_snoc_bimc,
	[SLAVE_EBI] = &slv_ebi,
	[SLAVE_HMSS_L3] = &slv_hmss_l3,
	[SLAVE_BIMC_SNOC_0] = &slv_bimc_snoc_0,
	[SLAVE_BIMC_SNOC_1] = &slv_bimc_snoc_1,
};

static const struct regmap_config msm8998_bimc_regmap_config = {
	.reg_bits	= 32,
	.reg_stride	= 4,
	.val_bits	= 32,
	.max_register	= 0x80000,
	.fast_io	= true,
};

static struct qcom_icc_desc msm8998_bimc = {
	.type = QCOM_ICC_BIMC,
	.nodes = msm8998_bimc_nodes,
	.num_nodes = ARRAY_SIZE(msm8998_bimc_nodes),
	.regmap_cfg = &msm8998_bimc_regmap_config,
	/*
	 * QoS offset is -0x300 (instead of 0x800 as one would expect) in this
	 * case, as MSM8998 includes a peculiar design decision, where
	 * what-would-be BIMC_BASE is really occupied by BWMON and then the
	 * "useful" BWMON regs end right where BIMC_QoS begins.. But the driver
	 * assumes we reserve BIMC_BASE and that the QPORT 0's register is at
	 * (BIMC_BASE + QoS_OFFSET + 0x300), but that's what we unfortunately
	 * have to treat as our BIMC_BASE in DT..
	 */
	.qos_offset = -0x300,
};

static struct qcom_icc_node *msm8998_cnoc_nodes[] = {
	[MASTER_SNOC_CNOC] = &mas_snoc_cnoc,
	[MASTER_QDSS_DAP] = &mas_qdss_dap,
	[SLAVE_CNOC_A2NOC] = &slv_cnoc_a2noc,
	[SLAVE_SSC_CFG] = &slv_ssc_cfg,
	[SLAVE_MPM] = &slv_mpm,
	[SLAVE_PMIC_ARB] = &slv_pmic_arb,
	[SLAVE_TLMM_NORTH] = &slv_tlmm_north,
	[SLAVE_PIMEM_CFG] = &slv_pimem_cfg,
	[SLAVE_IMEM_CFG] = &slv_imem_cfg,
	[SLAVE_MESSAGE_RAM] = &slv_message_ram,
	[SLAVE_SKL] = &slv_skl,
	[SLAVE_BIMC_CFG] = &slv_bimc_cfg,
	[SLAVE_PRNG] = &slv_prng,
	[SLAVE_A2NOC_CFG] = &slv_a2noc_cfg,
	[SLAVE_IPA] = &slv_ipa,
	[SLAVE_TCSR] = &slv_tcsr,
	[SLAVE_SNOC_CFG] = &slv_snoc_cfg,
	[SLAVE_CLK_CTL] = &slv_clk_ctl,
	[SLAVE_GLM] = &slv_glm,
	[SLAVE_SPDM] = &slv_spdm,
	[SLAVE_GPUSS_CFG] = &slv_gpuss_cfg,
	[SLAVE_CNOC_MNOC_CFG] = &slv_cnoc_mnoc_cfg,
	[SLAVE_QM_CFG] = &slv_qm_cfg,
	[SLAVE_MSS_CFG] = &slv_mss_cfg,
	[SLAVE_UFS_CFG] = &slv_ufs_cfg,
	[SLAVE_TLMM_WEST] = &slv_tlmm_west,
	[SLAVE_A1NOC_CFG] = &slv_a1noc_cfg,
	[SLAVE_AHB2PHY] = &slv_ahb2phy,
	[SLAVE_BLSP_2] = &slv_blsp_2,
	[SLAVE_PDM] = &slv_pdm,
	[SLAVE_USB3_0] = &slv_usb3_0,
	[SLAVE_A1NOC_SMMU_CFG] = &slv_a1noc_smmu_cfg,
	[SLAVE_BLSP_1] = &slv_blsp_1,
	[SLAVE_SDCC_2] = &slv_sdcc_2,
	[SLAVE_SDCC_4] = &slv_sdcc_4,
	[SLAVE_TSIF] = &slv_tsif,
	[SLAVE_QDSS_CFG] = &slv_qdss_cfg,
	[SLAVE_TLMM_EAST] = &slv_tlmm_east,
	[SLAVE_CNOC_MNOC_MMSS_CFG] = &slv_cnoc_mnoc_mmss_cfg,
	[SLAVE_SRVC_CNOC] = &slv_srvc_cnoc,
};

static const struct regmap_config msm8998_cnoc_regmap_config = {
	.reg_bits	= 32,
	.reg_stride	= 4,
	.val_bits	= 32,
	.max_register	= 0x10000,
	.fast_io	= true,
};

static struct qcom_icc_desc msm8998_cnoc = {
	.type = QCOM_ICC_NOC,
	.nodes = msm8998_cnoc_nodes,
	.num_nodes = ARRAY_SIZE(msm8998_cnoc_nodes),
	.regmap_cfg = &msm8998_cnoc_regmap_config,
	.intf_clocks = cnoc_snoc_intf_clocks,
	.num_intf_clocks = ARRAY_SIZE(cnoc_snoc_intf_clocks),
	.keep_alive = true,
};

static struct qcom_icc_node *msm8998_gnoc_nodes[] = {
	[MASTER_APSS_PROC] = &mas_apss_proc,
	[SLAVE_GNOC_BIMC] = &slv_gnoc_bimc,
};

static const struct regmap_config msm8998_gnoc_regmap_config = {
	.reg_bits	= 32,
	.reg_stride	= 4,
	.val_bits	= 32,
	.max_register	= 0x10000,
	.fast_io	= true,
};

static struct qcom_icc_desc msm8998_gnoc = {
	.type = QCOM_ICC_NOC,
	.nodes = msm8998_gnoc_nodes,
	.num_nodes = ARRAY_SIZE(msm8998_gnoc_nodes),
	.regmap_cfg = &msm8998_gnoc_regmap_config,
	.no_clk_scaling = true,
};

static struct qcom_icc_node *msm8998_mnoc_nodes[] = {
	[MASTER_CNOC_MNOC_CFG] = &mas_cnoc_mnoc_cfg,
	[MASTER_CPP] = &mas_cpp,
	[MASTER_JPEG] = &mas_jpeg,
	[MASTER_MDP_P0] = &mas_mdp_p0,
	[MASTER_MDP_P1] = &mas_mdp_p1,
	[MASTER_ROTATOR] = &mas_rotator,
	[MASTER_VENUS] = &mas_venus,
	[MASTER_VFE] = &mas_vfe,
	[MASTER_VENUS_VMEM] = &mas_venus_vmem,
	[SLAVE_MNOC_BIMC] = &slv_mnoc_bimc,
	[SLAVE_VMEM] = &slv_vmem,
	[SLAVE_SRVC_MNOC] = &slv_srvc_mnoc,
};

static const struct regmap_config msm8998_mnoc_regmap_config = {
	.reg_bits	= 32,
	.reg_stride	= 4,
	.val_bits	= 32,
	.max_register	= 0x10000,
	.fast_io	= true,
};

static struct qcom_icc_desc msm8998_mnoc = {
	.type = QCOM_ICC_NOC,
	.nodes = msm8998_mnoc_nodes,
	.num_nodes = ARRAY_SIZE(msm8998_mnoc_nodes),
	.regmap_cfg = &msm8998_mnoc_regmap_config,
	.intf_clocks = mnoc_intf_clocks,
	.num_intf_clocks = ARRAY_SIZE(mnoc_intf_clocks),
	.qos_offset = 0x4000,
};

static struct qcom_icc_node *msm8998_mnoc_ahb_nodes[] = {
	[MASTER_CNOC_MNOC_MMSS_CFG] = &mas_cnoc_mnoc_mmss_cfg,
	[SLAVE_CAMERA_CFG] = &slv_camera_cfg,
	[SLAVE_CAMERA_THROTTLE_CFG] = &slv_camera_throttle_cfg,
	[SLAVE_MISC_CFG] = &slv_misc_cfg,
	[SLAVE_VENUS_THROTTLE_CFG] = &slv_venus_throttle_cfg,
	[SLAVE_VENUS_CFG] = &slv_venus_cfg,
	[SLAVE_VMEM_CFG] = &slv_vmem_cfg,
	[SLAVE_MMSS_CLK_XPU_CFG] = &slv_mmss_clk_xpu_cfg,
	[SLAVE_MMSS_CLK_CFG] = &slv_mmss_clk_cfg,
	[SLAVE_DISPLAY_CFG] = &slv_display_cfg,
	[SLAVE_DISPLAY_THROTTLE_CFG] = &slv_display_throttle_cfg,
	[SLAVE_SMMU_CFG] = &slv_smmu_cfg,
};

static struct qcom_icc_desc msm8998_mnoc_ahb = {
	.type = QCOM_ICC_NOC,
	.nodes = msm8998_mnoc_ahb_nodes,
	.num_nodes = ARRAY_SIZE(msm8998_mnoc_ahb_nodes),
	.regmap_cfg = &msm8998_mnoc_regmap_config,
};

static struct qcom_icc_node *msm8998_snoc_nodes[] = {
	[MASTER_HMSS] = &mas_hmss,
	[MASTER_QDSS_BAM] = &mas_qdss_bam,
	[MASTER_SNOC_CFG] = &mas_snoc_cfg,
	[MASTER_BIMC_SNOC_0] = &mas_bimc_snoc_0,
	[MASTER_BIMC_SNOC_1] = &mas_bimc_snoc_1,
	[MASTER_A1NOC_SNOC] = &mas_a1noc_snoc,
	[MASTER_A2NOC_SNOC] = &mas_a2noc_snoc,
	[MASTER_QDSS_ETR] = &mas_qdss_etr,
	[SLAVE_HMSS] = &slv_hmss,
	[SLAVE_LPASS] = &slv_lpass,
	[SLAVE_WLAN] = &slv_wlan,
	[SLAVE_SNOC_BIMC] = &slv_snoc_bimc,
	[SLAVE_SNOC_CNOC] = &slv_snoc_cnoc,
	[SLAVE_IMEM] = &slv_imem,
	[SLAVE_PIMEM] = &slv_pimem,
	[SLAVE_QDSS_STM] = &slv_qdss_stm,
	[SLAVE_PCIE_0] = &slv_pcie_0,
	[SLAVE_SRVC_SNOC] = &slv_srvc_snoc,
};

static const struct regmap_config msm8998_snoc_regmap_config = {
	.reg_bits	= 32,
	.reg_stride	= 4,
	.val_bits	= 32,
	.max_register	= 0x40000,
	.fast_io	= true,
};

static struct qcom_icc_desc msm8998_snoc = {
	.type = QCOM_ICC_NOC,
	.nodes = msm8998_snoc_nodes,
	.num_nodes = ARRAY_SIZE(msm8998_snoc_nodes),
	.regmap_cfg = &msm8998_snoc_regmap_config,
	.intf_clocks = cnoc_snoc_intf_clocks,
	.num_intf_clocks = ARRAY_SIZE(cnoc_snoc_intf_clocks),
	.qos_offset = 0x5000,
};

static const struct of_device_id qnoc_of_match[] = {
	{ .compatible = "qcom,msm8998-a1noc", .data = &msm8998_a1noc },
	{ .compatible = "qcom,msm8998-a2noc", .data = &msm8998_a2noc },
	{ .compatible = "qcom,msm8998-bimc", .data = &msm8998_bimc },
	{ .compatible = "qcom,msm8998-cnoc", .data = &msm8998_cnoc },
	{ .compatible = "qcom,msm8998-mnoc", .data = &msm8998_mnoc },
	{ .compatible = "qcom,msm8998-mnoc-ahb", .data = &msm8998_mnoc_ahb },
	{ .compatible = "qcom,msm8998-gnoc", .data = &msm8998_gnoc },
	{ .compatible = "qcom,msm8998-snoc", .data = &msm8998_snoc },
	{ }
};
MODULE_DEVICE_TABLE(of, qnoc_of_match);

static struct platform_driver qnoc_driver = {
	.probe = qnoc_probe,
	.remove = qnoc_remove,
	.driver = {
		.name = "qnoc-msm8998",
		.of_match_table = qnoc_of_match,
		.sync_state = icc_sync_state,
	},
};
static int __init qnoc_driver_init(void)
{
	return platform_driver_register(&qnoc_driver);
}
core_initcall(qnoc_driver_init);

static void __exit qnoc_driver_exit(void)
{
	platform_driver_unregister(&qnoc_driver);
}
module_exit(qnoc_driver_exit);

MODULE_DESCRIPTION("MSM8998 NoC driver");
MODULE_LICENSE("GPL");
