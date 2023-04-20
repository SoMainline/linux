/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2023 Linaro Ltd
 */

#include <linux/soc/qcom/smd-rpm.h>

#include "icc-rpm.h"

const struct rpm_clk_resource aggre1_clk = {
	.resource_type = QCOM_SMD_RPM_AGGR_CLK,
	.clock_id = 1,
};

const struct rpm_clk_resource aggre2_clk = {
	.resource_type = QCOM_SMD_RPM_AGGR_CLK,
	.clock_id = 2,
};

const struct rpm_clk_resource bimc_clk = {
	.resource_type = QCOM_SMD_RPM_MEM_CLK,
	.clock_id = 0,
};

const struct rpm_clk_resource bus_0_clk = {
	.resource_type = QCOM_SMD_RPM_BUS_CLK,
	.clock_id = 0,
};

const struct rpm_clk_resource bus_1_clk = {
	.resource_type = QCOM_SMD_RPM_BUS_CLK,
	.clock_id = 1,
};

const struct rpm_clk_resource bus_2_clk = {
	.resource_type = QCOM_SMD_RPM_BUS_CLK,
	.clock_id = 2,
};

const struct rpm_clk_resource mmaxi_0_clk = {
	.resource_type = QCOM_SMD_RPM_MMAXI_CLK,
	.clock_id = 0,
};

const struct rpm_clk_resource mmaxi_1_clk = {
	.resource_type = QCOM_SMD_RPM_MMAXI_CLK,
	.clock_id = 1,
};

const struct rpm_clk_resource qup_clk = {
	.resource_type = QCOM_SMD_RPM_QUP_CLK,
	.clock_id = 0,
};

/* Branch clocks */
const struct rpm_clk_resource aggre1_branch_clk = {
	.resource_type = QCOM_SMD_RPM_AGGR_CLK,
	.clock_id = 1,
	.branch = true,
};

const struct rpm_clk_resource aggre2_branch_clk = {
	.resource_type = QCOM_SMD_RPM_AGGR_CLK,
	.clock_id = 2,
	.branch = true,
};
