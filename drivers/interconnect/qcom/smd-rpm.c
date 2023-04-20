// SPDX-License-Identifier: GPL-2.0
/*
 * RPM over SMD communication wrapper for interconnects
 *
 * Copyright (C) 2019 Linaro Ltd
 * Author: Georgi Djakov <georgi.djakov@linaro.org>
 */

#include <linux/interconnect-provider.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/soc/qcom/smd-rpm.h>

#include "icc-rpm.h"

#define RPM_KEY_BW		0x00007762
#define QCOM_RPM_SMD_KEY_RATE	0x007a484b

static struct qcom_smd_rpm *icc_smd_rpm;

struct icc_rpm_smd_req {
	__le32 key;
	__le32 nbytes;
	__le32 value;
};

bool qcom_icc_rpm_smd_available(void)
{
	return !!icc_smd_rpm;
}
EXPORT_SYMBOL_GPL(qcom_icc_rpm_smd_available);

int qcom_icc_rpm_smd_send(int ctx, int rsc_type, int id, u32 val)
{
	struct icc_rpm_smd_req req = {
		.key = cpu_to_le32(RPM_KEY_BW),
		.nbytes = cpu_to_le32(sizeof(u32)),
		.value = cpu_to_le32(val),
	};

	return qcom_rpm_smd_write(icc_smd_rpm, ctx, rsc_type, id, &req,
				  sizeof(req));
}
EXPORT_SYMBOL_GPL(qcom_icc_rpm_smd_send);

int qcom_icc_rpm_set_bus_rate(struct rpm_clk_resource *clk, u32 active_rate, u32 sleep_rate)
{
	struct clk_smd_rpm_req req = {
		.key = cpu_to_le32(QCOM_RPM_SMD_KEY_RATE),
		.nbytes = cpu_to_le32(sizeof(u32)),
	};
	int ret;

	pr_debug("act=%ukHz slp=%ukHz", active_rate, sleep_rate);

	req.value = cpu_to_le32(active_rate);
	ret = qcom_rpm_smd_write(icc_smd_rpm,
				 QCOM_SMD_RPM_ACTIVE_STATE,
				 clk->resource_type,
				 clk->clock_id,
				 &req, sizeof(req));
	if (ret)
		return ret;

	req.value = cpu_to_le32(sleep_rate);
	return qcom_rpm_smd_write(icc_smd_rpm,
				  QCOM_SMD_RPM_SLEEP_STATE,
				  clk->resource_type,
				  clk->clock_id,
				  &req, sizeof(req));
}
EXPORT_SYMBOL_GPL(qcom_icc_rpm_set_bus_rate);

static int qcom_icc_rpm_smd_remove(struct platform_device *pdev)
{
	icc_smd_rpm = NULL;

	return 0;
}

static int qcom_icc_rpm_smd_probe(struct platform_device *pdev)
{
	icc_smd_rpm = dev_get_drvdata(pdev->dev.parent);

	if (!icc_smd_rpm) {
		dev_err(&pdev->dev, "unable to retrieve handle to RPM\n");
		return -ENODEV;
	}

	return 0;
}

static struct platform_driver qcom_interconnect_rpm_smd_driver = {
	.driver = {
		.name		= "icc_smd_rpm",
	},
	.probe = qcom_icc_rpm_smd_probe,
	.remove = qcom_icc_rpm_smd_remove,
};
module_platform_driver(qcom_interconnect_rpm_smd_driver);
MODULE_AUTHOR("Georgi Djakov <georgi.djakov@linaro.org>");
MODULE_DESCRIPTION("Qualcomm SMD RPM interconnect proxy driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:icc_smd_rpm");
