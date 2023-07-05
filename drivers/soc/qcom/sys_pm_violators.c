// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2020-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2021-2022, Qualcomm Innovation Center, Inc. All rights reserved.
 * Copyright (c) 2023, Linaro Limited
 */

#include <linux/debugfs.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/seq_file.h>
#include <linux/soc/qcom/qcom_aoss.h>

#define MODE_AOSS		0xaa
#define MODE_CXPC		0xcc
#define MODE_DDR		0xdd
#define MODE_STR(m)		(m == MODE_CXPC ? "CXPC" : \
				(m == MODE_AOSS ? "AOSS" : \
				(m == MODE_DDR  ? "DDR"  : "")))

struct rpmh_vx_header {
	u8 type;
	u8 log_entries;
	u16 unused;
	u16 dur_ms;
	u8 timestamp_shift;
	u8 flush_threshold;
} __packed;

struct rpmh_vx_data {
	u32 timestamp;
	u8 drv_violations[ /* ALIGN(num_drvs, 4) */ ];
} __packed;

struct rpmh_vx_log {
	struct rpmh_vx_header hdr;
	struct rpmh_vx_data *data;
	int loglines;
};

struct pm_violators_data {
	struct device *dev;
	void __iomem *base;
	struct dentry *rpmh_vx_file;
	struct qmp *qmp;
	u32 num_drvs;
	const char *drvs[];
};

static int read_rpmh_vx_data(struct pm_violators_data *pdata, struct rpmh_vx_log *log)
{
	struct rpmh_vx_header *hdr = &log->hdr;
	void __iomem *base = pdata->base;
	u32 vx_data_size;
	int i, j;

	/* Read the header from MMIO to get the log length */
	memcpy_fromio(&log->hdr, base, sizeof(*hdr));
	if (!hdr->log_entries)
		return -ENODATA;

	pr_debug("log entries=%u\n", hdr->log_entries);

	vx_data_size = struct_size(log->data, drv_violations, ALIGN(pdata->num_drvs, 4));

	log->data = kcalloc(hdr->log_entries, vx_data_size, GFP_KERNEL);
	if (!log->data)
		return -ENOMEM;

	/* Get the violation data */
	memcpy_fromio(log->data, base + offsetof(struct rpmh_vx_log, data),
		      hdr->log_entries * vx_data_size);

	for (i = 0; i < hdr->log_entries; i++) {
		/* Empty entry == EOD, jump out and stash the iterator value */
		if (!log->data[i].timestamp)
			break;

		log->data[i].timestamp <<= hdr->timestamp_shift;
	}

	log->loglines = i;

	return 0;
}

static int pm_violators_show(struct seq_file *seq, void *unused)
{
	struct pm_violators_data *data = seq->private;
	struct rpmh_vx_log log;
	struct rpmh_vx_header *hdr = &log.hdr;
	bool from_exit = false;
	int i, j, ret;

	ret = read_rpmh_vx_data(data, &log);
	if (ret)
		return ret;

	if (!log.loglines) {
		seq_printf(seq, "AOSS returned no useful data.\n");
		kfree(log.data);
		return 0;
	}

	seq_printf(seq, "Mode           : %s\n", MODE_STR(hdr->type));
	seq_printf(seq, "Duration (ms)  : %u\n", hdr->dur_ms);
	seq_printf(seq, "Time Shift     : %u\n", hdr->timestamp_shift);
	seq_printf(seq, "Flush Threshold: %u\n", hdr->flush_threshold);
	seq_printf(seq, "Max Log Entries: %u\n", hdr->log_entries);

	seq_puts(seq, "Timestamp |");

	for (i = 0; i < data->num_drvs; i++)
		seq_printf(seq, "%-*s|", 8, data->drvs[i]);
	seq_puts(seq, "\n");

	for (i = 0; i < log.loglines; i++) {
		struct rpmh_vx_data *vxdata = &log.data[i];
		u8 prev_drv_vx = 0;

		seq_printf(seq, "%-*u|", 10, vxdata->timestamp);

		/* An all-zero line indicates we entered LPM */
		for (j = 0; j < data->num_drvs; j++)
			prev_drv_vx |= log.data->drv_violations[j];

		if (!prev_drv_vx) {
			seq_printf(seq, "%s %s\n",
				   from_exit ? "Exit" : "Enter",
				   MODE_STR(hdr->type));

			from_exit = !from_exit;
			continue;
		}

		for (j = 0; j < data->num_drvs; j++)
			seq_printf(seq, "%-*u|", 8, vxdata->drv_violations[j]);
		seq_puts(seq, "\n");
	}

	kfree(log.data);
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(pm_violators);

static const struct of_device_id drv_match_table[] = {
	{ .compatible = "qcom,rpmh-pm-violators" },
	{ }
};

static int rpmh_vx_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct pm_violators_data *data;
	int num_drvs, i, ret;

	num_drvs = of_property_count_strings(dev->of_node, "qcom,drv-names");
	if (num_drvs < 0)
		return dev_err_probe(dev, num_drvs, "Couldn't count qcom,drv-names\n");

	data = devm_kzalloc(dev, struct_size(data, drvs, num_drvs), GFP_KERNEL);
	if (!data)
		return dev_err_probe(dev, ENOMEM,
				     "Couldn't allocate driver data with %u DRVs\n", num_drvs);

	for (i = 0; i < num_drvs; i++) {
		ret = of_property_read_string_index(dev->of_node, "qcom,drv-names", i,
						&data->drvs[i]);
		if (ret < 0)
			return dev_err_probe(dev, ret, "Couldn't read qcom,drv-names\n");
	}

	data->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR_OR_NULL(data->base))
		return dev_err_probe(dev, PTR_ERR(data->base), "Couldn't ioremap region\n");

	data->dev = dev;
	data->num_drvs = num_drvs;

	data->rpmh_vx_file = debugfs_create_file("qcom_pm_violators", 0400, NULL,
						 data, &pm_violators_fops);
	if (IS_ERR(data->rpmh_vx_file))
		return dev_err_probe(dev, PTR_ERR(data->rpmh_vx_file),
				     "Couldn't create debugfs entry\n");

	data->qmp = qmp_get(dev);
	if (IS_ERR(data->qmp))
		return dev_err_probe(dev, PTR_ERR(data->qmp), "Couldn't get QMP mailbox");

	/* Turn on the monitor */
	ret = qmp_send(data->qmp, "{class: lpm_mon, type: cxpc, dur: 1000, flush: 5, ts_adj: 1}");
	if (ret)
		return dev_err_probe(data->dev, ret, "failed to send QMP message\n");

	platform_set_drvdata(pdev, data);
	device_set_pm_not_required(dev);

	return 0;
}

static void rpmh_vx_remove(struct platform_device *pdev)
{
	struct pm_violators_data *data = platform_get_drvdata(pdev);
	int ret;

	/* Turn off the monitor */
	ret = qmp_send(data->qmp, "{class: lpm_mon, type: cxpc, dur: 1000, flush: 1, log_once: 1}");
	if (ret)
		dev_err(data->dev, "failed to send QMP message: %d\n", ret);

	debugfs_remove(data->rpmh_vx_file);
}

static const struct of_device_id rpmh_pm_vx_table[] = {
	{ .compatible = "qcom,rpmh-pm-violators" },
	{ }
};

static struct platform_driver rpmh_vx_driver = {
	.probe = rpmh_vx_probe,
	.remove_new = rpmh_vx_remove,
	.driver = {
		.name = "qcom-rpmh-pm-violators",
		.of_match_table = rpmh_pm_vx_table,
	},
};
module_platform_driver(rpmh_vx_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Qualcomm RPMh System PM Violators Driver");
