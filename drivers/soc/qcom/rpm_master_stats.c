// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2012-2020, The Linux Foundation. All rights reserved.
 * Copyright (c) 2023, Linaro Limited
 *
 * This driver supports what is known as "Master Stats v2", which seems to be
 * the only version which has ever shipped, all the way from 2013 to 2023.
 */

#include <linux/debugfs.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>

struct master_stats_data {
	void __iomem *base;
	const char *label;
};

struct rpm_master_stats {
	uint32_t active_cores;
	uint32_t num_shutdowns;
	uint64_t shutdown_req;
	uint64_t wakeup_idx;
	uint64_t bringup_req;
	uint64_t bringup_ack;
	uint32_t wakeup_reason; /* 0 = rude wakeup, 1 = scheduled wakeup */
	uint32_t last_sleep_trans_dur;
	uint32_t last_wake_trans_dur;

	/* Per-subsystem (NOT necessarily SoC-wide) XO shutdown stats */
	uint32_t xo_count;
	uint64_t xo_last_enter;
	uint64_t last_exit;
	uint64_t xo_total_dur;
};

static int rpm_master_stats_show(struct seq_file *s, void *unused)
{
	struct master_stats_data *d = s->private;
	struct rpm_master_stats stat;

	memcpy_fromio(&stat, d->base, sizeof(stat));

	seq_printf(s, "%s:\n", d->label);

	seq_printf(s, "\tLast shutdown @ %llu\n", stat.shutdown_req);
	seq_printf(s, "\tLast bringup req @ %llu\n", stat.bringup_req);
	seq_printf(s, "\tLast bringup ack @ %llu\n", stat.bringup_ack);
	seq_printf(s, "\tLast wakeup idx: %llu\n", stat.wakeup_idx);
	seq_printf(s, "\tLast XO shutdown enter @ %llu\n",
			stat.xo_last_enter);
	seq_printf(s, "\tLast XO shutdown exit @ %llu\n",
			stat.last_exit);
	seq_printf(s, "\tXO total duration: %llu\n",
			stat.xo_total_dur);
	seq_printf(s, "\tLast sleep transition duration: %u\n",
			stat.last_sleep_trans_dur);
	seq_printf(s, "\tLast wake transition duration: %u\n",
			stat.last_wake_trans_dur);
	seq_printf(s, "\tXO shutdown count: %u\n", stat.xo_count);
	seq_printf(s, "\tWakeup reason: 0x%x\n", stat.wakeup_reason);
	seq_printf(s, "\tShutdown count: %u\n", stat.num_shutdowns);
	seq_printf(s, "\tActive cores bitmask: 0x%x\n", stat.active_cores);

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(rpm_master_stats);

static int rpm_master_stats_probe(struct platform_device *pdev)
{
	struct platform_device *rpm_msg_ram_root;
	struct device *dev = &pdev->dev;
	struct master_stats_data *d;
	struct dentry *dent, *root;
	struct resource res;
	int ret;

	d = devm_kzalloc(dev, sizeof(*d), GFP_KERNEL);
	if (!d)
		return -ENOMEM;

	/* Purposefully skip devm_platform helpers as we're using a shared resource */
	ret = of_address_to_resource(dev->of_node, 0, &res);
	if (ret < 0)
		return ret;

	/* We should *only* map the rpm_master_stats struct within the RPM MSG RAM */
	if (sizeof(struct rpm_master_stats) != resource_size(&res))
		return -EINVAL;

	d->base = devm_ioremap(dev, res.start, resource_size(&res));
	if (IS_ERR(d->base))
		return -ENOMEM;

	/* Piggyback off of the common parent's unused platform data slot. */
	rpm_msg_ram_root = to_platform_device(dev->parent);
	if (!rpm_msg_ram_root)
		return -EINVAL;

	/* If it's the first instance of master stats, create a dir to contain them. */
	root = platform_get_drvdata(rpm_msg_ram_root);
	if (!root) {
		root = debugfs_create_dir("rpm_master_stats", NULL);
		platform_set_drvdata(rpm_msg_ram_root, root);
	}

	/* We can't determine the name dynamically, get it from the DT. */
	ret = of_property_read_string(dev->of_node, "label", &d->label);
	if (ret < 0) {
		if (list_empty(&root->d_child))
			debugfs_remove(root);
		return ret;
	}

	/*
	 * Generally it's not advised to fail on debugfs errors, but this
	 * driver's only job is exposing data therein.
	 */
	dent = debugfs_create_file(d->label, 0444, root,
				   d, &rpm_master_stats_fops);
	if (!dent) {
		if (list_empty(&root->d_child))
			debugfs_remove(root);
		return -EINVAL;
	}

	platform_set_drvdata(pdev, dent);

	device_set_pm_not_required(dev);

	return 0;
}

static void rpm_master_stats_remove(struct platform_device *pdev)
{
	struct dentry *dent = platform_get_drvdata(pdev);
	struct platform_device *rpm_msg_ram_root;
	struct dentry *root;

	rpm_msg_ram_root = to_platform_device(pdev->dev.parent);
	root = platform_get_drvdata(rpm_msg_ram_root);

	debugfs_remove(dent);

	/* If we removed the last entry, clean up our directory. */
	if (list_empty(&root->d_child))
		debugfs_remove(root);
}

static const struct of_device_id rpm_master_table[] = {
	{ .compatible = "qcom,rpm-master-stats" },
	{ },
};

static struct platform_driver rpm_master_stats_driver = {
	.probe = rpm_master_stats_probe,
	.remove_new = rpm_master_stats_remove,
	.driver = {
		.name = "rpm_master_stats",
		.of_match_table = rpm_master_table,
	},
};
module_platform_driver(rpm_master_stats_driver);

MODULE_DESCRIPTION("RPM Master Statistics driver");
MODULE_LICENSE("GPL");
