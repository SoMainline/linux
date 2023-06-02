// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2017-2020, The Linux Foundation. All rights reserved.
 * Copyright (c) 2023, Linaro Limited
 */

#include <linux/module.h>
#include <linux/of.h>
#include <linux/perf_event.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>

struct llcc_pmu {
	struct pmu		pmu;
	struct hlist_node	node;
	void __iomem		*base;
	struct perf_event	event;
	raw_spinlock_t		lock;
	u64			*llcc_stats;
};
#define to_llcc_pmu(p) (container_of(p, struct llcc_pmu, pmu))

#define LLCC_READ_MISS_EV 	0x1000

#define CNT_SCALING_FACTOR	0x3

#define MAX_NUM_CPUS		16

#define MON_CFG(m)		((m)->base + 0x200)
 #define MON_CFG_ENABLE(cpu)	BIT(cpu)
 #define MON_CFG_CLEARn(cpu)	BIT(16 + cpu)

#define MON_CNT(m)		((m)->base + 0x220)
 #define MON_CNT_VAL		GENMASK(23, 0)
#define MON_CNTn(m, cpu)	(MON_CNT(m) + 0x4 * cpu)

static DEFINE_PER_CPU(unsigned int, users_alive);

static void mon_enable(struct llcc_pmu *llcc_pmu, int cpu)
{
	u32 val;

	val = readl_relaxed(MON_CFG(llcc_pmu));
	val |= MON_CFG_ENABLE(cpu);
	writel_relaxed(val, MON_CFG(llcc_pmu));
}

static void mon_disable(struct llcc_pmu *llcc_pmu, int cpu)
{
	u32 val;

	val = readl_relaxed(MON_CFG(llcc_pmu));
	val &= ~MON_CFG_ENABLE(cpu);
	writel_relaxed(val, MON_CFG(llcc_pmu));
}

static void mon_clear(struct llcc_pmu *llcc_pmu, int cpu)
{
	u32 val;

	val = readl_relaxed(MON_CFG(llcc_pmu));

	val |= MON_CFG_CLEARn(cpu);
	writel_relaxed(val, MON_CFG(llcc_pmu));

	val &= ~MON_CFG_CLEARn(cpu);
	writel_relaxed(val, MON_CFG(llcc_pmu));
}

static int qcom_llcc_event_init(struct perf_event *event)
{
	struct llcc_pmu *llcc_pmu = to_llcc_pmu(event->pmu);

	if (event->attr.type != event->pmu->type)
		return -ENOENT;

	if (event->attach_state & PERF_ATTACH_TASK)
		return -EINVAL;

	if (is_sampling_event(event)) {
		dev_dbg(llcc_pmu->pmu.dev, "Per-task counters are unsupported\n");
		return -EOPNOTSUPP;
	}

	if (has_branch_stack(event)) {
		dev_dbg(llcc_pmu->pmu.dev, "Filtering is unsupported\n");
		return -EINVAL;
	}

	if (event->cpu < 0) {
		dev_warn(llcc_pmu->pmu.dev, "Can't provide per-task data!\n");
		return -EINVAL;
	}

	return 0;
}

static void qcom_llcc_event_read(struct perf_event *event)
{
	struct llcc_pmu *llcc_pmu = to_llcc_pmu(event->pmu);
	unsigned long irq_flags;
	int cpu = event->cpu;
	u64 readout;

	raw_spin_lock_irqsave(&llcc_pmu->lock, irq_flags);

	mon_disable(llcc_pmu, cpu);

	readout = FIELD_GET(MON_CNT_VAL, readl_relaxed(MON_CNTn(llcc_pmu, cpu)));
	readout <<= CNT_SCALING_FACTOR;

	llcc_pmu->llcc_stats[cpu] += readout;

	mon_clear(llcc_pmu, cpu);
	mon_enable(llcc_pmu, cpu);

	if (!(event->hw.state & PERF_HES_STOPPED))
		local64_set(&event->count, llcc_pmu->llcc_stats[cpu]);

	raw_spin_unlock_irqrestore(&llcc_pmu->lock, irq_flags);
}

static void qcom_llcc_pmu_start(struct perf_event *event, int flags)
{
	if (flags & PERF_EF_RELOAD)
		WARN_ON(!(event->hw.state & PERF_HES_UPTODATE));

	event->hw.state = 0;
}

static void qcom_llcc_event_stop(struct perf_event *event, int flags)
{
	qcom_llcc_event_read(event);
	event->hw.state |= PERF_HES_STOPPED | PERF_HES_UPTODATE;
}

static int qcom_llcc_event_add(struct perf_event *event, int flags)
{
	struct llcc_pmu *llcc_pmu = to_llcc_pmu(event->pmu);
	unsigned int cpu_users;

	raw_spin_lock(&llcc_pmu->lock);

	cpu_users = per_cpu(users_alive, event->cpu);
	if (!cpu_users)
		mon_enable(llcc_pmu, event->cpu);

	cpu_users++;
	per_cpu(users_alive, event->cpu) = cpu_users;

	raw_spin_unlock(&llcc_pmu->lock);

	event->hw.state = PERF_HES_STOPPED | PERF_HES_UPTODATE;

	if (flags & PERF_EF_START)
		qcom_llcc_pmu_start(event, PERF_EF_RELOAD);

	return 0;
}

static void qcom_llcc_event_del(struct perf_event *event, int flags)
{
	struct llcc_pmu *llcc_pmu = to_llcc_pmu(event->pmu);
	unsigned int cpu_users;

	raw_spin_lock(&llcc_pmu->lock);

	cpu_users = per_cpu(users_alive, event->cpu);
	cpu_users--;
	if (!cpu_users)
		mon_disable(llcc_pmu, event->cpu);

	per_cpu(users_alive, event->cpu) = cpu_users;

	raw_spin_unlock(&llcc_pmu->lock);
}

static ssize_t llcc_pmu_event_show(struct device *dev, struct device_attribute *attr, char *page)
{
	struct perf_pmu_events_attr *pmu_attr;

	pmu_attr = container_of(attr, struct perf_pmu_events_attr, attr);

	return sysfs_emit(page, "event=0x%04llx\n", pmu_attr->id);
}

static struct attribute *qcom_llcc_pmu_events[] = {
	PMU_EVENT_ATTR_ID(read_miss, llcc_pmu_event_show, LLCC_READ_MISS_EV),
	NULL,
};

static const struct attribute_group qcom_llcc_pmu_events_group = {
	.name = "events",
	.attrs = qcom_llcc_pmu_events,
};

PMU_FORMAT_ATTR(event, "config:0-15");
static struct attribute *qcom_llcc_pmu_format_attrs[] = {
	&format_attr_event.attr,
	NULL,
};

static const struct attribute_group qcom_llcc_pmu_format_group = {
	.name = "format",
	.attrs = qcom_llcc_pmu_format_attrs,
};

static const struct attribute_group *qcom_llcc_pmu_attr_groups[] = {
	&qcom_llcc_pmu_format_group,
	&qcom_llcc_pmu_events_group,
	NULL,
};

static int qcom_llcc_pmu_probe(struct platform_device *pdev)
{
	static struct llcc_pmu *llcc_pmu;
	int ret;

	if (num_possible_cpus() > MAX_NUM_CPUS)
		return dev_err_probe(&pdev->dev, -EINVAL,
				     "LLCC PMU only supports <=%u CPUs\n",
				     MAX_NUM_CPUS);

	llcc_pmu = devm_kzalloc(&pdev->dev, sizeof(*llcc_pmu), GFP_KERNEL);
	if (!llcc_pmu)
		return -ENOMEM;

	llcc_pmu->llcc_stats = devm_kcalloc(&pdev->dev, num_possible_cpus(),
					    sizeof(*llcc_pmu->llcc_stats), GFP_KERNEL);

	llcc_pmu->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(llcc_pmu->base))
		return dev_err_probe(&pdev->dev, PTR_ERR(llcc_pmu->base),
				     "Failed to register LLCC PMU\n");

	llcc_pmu->pmu = (struct pmu) {
		.event_init	= qcom_llcc_event_init,
		.add		= qcom_llcc_event_add,
		.del		= qcom_llcc_event_del,
		.start		= qcom_llcc_pmu_start,
		.stop		= qcom_llcc_event_stop,
		.read		= qcom_llcc_event_read,

		.attr_groups	= qcom_llcc_pmu_attr_groups,
		.capabilities	= PERF_PMU_CAP_NO_EXCLUDE,
		.task_ctx_nr	= perf_invalid_context,

		.module		= THIS_MODULE,
	};

	raw_spin_lock_init(&llcc_pmu->lock);

	ret = perf_pmu_register(&llcc_pmu->pmu, "llcc_pmu", -1);
	if (ret)
		return dev_err_probe(&pdev->dev, ret, "Failed to register LLCC PMU\n");

	return 0;
}

static const struct of_device_id qcom_llcc_pmu_match_table[] = {
	{ .compatible = "qcom,llcc-pmu-v2" },
	{ }
};

static struct platform_driver qcom_llcc_pmu_driver = {
	.probe = qcom_llcc_pmu_probe,
	.driver = {
		.name = "qcom-llcc-pmu",
		.of_match_table = qcom_llcc_pmu_match_table,
		.suppress_bind_attrs = true,
	},
};
module_platform_driver(qcom_llcc_pmu_driver);

MODULE_DEVICE_TABLE(of, qcom_llcc_pmu_match_table);
MODULE_DESCRIPTION("QCOM LLCC PMU");
MODULE_LICENSE("GPL");
