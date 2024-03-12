// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2010-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022 Qualcomm Innovation Center, Inc. All rights reserved.
 * Copyright (c) 2024, Linaro Limited
 */
#define pr_fmt(fmt) "gov_adreno_tz: " fmt

#include <linux/devfreq.h>
#include <linux/devfreq/governor_adreno_tz.h>
#include <linux/dma-mapping.h>
#include <linux/errno.h>
#include <linux/firmware/qcom/qcom_scm.h>
#include <linux/of_platform.h>
#include <linux/spinlock.h>

#include "governor.h"

static DEFINE_MUTEX(tz_lock);
static DEFINE_SPINLOCK(sample_lock);
static DEFINE_SPINLOCK(suspend_lock);

#define MAX_TZ_VERSION			0
#define MSM_ADRENO_MAX_PWRLEVELS	16
#define DEFAULT_CTX_AWARE_BUSY_PENALTY	12000 /* ns */

/* 5ms to capture up to 3 re-draws per frame for 60fps content. */
#define TOTAL_TIME_FLOOR		5000 /* ns */

/* BUSY_TIME_FLOOR is 1 ms for the sample to be sent */
#define BUSY_TIME_FLOOR			1000 /* ns */

/* 50ms, larger than any standard frame length, but less than the idle timer. */
#define BUSY_TIME_CEILING		50000 /* ns */

static u64 suspend_start;
static u64 suspend_time;
static unsigned long acc_relative_busy;
static unsigned long acc_total;

/* Returns GPU suspend time in millisecond. */
static u64 suspend_time_ms(void)
{
	u64 suspend_sampling_time;
	u64 time_diff;

	if (!suspend_start)
		return 0;

	suspend_sampling_time = ktime_to_ms(ktime_get());
	time_diff = suspend_sampling_time - suspend_start;

	/* Update the suspend_start sample again */
	suspend_start = suspend_sampling_time;

	return time_diff;
}

static ssize_t gpu_load_show(struct device *dev,
			     struct device_attribute *attr,
			     char *buf)
{
	unsigned long sysfs_busy_perc = 0;

	/*
	 * Average out the samples taken since last read. This will keep the
	 * average value in sync with with the client sampling duration.
	 */
	guard(spinlock)(&sample_lock);
	if (acc_total)
		sysfs_busy_perc = (acc_relative_busy * 100) / acc_total;

	/* Reset the parameters */
	acc_total = 0;
	acc_relative_busy = 0;
	return snprintf(buf, PAGE_SIZE, "%lu\n", sysfs_busy_perc);
}

/*
 * Returns the time in ms for which gpu was in suspend state
 * since last time the entry is read.
 */
static ssize_t suspend_time_show(struct device *dev,
				 struct device_attribute *attr,
				 char *buf)
{
	u64 time_diff;

	guard(spinlock)(&suspend_lock);
	time_diff = suspend_time_ms();
	/*
	 * Adding the previous suspend time also as the gpu
	 * can go and come out of suspend states in between
	 * reads also and we should have the total suspend
	 * since last read.
	 */
	time_diff += suspend_time;
	suspend_time = 0;

	return snprintf(buf, PAGE_SIZE, "%llu\n", time_diff);
}
static DEVICE_ATTR_RO(gpu_load);
static DEVICE_ATTR_RO(suspend_time);

static const struct device_attribute *adreno_tz_attr_list[] = {
	&dev_attr_gpu_load,
	&dev_attr_suspend_time,
	NULL
};

/**
 * adreno_tz_compute_work_load() - Compute the weighted size of the work
 * @stats: Pointer to devfreq statistics
 * @priv: Pointer to the governor private data
 * @devfreq: Pointer to the devfreq instance
 */
static void adreno_tz_compute_work_load(struct devfreq_dev_status *stats,
					struct devfreq_msm_adreno_tz_data *priv,
					struct devfreq *devfreq)
{
	u64 busy;

	guard(spinlock)(&sample_lock);
	/*
	 * Keep collecting the stats until the client reads them.
	 * The data is cleared upon reading the stats in sysfs.
	 */
	acc_total += stats->total_time;
	busy = (u64)stats->busy_time * stats->current_frequency;
	do_div(busy, devfreq->profile->freq_table[devfreq->profile->max_state - 1]);
	acc_relative_busy += busy;
}

static int scm_reset_entry(unsigned int *scm_data,
			   u32 size_scm_data, bool is_64)
{
	/* Make sure TZ reads the data we might have *just* stored */
	wmb();

	if (is_64)
		return qcom_scm_dcvs_reset();

	guard(mutex)(&tz_lock);
	return qcom_scm_io_reset();
}

static int scm_update_entry(int level, s64 total_time, s64 busy_time,
			    int context_count,
			    struct devfreq_msm_adreno_tz_data *priv)
{
	/* Make sure TZ reads the data we might have *just* stored */
	wmb();

	if (priv->ctxt_aware_enable)
		return qcom_scm_dcvs_update_ca_v2(level, total_time, busy_time, context_count);
	else if (priv->is_64)
		return qcom_scm_dcvs_update_v2(level, total_time, busy_time);

	guard(mutex)(&tz_lock);
	return qcom_scm_dcvs_update(level, total_time, busy_time);
}

/**
 * adreno_tz_init_ca() - Configure the context-aware algos within the adreno_tz
 * 			 devfreq governor
 * @dev: Pointer to the device that owns the devfreq instance
 * @priv: Pointer to the governor private data
 */
static int adreno_tz_init_ca(struct device *dev,
			     struct devfreq_msm_adreno_tz_data *priv)
{
	u8 *tz_buf __free(kfree_sensitive);
	unsigned int tz_ca_data[2];
	phys_addr_t paddr;

	/* Set data for TZ */
	tz_ca_data[0] = priv->ctxt_aware_target_pwrlevel;
	tz_ca_data[1] = DEFAULT_CTX_AWARE_BUSY_PENALTY; /* This is *technically* configurable */

	tz_buf = kzalloc(PAGE_ALIGN(sizeof(tz_ca_data)), GFP_KERNEL);
	if (!tz_buf)
		return -ENOMEM;
	memcpy(tz_buf, tz_ca_data, sizeof(tz_ca_data));

	paddr = virt_to_phys(tz_buf);
	dma_sync_single_for_device(dev, paddr, PAGE_ALIGN(sizeof(tz_ca_data)),
				   DMA_BIDIRECTIONAL);

	return qcom_scm_dcvs_init_ca_v2(paddr, sizeof(tz_ca_data));
}

static int adreno_tz_init(struct device *dev, struct devfreq_msm_adreno_tz_data *priv,
			  unsigned int *tz_pwrlevels, u32 size_pwrlevels,
			  unsigned int *version, u32 size_version)
{
	u8 *tz_buf __free(kfree_sensitive);
	phys_addr_t paddr;
	int ret;

	if (!qcom_scm_dcvs_core_available()) {
		pr_err("Adreno TZ governor is not exposed by TZ\n");
		return -EINVAL;
	}

	tz_buf = kzalloc(PAGE_ALIGN(size_pwrlevels), GFP_KERNEL);
	if (!tz_buf)
		return -ENOMEM;
	memcpy(tz_buf, tz_pwrlevels, size_pwrlevels);

	paddr = virt_to_phys(tz_buf);
	dma_sync_single_for_device(dev, paddr, PAGE_ALIGN(size_pwrlevels),
				   DMA_BIDIRECTIONAL);

	ret = qcom_scm_dcvs_init_v2(paddr, size_pwrlevels, version);
	if (!ret)
		priv->is_64 = true;

	/*
	 * If context-aware DCVS is not available, or the initialization fails,
	 * simply print an error message and return success, as "normal" DCVS
	 * will still work just fine.
	 */
	priv->ctxt_aware_enable = priv->is_64 && qcom_scm_dcvs_ca_available();
	if (!priv->ctxt_aware_enable) {
		pr_warn("Context-aware DCVS isn't supported\n");
		return 0;
	}

	ret = adreno_tz_init_ca(dev, priv);
	if (ret) {
		pr_err("Context-aware DCVS init failed\n");
		priv->ctxt_aware_enable = false;
		return 0;
	}

	return ret;
}

static int adreno_tz_get_target_freq(struct devfreq *devfreq,
				     unsigned long *freq)
{
	struct devfreq_dev_status *status = &devfreq->last_status;
	struct devfreq_msm_adreno_tz_data *priv = devfreq->data;
	int level, level_offset, ret;
	int context_count = 0;

	ret = devfreq_update_stats(devfreq);
	if (ret) {
		pr_err("devfreq_update_stats returned: %d\n", ret);
		return ret;
	}

	*freq = status->current_frequency;
	priv->total_time += status->total_time;

	/* busy_time should not go over total_time */
	status->busy_time = min_t(u64, status->busy_time, status->total_time);

	priv->busy_time += status->busy_time;

	if (status->private_data)
		context_count = *(int *)status->private_data;

	/* Update the GPU load statistics */
	adreno_tz_compute_work_load(status, priv, devfreq);

	/*
	 * Do not waste CPU cycles running this algorithm if the GPU has just
	 * started, OR if less than TOTAL_TIME_FLOOR time has passed since the
	 * last run, OR the gpu hasn't been busy for at least BUSY_TIME_FLOOR.
	 */
	if ((status->total_time == 0) ||
	    (priv->total_time < TOTAL_TIME_FLOOR) ||
	    (priv->busy_time < BUSY_TIME_FLOOR)) {
		return 0;
	}

	level = devfreq_get_freq_level(devfreq, status->current_frequency);
	if (level < 0) {
		pr_err("Couldn't get level for frequency %ld\n", status->current_frequency);
		return level;
	}

	/*
	 * If there is an extended block of busy processing, bump up the
	 * frequency. Otherwise, run the normal algorithm.
	 */
	if (priv->busy_time > BUSY_TIME_CEILING) {
		level_offset = 0;
		level = 0; /* Jump to F_MAX */
	} else {
		level_offset = scm_update_entry(level, priv->total_time,
						priv->busy_time,
						context_count, priv);
	}

	priv->total_time = 0;
	priv->busy_time = 0;

	/*
	 * If the decision is to move to a different level, make sure the GPU
	 * frequency changes.
	 */
	if (level_offset) {
		level += level_offset;
		level = max(level, 0);
		level = min_t(int, level, devfreq->profile->max_state - 1);
	}

	*freq = devfreq->profile->freq_table[level];

	pr_debug("TZ suggests %lu Hz (level = %u)\n", *freq, level);

	return ret;
}

/**
 * adreno_tz_start() - Start the adreno_tz governor
 * @devfreq: Pointer to the devfreq instance
 *
 * Returns: 0 on success, negative errno on failure
 */
static int adreno_tz_start(struct devfreq *devfreq)
{
	unsigned int tz_pwrlevels[MSM_ADRENO_MAX_PWRLEVELS + 1] = { 0 };
	struct devfreq_msm_adreno_tz_data *priv = devfreq->data;
	struct devfreq_dev_profile *profile = devfreq->profile;
	unsigned int version;
	int i, ret;

	if (profile->max_state >= ARRAY_SIZE(tz_pwrlevels)) {
		pr_err("Power level array is too long (%d > %d)\n",
		       profile->max_state, MSM_ADRENO_MAX_PWRLEVELS);
		return -EINVAL;
	}

	/* devfreq internally uses low-to-high OPP ordering, adreno_tz doesn't.. */
	for (i = 0; i < profile->max_state; i++)
		tz_pwrlevels[i + 1] = profile->freq_table[profile->max_state - 1 - i];

	/* The first element holds the number of entries */
	tz_pwrlevels[0] = profile->max_state - 1;

	ret = adreno_tz_init(&devfreq->dev, priv,
			     tz_pwrlevels, sizeof(tz_pwrlevels),
			     &version, sizeof(version));
	if (ret) {
		pr_err("adreno_tz_init failed with: %d\n", ret);
		return ret;
	} else if (version > MAX_TZ_VERSION) {
		pr_err("TZ governor v%d is not supported!\n", version);
		return -EINVAL;
	}

	for (i = 0; adreno_tz_attr_list[i]; i++) {
		ret = device_create_file(&devfreq->dev, adreno_tz_attr_list[i]);
		if (ret) {
			pr_err("Failed to create adreno_tz file %d\n", i);
			while (i--) {
				device_remove_file(&devfreq->dev,
						   adreno_tz_attr_list[i]);
			}

			return ret;
		}
	}

	return 0;
}

/**
 * adreno_tz_stop() - Stop the adreno_tz governor instance
 * @devfreq: Pointer to the devfreq instance
 */
static void adreno_tz_stop(struct devfreq *devfreq)
{
	int i;

	for (i = 0; adreno_tz_attr_list[i]; i++)
		device_remove_file(&devfreq->dev, adreno_tz_attr_list[i]);
}

/**
 * adreno_tz_suspend() - Suspend the adreno_tz devfreq governor
 * @devfreq: Pointer to the devfreq instance
 *
 * Return: 0 on success, negative errno on failure
 */
static int adreno_tz_suspend(struct devfreq *devfreq)
{
	struct devfreq_msm_adreno_tz_data *priv = devfreq->data;
	unsigned int scm_data[2] = { 0, 0 };
	int ret;

	ret = scm_reset_entry(scm_data, sizeof(scm_data), priv->is_64);
	if (ret) {
		pr_err("Couldn't reset adreno_tz data: %d\n", ret);
		return ret;
	}

	priv->total_time = 0;
	priv->busy_time = 0;

	return 0;
}

static int adreno_tz_handler(struct devfreq *devfreq,
			     unsigned int event, void *data)
{
	int ret;

	switch (event) {
	case DEVFREQ_GOV_START:
		ret = adreno_tz_start(devfreq);
		break;
	case DEVFREQ_GOV_STOP:
		spin_lock(&suspend_lock);
		suspend_start = 0;
		spin_unlock(&suspend_lock);
		adreno_tz_stop(devfreq);
		break;
	case DEVFREQ_GOV_SUSPEND:
		ret = adreno_tz_suspend(devfreq);
		if (!ret) {
			spin_lock(&suspend_lock);
			/* Collect the start sample for suspend time */
			suspend_start = ktime_to_ms(ktime_get());
			spin_unlock(&suspend_lock);
		}
		break;
	case DEVFREQ_GOV_RESUME:
		spin_lock(&suspend_lock);
		suspend_time += suspend_time_ms();
		/* Reset suspend_start when gpu resumes */
		suspend_start = 0;
		spin_unlock(&suspend_lock);
		ret = 0;
		break;
	default:
		ret = 0;
		break;
	}

	return ret;
}

static struct devfreq_governor msm_adreno_tz = {
	.name = DEVFREQ_GOV_ADRENO_TZ,
	.get_target_freq = adreno_tz_get_target_freq,
	.event_handler = adreno_tz_handler,
	.flags = DEVFREQ_GOV_FLAG_IMMUTABLE,
};

/**
 * msm_adreno_tz_init() - Initialize the adreno_tz devfreq governor
 * @dev: Pointer to the device that owns the devfreq instance
 *
 * Return: 0 on success, negative errno on failure
 */
int governor_adreno_tz_init(struct device *dev)
{
	return devm_devfreq_add_governor(dev, &msm_adreno_tz);
}
EXPORT_SYMBOL_GPL(governor_adreno_tz_init);
