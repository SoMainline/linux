// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2013-2015, The Linux Foundation. All rights reserved.
 * Copyright (c) 2019, Linaro Limited
 */

#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/nvmem-consumer.h>
#include <linux/of_device.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_domain.h>
#include <linux/pm_opp.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/string.h>

#include "cpr-common.h"

int cpr_populate_ring_osc_idx(struct device *dev,
			      struct fuse_corner *fuse_corner,
			      const struct cpr_fuse *cpr_fuse,
			      int num_fuse_corners)
{
	struct fuse_corner *end = fuse_corner + num_fuse_corners;
	u32 data;
	int ret;

	for (; fuse_corner < end; fuse_corner++, cpr_fuse++) {
		ret = nvmem_cell_read_variable_le_u32(dev, cpr_fuse->ring_osc, &data);
		if (ret)
			return ret;
		fuse_corner->ring_osc_idx = data;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(cpr_populate_ring_osc_idx);

static int cpr_read_fuse_uV(int init_v_width, int step_size_uV, int ref_uV,
			    int adj, int step_volt, const char *init_v_efuse,
			    struct device *dev)
{
	int steps, uV;
	u32 bits = 0;
	int ret;

	ret = nvmem_cell_read_variable_le_u32(dev, init_v_efuse, &bits);
	if (ret)
		return ret;

	steps = bits & ~BIT(init_v_width - 1);
	/* Not two's complement.. instead highest bit is sign bit */
	if (bits & BIT(init_v_width - 1))
		steps = -steps;

	uV = ref_uV + steps * step_size_uV;

	/* Apply open-loop fixed adjustments to fused values */
	uV += adj;

	return DIV_ROUND_UP(uV, step_volt) * step_volt;
}

const struct cpr_fuse *cpr_get_fuses(struct device *dev, int tid,
				     unsigned int num_fuse_corners)
{
	struct cpr_fuse *fuses;
	char cpr_name[11]; /* length of "cpr" + length of UINT_MAX (7) + \0 */
	unsigned int i;

	fuses = devm_kcalloc(dev, num_fuse_corners, sizeof(*fuses), GFP_KERNEL);
	if (!fuses)
		return ERR_PTR(-ENOMEM);

	/* Support legacy bindings */
	if (tid == UINT_MAX)
		snprintf(cpr_name, sizeof(cpr_name), "cpr");
	else
		snprintf(cpr_name, sizeof(cpr_name), "cpr%d", tid);

	for (i = 0; i < num_fuse_corners; i++) {
		char tbuf[50];

		snprintf(tbuf, sizeof(tbuf), "%s_ring_osc%d", cpr_name, i + 1);
		fuses[i].ring_osc = devm_kstrdup(dev, tbuf, GFP_KERNEL);
		if (!fuses[i].ring_osc)
			return ERR_PTR(-ENOMEM);

		snprintf(tbuf, sizeof(tbuf), "%s_init_voltage%d", cpr_name, i + 1);
		fuses[i].init_voltage = devm_kstrdup(dev, tbuf, GFP_KERNEL);
		if (!fuses[i].init_voltage)
			return ERR_PTR(-ENOMEM);

		snprintf(tbuf, sizeof(tbuf), "%s_quotient%d", cpr_name, i + 1);
		fuses[i].quotient = devm_kstrdup(dev, tbuf, GFP_KERNEL);
		if (!fuses[i].quotient)
			return ERR_PTR(-ENOMEM);

		snprintf(tbuf, sizeof(tbuf), "%s_quotient_offset%d", cpr_name, i + 1);
		fuses[i].quotient_offset = devm_kstrdup(dev, tbuf, GFP_KERNEL);
		if (!fuses[i].quotient_offset)
			return ERR_PTR(-ENOMEM);
	}

	return fuses;
}
EXPORT_SYMBOL_GPL(cpr_get_fuses);

int cpr_populate_fuse_common(struct device *dev,
			     struct fuse_corner_data *fdata,
			     const struct cpr_fuse *cpr_fuse,
			     struct fuse_corner *fuse_corner,
			     int step_volt, int init_v_width,
			     int init_v_step)
{
	int uV, ret;

	/* Populate uV */
	uV = cpr_read_fuse_uV(init_v_width, init_v_step,
			      fdata->ref_uV, fdata->volt_oloop_adjust,
			      step_volt, cpr_fuse->init_voltage, dev);
	if (uV < 0)
		return uV;

	/*
	 * Update SoC voltages: platforms might choose a different
	 * regulators than the one used to characterize the algorithms
	 * (ie, init_voltage_step).
	 */
	fdata->min_uV = roundup(fdata->min_uV, step_volt);
	fdata->max_uV = roundup(fdata->max_uV, step_volt);

	fuse_corner->min_uV = fdata->min_uV;
	fuse_corner->max_uV = fdata->max_uV;
	fuse_corner->uV = clamp(uV, fuse_corner->min_uV, fuse_corner->max_uV);

	/* Populate target quotient by scaling */
	ret = nvmem_cell_read_variable_le_u32(dev, cpr_fuse->quotient, &fuse_corner->quot);
	if (ret)
		return ret;

	fuse_corner->quot *= fdata->quot_scale;
	fuse_corner->quot += fdata->quot_offset;
	fuse_corner->quot += fdata->quot_adjust;

	return 0;
}
EXPORT_SYMBOL_GPL(cpr_populate_fuse_common);

int cpr_find_initial_corner(struct device *dev, struct clk *cpu_clk,
			    struct corner *corners, int num_corners)
{
	unsigned long rate;
	struct corner *iter, *corner;
	const struct corner *end;
	unsigned int i = 0;

	if (!cpu_clk) {
		dev_err(dev, "cannot get rate from NULL clk\n");
		return -EINVAL;
	}

	end = &corners[num_corners - 1];
	rate = clk_get_rate(cpu_clk);

	/*
	 * Some bootloaders set a CPU clock frequency that is not defined
	 * in the OPP table. When running at an unlisted frequency,
	 * cpufreq_online() will change to the OPP which has the lowest
	 * frequency, at or above the unlisted frequency.
	 * Since cpufreq_online() always "rounds up" in the case of an
	 * unlisted frequency, this function always "rounds down" in case
	 * of an unlisted frequency. That way, when cpufreq_online()
	 * triggers the first ever call to cpr_set_performance_state(),
	 * it will correctly determine the direction as UP.
	 */
	for (iter = corners; iter <= end; iter++) {
		if (iter->freq > rate)
			break;
		i++;
		if (iter->freq == rate) {
			corner = iter;
			break;
		}
		if (iter->freq < rate)
			corner = iter;
	}

	if (!corner) {
		dev_err(dev, "boot up corner not found\n");
		return -EINVAL;
	}

	dev_dbg(dev, "boot up perf state: %u\n", i);

	return 0;
}
EXPORT_SYMBOL_GPL(cpr_find_initial_corner);

unsigned int cpr_get_fuse_corner(struct dev_pm_opp *opp, u32 tid)
{
	struct device_node *np;
	unsigned int fuse_corner = 0;

	np = dev_pm_opp_get_of_node(opp);
	if (of_property_read_u32_index(np, "qcom,opp-fuse-level", tid, &fuse_corner))
		pr_err("%s: missing 'qcom,opp-fuse-level[%u]' property\n",
		       __func__, tid);

	of_node_put(np);

	return fuse_corner;
}
EXPORT_SYMBOL_GPL(cpr_get_fuse_corner);

u64 cpr_get_opp_hz_for_req(struct dev_pm_opp *ref,
				     struct device *cpu_dev)
{
	u64 rate = 0;
	struct device_node *ref_np;
	struct device_node *desc_np;
	struct device_node *child_np = NULL;
	struct device_node *child_req_np = NULL;

	desc_np = dev_pm_opp_of_get_opp_desc_node(cpu_dev);
	if (!desc_np)
		return 0;

	ref_np = dev_pm_opp_get_of_node(ref);
	if (!ref_np)
		goto out_ref;

	for_each_available_child_of_node(desc_np, child_np) {
		child_req_np = of_parse_phandle(child_np, "required-opps", 0);

		if (child_np && child_req_np == ref_np) {
			of_property_read_u64(child_np, "opp-hz", &rate);
			goto out;
		}
	}

out:
	of_node_put(child_req_np);
	of_node_put(child_np);
	of_node_put(ref_np);
out_ref:
	of_node_put(desc_np);

	return rate;
}
EXPORT_SYMBOL_GPL(cpr_get_opp_hz_for_req);

int cpr_calculate_scaling(struct device *dev,
			  const char *quot_offset,
			  const struct fuse_corner_data *fdata,
			  const struct corner *corner)
{
	u32 quot_diff = 0;
	u64 freq_diff;
	int scaling;
	const struct fuse_corner *fuse, *prev_fuse;
	int ret;

	fuse = corner->fuse_corner;
	prev_fuse = fuse - 1;

	if (quot_offset) {
		ret = nvmem_cell_read_variable_le_u32(dev, quot_offset, &quot_diff);
		if (ret)
			return ret;

		quot_diff *= fdata->quot_offset_scale;
		quot_diff += fdata->quot_offset_adjust;
	} else {
		quot_diff = fuse->quot - prev_fuse->quot;
	}

	freq_diff = fuse->max_freq - prev_fuse->max_freq;
	freq_diff = div_u64(freq_diff, 1000000); /* Convert to MHz */
	scaling = 1000 * quot_diff;
	do_div(scaling, freq_diff);
	return min(scaling, fdata->max_quot_scale);
}
EXPORT_SYMBOL_GPL(cpr_calculate_scaling);

int cpr_interpolate(const struct corner *corner, int step_volt,
		    const struct fuse_corner_data *fdata)
{
	u64 f_high, f_low, f_diff;
	int uV_high, uV_low, uV;
	u64 temp, temp_limit;
	const struct fuse_corner *fuse, *prev_fuse;

	fuse = corner->fuse_corner;
	prev_fuse = fuse - 1;

	f_high = fuse->max_freq;
	f_low = prev_fuse->max_freq;
	uV_high = fuse->uV;
	uV_low = prev_fuse->uV;
	f_diff = fuse->max_freq - corner->freq;

	/*
	 * Don't interpolate in the wrong direction. This could happen
	 * if the adjusted fuse voltage overlaps with the previous fuse's
	 * adjusted voltage.
	 */
	if (f_high <= f_low || uV_high <= uV_low || f_high <= corner->freq)
		return corner->uV;

	temp = f_diff * (uV_high - uV_low);
	temp = div64_ul(temp, f_high - f_low);

	/*
	 * max_volt_scale has units of uV/MHz while freq values
	 * have units of Hz.  Divide by 1000000 to convert to.
	 */
	temp_limit = f_diff * fdata->max_volt_scale;
	do_div(temp_limit, 1000000);

	uV = uV_high - min(temp, temp_limit);
	return roundup(uV, step_volt);
}
EXPORT_SYMBOL_GPL(cpr_interpolate);

int cpr_check_vreg_constraints(struct device *dev, struct regulator *vreg,
			       struct fuse_corner *f)
{
	int ret;

	ret = regulator_is_supported_voltage(vreg, f->min_uV, f->min_uV);
	if (!ret) {
		dev_err(dev, "min uV: %d not supported by regulator\n",
			f->min_uV);
		return -EINVAL;
	}

	ret = regulator_is_supported_voltage(vreg, f->max_uV, f->max_uV);
	if (!ret) {
		dev_err(dev, "max uV: %d not supported by regulator\n",
			f->max_uV);
		return -EINVAL;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(cpr_check_vreg_constraints);

MODULE_DESCRIPTION("Core Power Reduction (CPR) common");
MODULE_LICENSE("GPL");
