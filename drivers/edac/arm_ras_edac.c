// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2023, Linaro Limited
 *
 * Useful resources:
 * Arm RAS Supplement (rev. D.d)
 * ACPI for the Armv8 RAS Extensions 1.1 Platform Design Document (v1.1)
 * ARM Cortex <insert your core name> TRM
 * ARM DSU TRM
 */

#include <linux/acpi.h>
#include <linux/bitfield.h>
#include <linux/cpu_pm.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/smp.h>
#include <linux/types.h>

#include <acpi/actbl2.h>
#include <asm/cputype.h>
#include <asm/sysreg.h>

#include "edac_device.h"
#include "edac_mc.h"

#include "arm_ras_edac.h"

#define arm_ras_printk(level, fmt, arg...) edac_printk(level, "arm_ras", fmt, ##arg)

static const struct error_type err_type_cpu_cache[] = {
	{ edac_device_handle_ce, "Cache Corrected Error"	},
	{ edac_device_handle_ue, "Cache Deferred Error"		},
	{ edac_device_handle_ue, "Cache Uncorrected Error"	},
};

#define UNKNOWN_SERR		NULL
static const char *get_cpucache_err_msg(u64 errxstatus_serr)
{
	u32 part_num = read_cpuid_part_number();

	switch (errxstatus_serr) {
	case 0x0:
		switch (part_num) {
		case ARM_CPU_PART_CORTEX_A75:
		case ARM_CPU_PART_CORTEX_A76:
		case ARM_CPU_PART_CORTEX_A77:
		case ARM_CPU_PART_CORTEX_A78:
		case ARM_CPU_PART_CORTEX_A78C:
		case ARM_CPU_PART_CORTEX_X1:
		case ARM_CPU_PART_CORTEX_X1C:
			return "No error";

		default:
			return UNKNOWN_SERR;
		}

	case 0x1:
		switch (part_num) {
		case ARM_CPU_PART_CORTEX_A55:
		case ARM_CPU_PART_CORTEX_A76:
		case ARM_CPU_PART_CORTEX_A77:
		case ARM_CPU_PART_CORTEX_A78C:
		case ARM_CPU_PART_CORTEX_X1:
			return "Errors due to fault injection";
		case ARM_CPU_PART_CORTEX_A78:
		case ARM_CPU_PART_CORTEX_X1C:
			return "IMPLEMENTATION DEFINED error";

		default:
			return UNKNOWN_SERR;
		}

	case 0x2:
		switch (part_num) {
		case ARM_CPU_PART_CORTEX_A55:
		case ARM_CPU_PART_CORTEX_A75:
		case ARM_CPU_PART_CORTEX_A76:
		case ARM_CPU_PART_CORTEX_A77:
		case ARM_CPU_PART_CORTEX_A78:
		case ARM_CPU_PART_CORTEX_A78C:
		case ARM_CPU_PART_CORTEX_X1:
		case ARM_CPU_PART_CORTEX_X1C:
			return "ECC error from internal data buffer";

		default:
			return UNKNOWN_SERR;
		}

	case 0x6:
		switch (part_num) {
		case ARM_CPU_PART_CORTEX_A55:
		case ARM_CPU_PART_CORTEX_A75:
		case ARM_CPU_PART_CORTEX_A76:
		case ARM_CPU_PART_CORTEX_A77:
		case ARM_CPU_PART_CORTEX_A78:
		case ARM_CPU_PART_CORTEX_A78C:
		case ARM_CPU_PART_CORTEX_X1:
		case ARM_CPU_PART_CORTEX_X1C:
			return "ECC error on cache data RAM";

		default:
			return UNKNOWN_SERR;
		}

	case 0x7:
		switch (part_num) {
		case ARM_CPU_PART_CORTEX_A55:
		case ARM_CPU_PART_CORTEX_A75:
		case ARM_CPU_PART_CORTEX_A76:
		case ARM_CPU_PART_CORTEX_A77:
		case ARM_CPU_PART_CORTEX_A78:
		case ARM_CPU_PART_CORTEX_A78C:
		case ARM_CPU_PART_CORTEX_X1:
		case ARM_CPU_PART_CORTEX_X1C:
			return "ECC error on cache tag or dirty RAM";

		default:
			return UNKNOWN_SERR;
		}

	case 0x8:
		switch (part_num) {
		case ARM_CPU_PART_CORTEX_A55:
		case ARM_CPU_PART_CORTEX_A75:
		case ARM_CPU_PART_CORTEX_A76:
		case ARM_CPU_PART_CORTEX_A77:
		case ARM_CPU_PART_CORTEX_A78:
		case ARM_CPU_PART_CORTEX_A78C:
		case ARM_CPU_PART_CORTEX_X1:
		case ARM_CPU_PART_CORTEX_X1C:
			return "Parity error on TLB data RAM";

		default:
			return UNKNOWN_SERR;
		}

	case 0x9:
		switch (part_num) {
		case ARM_CPU_PART_CORTEX_A55:
		case ARM_CPU_PART_CORTEX_A75:
			return "Parity error on TLB tag RAM";

		default:
			return UNKNOWN_SERR;
		}

	case 0x12:
		switch (part_num) {
		case ARM_CPU_PART_CORTEX_A76:
		case ARM_CPU_PART_CORTEX_A77:
		case ARM_CPU_PART_CORTEX_A78:
		case ARM_CPU_PART_CORTEX_A78C:
		case ARM_CPU_PART_CORTEX_X1:
		case ARM_CPU_PART_CORTEX_X1C:
			return "Error response for a cache copyback";

		default:
			return UNKNOWN_SERR;
		}

	case 0x15:
		switch (part_num) {
		case ARM_CPU_PART_CORTEX_A55:
		case ARM_CPU_PART_CORTEX_A75:
		case ARM_CPU_PART_CORTEX_A76:
		case ARM_CPU_PART_CORTEX_A77:
		case ARM_CPU_PART_CORTEX_A78:
		case ARM_CPU_PART_CORTEX_A78C:
		case ARM_CPU_PART_CORTEX_X1:
		case ARM_CPU_PART_CORTEX_X1C:
			return "Deferred error not supported";

		default:
			return UNKNOWN_SERR;
		}

	default:
		return UNKNOWN_SERR;
	}
}

static const char *get_error_msg_cpu_cache(struct arm_ras_device *ras_edac, u64 errxstatus)
{
	u32 part_num = read_cpuid_part_number();
	u32 errxstatus_serr = 0;

	if (ras_edac->node->intf.type == ACPI_AEST_NODE_SYSTEM_REGISTER) {
		switch (part_num) {
		case ARM_CPU_PART_CORTEX_A76:
		case ARM_CPU_PART_CORTEX_A77:
		case ARM_CPU_PART_CORTEX_A78C:
		case ARM_CPU_PART_CORTEX_X1:
		case ARM_CPU_PART_CORTEX_X1C:
			errxstatus_serr = FIELD_GET(ERRXSTATUS_SERR_4, errxstatus);
			break;

		case ARM_CPU_PART_CORTEX_A55:
		case ARM_CPU_PART_CORTEX_A75:
		case ARM_CPU_PART_CORTEX_A78:
			errxstatus_serr = FIELD_GET(ERRXSTATUS_SERR_7, errxstatus);
			break;

		default:
			arm_ras_printk(KERN_ERR, "Missing core data for partnum 0x%x", part_num);
			return NULL;
		}

		return get_cpucache_err_msg(errxstatus_serr);
	} else {
		return NULL;
	}

	return NULL;
}

static void dump_syndrome_reg(int error_type, int index,
			      u64 errxstatus, u64 errxmisc,
			      struct edac_device_ctl_info *edev_ctl)
{
	struct arm_ras_device *ras_edac = edev_ctl->pvt_info;
	const struct error_type *err_type;
	char msg[ARM_RAS_EDAC_MSG_MAX];
	const char *error_msg;
	int cpu;

	cpu = raw_smp_processor_id();

	snprintf(msg, ARM_RAS_EDAC_MSG_MAX, "CPU%d: ", cpu);

	switch (ras_edac->node->hdr.type) {
	case ACPI_AEST_PROCESSOR_ERROR_NODE:
		switch (ras_edac->node->processor.proc.resource_type) {
		case ACPI_AEST_CACHE_RESOURCE:
			err_type = &err_type_cpu_cache[0];
			snprintf(msg, ARM_RAS_EDAC_MSG_MAX, "L%d ", index);
			error_msg = get_error_msg_cpu_cache(ras_edac, errxstatus);
			break;

		default:
			pr_err("Unknown processor error!\n");
			return;
		}
		break;

	default:
		pr_err("Unknown processor error!\n");
		return;
	}

	if (!error_msg) {
		arm_ras_printk(KERN_WARNING, "found an error of unknown type\n");
		return;
	}

	snprintf(msg, ARM_RAS_EDAC_MSG_MAX,
		 "%s, ERRXSTATUS_EL1:%#llx ERRXMISC0_EL1:%#llx, %s",
		 err_type[error_type].msg, errxstatus, errxmisc, error_msg);

	err_type[error_type].fn(edev_ctl, 0, index, msg);
}

static void arm_ras_clear_error(u64 errxstatus)
{
	write_sysreg_s(errxstatus, SYS_ERXSTATUS_EL1);
	isb();
}

/* Check if at least one error has been recorded */
static bool arm_ras_check_regs_valid(u64 errxstatus)
{
	return !!(FIELD_GET(ERRXSTATUS_VALID, errxstatus) && FIELD_GET(ERRXSTATUS_MV, errxstatus));
}

static void arm_ras_check_err_type(u64 errxstatus, u64 errxmisc,
				   struct edac_device_ctl_info *edev_ctl,
				   int index)
{
	u32 errxstatus_ue, errxstatus_ce, errxstatus_de;

	errxstatus_ce = FIELD_GET(ERRXSTATUS_CE, errxstatus);
	errxstatus_de = FIELD_GET(ERRXSTATUS_DE, errxstatus);
	errxstatus_ue = FIELD_GET(ERRXSTATUS_UE, errxstatus);

	if (errxstatus_ce)
		dump_syndrome_reg(TYPE_CE, index, errxstatus, errxmisc, edev_ctl);
	else if (errxstatus_de)
		dump_syndrome_reg(TYPE_DE, index, errxstatus, errxmisc, edev_ctl);
	else if (errxstatus_ue)
		dump_syndrome_reg(TYPE_UE, index, errxstatus, errxmisc, edev_ctl);
	else
		arm_ras_printk(KERN_ERR, "Unknown error\n");
}

/* Check for errors on cores implementing ERR<n>MISC0.LVL[3:1] */
static void arm_ras_check_ecc_lvl(struct edac_device_ctl_info *edev_ctl,
				  u64 errxstatus, u64 errxmisc, int cpu)
{
	u8 lvl = FIELD_GET(ERRXMISC0_LVL, errxmisc);

	switch (lvl) {
	case ERRXMISC0_LVL_L1:
		arm_ras_check_err_type(errxstatus, errxmisc, edev_ctl, INDEX_L1);
		break;

	case ERRXMISC0_LVL_L2:
		arm_ras_check_err_type(errxstatus, errxmisc, edev_ctl, INDEX_L2);
		break;

	case ERRXMISC0_LVL_L3:
		arm_ras_check_err_type(errxstatus, errxmisc, edev_ctl, INDEX_L3);
		break;

	default:
		arm_ras_printk(KERN_ERR, "cpu:%d unknown error: %lu\n", cpu, lvl);
	}
}

/* Check for errors on cores implementing ERR<n>MISC0.UNIT[3:0] */
static void arm_ras_check_ecc_unit(struct edac_device_ctl_info *edev_ctl,
				   u64 errxstatus, u64 errxmisc, int cpu)
{
	u8 unit = FIELD_GET(ERRXMISC0_UNIT, errxmisc);

	switch (unit) {
	case ERRXMISC0_UNIT_L1_IC:
	case ERRXMISC0_UNIT_L1_DC:
		arm_ras_check_err_type(errxstatus, errxmisc, edev_ctl, INDEX_L1);
		break;

	case ERRXMISC0_UNIT_L2:
	case ERRXMISC0_UNIT_L2_TLB:
		arm_ras_check_err_type(errxstatus, errxmisc, edev_ctl, INDEX_L2);
		break;

	default:
		arm_ras_printk(KERN_ERR, "cpu:%d unknown error: %lu\n", cpu, unit);
	}
}

static void arm_ras_check_cpu_cache_ecc(struct edac_device_ctl_info *edev_ctl,
					u64 errxstatus, u32 part_num, int cpu)
{
	u64 errxmisc;

	/* Unfortunately, ERR<n>MISC0 is almost entirely IMPLEMENTATION DEFINED */
	errxmisc = read_sysreg_s(SYS_ERXMISC0_EL1);

	/* Check if UNIT/(LVL+IND) are empty for some reason (e.g. tz handled that) */
	if (!(FIELD_GET(ERRXMISC0_UNIT, errxmisc)))
		return;

	switch(part_num) {
	/* Cores implementing ERR<n>MISC0.LVL[3:1] */
	case ARM_CPU_PART_CORTEX_A55:
	case QCOM_CPU_PART_KRYO_4XX_SILVER:
		arm_ras_check_ecc_lvl(edev_ctl, errxstatus, errxmisc, cpu);
		break;

	/* Cores implementing ERR<n>MISC0.UNIT[3:0] */
	case ARM_CPU_PART_CORTEX_A76:
	case ARM_CPU_PART_CORTEX_A77:
	case ARM_CPU_PART_CORTEX_A78:
	case ARM_CPU_PART_CORTEX_A78C:
	case ARM_CPU_PART_CORTEX_X1:
	case ARM_CPU_PART_CORTEX_X1C:
		arm_ras_check_ecc_unit(edev_ctl, errxstatus, errxmisc, cpu);
		break;

	default:
		arm_ras_printk(KERN_ERR, "Missing core data for partnum 0x%x", part_num);
		return;
	}
}

static void arm_ras_check_ecc(void *info)
{
	struct edac_device_ctl_info *edev_ctl = info;
	struct arm_ras_device *ras_edac = edev_ctl->pvt_info;
	struct acpi_aest_node *node = ras_edac->node;
	u32 part_num = read_cpuid_part_number();
	u32 err_rec_idx;
	u64 errxstatus;
	int cpu, i;

	cpu = smp_processor_id();

	for (i = 0; i < node->intf.error_record_count; i++) {
		err_rec_idx = node->intf.error_record_index + i;

		errxstatus = read_sysreg_s(SYS_ERXSTATUS_EL1);
		if (!arm_ras_check_regs_valid(errxstatus))
			return;

		write_sysreg_s(err_rec_idx, SYS_ERRSELR_EL1);
		isb();

		switch (ras_edac->node->hdr.type) {
		case ACPI_AEST_PROCESSOR_ERROR_NODE:
			switch (ras_edac->node->processor.proc.resource_type) {
			case ACPI_AEST_CACHE_RESOURCE:
				arm_ras_check_cpu_cache_ecc(edev_ctl, errxstatus,
								 part_num, cpu);
				break;

			default:
				return;
			}
			break;

		default:
			return;
		}
		
		arm_ras_clear_error(errxstatus);
	}
}

#define AFFINE_TO_HIGHER_LEVEL(bitfield, errdevaff) \
	FIELD_GET(bitfield, errdevaff) == MPIDR_AFF_HIGHER_LEVEL

/*
 * ERRDEVAFF.Affn fields look like 0bx..x10..0
 * x-es are 'useful' bits which contain the value of MPIDR_EL1.Affn.
 * The last 'useful' bit is the one followed by a 1 and any amount of zeroes (there can be none).
 * We're guaranteed affn != BIT(7), as that's handled by the AFFINE_TO_HIGHER_LEVEL case and
 * we're architecturally guaranteed that this register will not read 0 (that is, unless some
 * vendor screws up..).
 */
static int find_lowest_affinity_bit(u32 affn)
{
	/*
	 * ffs() finds the lowest bit set, but it's 1-based. That means it
	 * will give us precisely the lowest "useful" bit!
	 */
	return ffs(affn);
}

/*
 * For a group of error records that has affinity with a single
 * Processing Element (e.g. a CPU core) ERRDEVAFF ⊆ MPIDR_EL1.
 *   - Arm A-profile Architecture Registers doc
 */
static void set_cpumask_from_errdevaff(struct arm_ras_device *ras_edac)
{
	u64 aff_mask = ras_edac->node->processor.proc.processor_affinity;
	u64 errdevaff, mpidr_aff;
	u8 lowest_bit;
	int cpu;

	/*
	 * NOTE:
	 * This is all "viewed from the highest exception level of the associated PEs"
	 * which can mean that some of the PEs may not exist from our POV (e.g. our
	 * hypervisor didn't do a good job of trapping and altering ERRDEVAFF and it
	 * mentions CPUs that don't exist on our virtual machine).
	 */
	for_each_possible_cpu(cpu) {
		preempt_disable();

		if (aff_mask & ERRDEVAFF_F0V) {
			/* ERRDEVAFF.Aff0 is valid, and the PE affinity is at level 0. */
			mpidr_aff = FIELD_GET(MPIDR_AFF0, read_sysreg_s(SYS_MPIDR_EL1));
			errdevaff = FIELD_GET(MPIDR_AFF0, aff_mask);
		} else {
			/*
			 * ERRDEVAFF.Aff0 is not valid, and the PE affinity is
			 * above level 0 or a subset of level 0.
			 */
			if (AFFINE_TO_HIGHER_LEVEL(MPIDR_AFF0, aff_mask)) {
				/* PE affinity is at level 1. */
				mpidr_aff = FIELD_GET(MPIDR_AFF1, read_sysreg_s(SYS_MPIDR_EL1));
				errdevaff = FIELD_GET(MPIDR_AFF1, aff_mask);
			} else {
				/* PE affinity is above level 1 or a subset of level 1. */
				if (AFFINE_TO_HIGHER_LEVEL(MPIDR_AFF1, aff_mask)) {
					/* PE affinity is at level 2. */
					mpidr_aff = FIELD_GET(MPIDR_AFF2, read_sysreg_s(SYS_MPIDR_EL1));
					errdevaff = FIELD_GET(MPIDR_AFF2, aff_mask);
				} else {
					/* PE affinity is above level 2 or a subset of level 2. */
					if (AFFINE_TO_HIGHER_LEVEL(MPIDR_AFF2, aff_mask)) {
						/* PE affinity is at level 3 (highest). */
						mpidr_aff = FIELD_GET(MPIDR_AFF3, read_sysreg_s(SYS_MPIDR_EL1));
						errdevaff = FIELD_GET(MPIDR_AFF3, aff_mask);
					}
				}
			}
		}

		lowest_bit = find_lowest_affinity_bit(errdevaff);
		mpidr_aff >>= lowest_bit;
		errdevaff >>= lowest_bit;

		/*
		 * Depending on the affinity level, errdevaff[x:y] should be a direct copy
		 * of mpidr, so for example if the affinity level is core, then only one
		 * will match and if it's cluster, CLUSTER_SIZE() cores will match
		 */
		if (mpidr_aff == errdevaff) {
			pr_debug("cpu %d matched!", cpu);
			cpumask_set_cpu(cpu, &ras_edac->cpu_mask);
		}
		preempt_enable();
	}
}

static void arm_ras_run_on_relevant_cpus(struct arm_ras_device *ras_edac,
					 smp_call_func_t func, void *info)
{
	struct acpi_aest_processor *proc = &ras_edac->node->processor.proc;
	int cpu;

	/* No *info is okay, no function not so much.. */
	if (WARN_ON(!func))
		return;

	/* Global nodes only need to be touched once, no matter by which CPU */
	if (!IS_PROCESSOR_NODE(ras_edac->node) ||
	    (IS_PROCESSOR_NODE(ras_edac->node) && proc->flags & PROC_NODE_GLOBAL)) {
		func(info);
		return;
	}

	/* 
	 * !global (dedicated) nodes are affined to specific CPUs.
	 * !shared (private) nodes only belong to a single CPU, but that's
	 * reflected in the associated cpu_mask.
	 */
	for_each_cpu(cpu, &ras_edac->cpu_mask) {
		func(info);
	}
}

static irqreturn_t arm_ras_irq_handler(int irq, void *info)
{
	struct edac_device_ctl_info *edev_ctl = (void *)info;

	/* Check for errors in each error record associated with this intf */
	arm_ras_check_ecc(edev_ctl);

	return IRQ_HANDLED;
}

static void arm_ras_irq_disable(void *drvdata)
{
	int irq = *(int *)drvdata;

	disable_percpu_irq(irq);
}

static void arm_ras_irq_enable(void *drvdata)
{
	int irq = *(int *)drvdata;

	/* This *theoretically* could also be edge-triggered, but does it really matter? */
	enable_percpu_irq(irq, IRQ_TYPE_LEVEL_HIGH);
}

static struct edac_device_ctl_info __percpu *edac_dev;

/* Set up IRQs for dedicated nodes */
static int arm_ras_setup_irq_dedicated(struct platform_device *pdev,
				       struct acpi_aest_node *node,
				       struct edac_device_ctl_info *edev_ctl)
{
	struct arm_ras_device *ras_edac = edev_ctl->pvt_info;
	int cpu, errirq = 0, faultirq = 0, ret;

	edac_dev = devm_alloc_percpu(&pdev->dev, struct edac_device_ctl_info);
	if (!edac_dev)
		return -ENOMEM;

	preempt_disable();

	for_each_cpu(cpu, &ras_edac->cpu_mask) {
		per_cpu(edac_dev, cpu) = edev_ctl;
	}

	ret = platform_get_irq_byname_optional(pdev, "fault");
	if (ret >= 0) {
		faultirq = ret;

		ret = request_percpu_irq(faultirq, arm_ras_irq_handler,
					 "ras_dedicated_faultirq", this_cpu_ptr(edac_dev));
		if (ret) {
			arm_ras_printk(KERN_ERR,
				       "Failed to request dedicated fault irq: %d\n", faultirq);
			faultirq = 0;
			goto out_err;
		}

		arm_ras_run_on_relevant_cpus(ras_edac, arm_ras_irq_enable, &faultirq);
	}

	ret = platform_get_irq_byname_optional(pdev, "err");
	if (ret >= 0) {
		errirq = ret;

		ret = request_percpu_irq(errirq, arm_ras_irq_handler,
					 "ras_dedicated_errirq", this_cpu_ptr(edac_dev));
	
		if (ret) {
			arm_ras_printk(KERN_ERR,
				       "Failed to request dedicated err irq: %d\n", errirq);
			errirq = 0;
			goto out_err;
		}

		arm_ras_run_on_relevant_cpus(ras_edac, arm_ras_irq_enable, &errirq);
	}

	preempt_enable();

	return 0;

out_err:
	preempt_enable();

	if (faultirq)
		free_percpu_irq(faultirq, &edac_dev);

	if (errirq)
		free_percpu_irq(errirq, &edac_dev);

	return ret;
}

/* Set up IRQs for global nodes */
static int arm_ras_setup_irq_global(struct platform_device *pdev, void *edev_ctl)
{
	int errirq, faultirq, ret;

	ret = platform_get_irq_byname_optional(pdev, "fault");
	if (ret >= 0) {
		faultirq = ret;
		ret = devm_request_irq(&pdev->dev, faultirq, arm_ras_irq_handler,
				       IRQF_ONESHOT | IRQF_TRIGGER_HIGH,
				       "ras_global_faultirq", edev_ctl);

		if (ret) {
			arm_ras_printk(KERN_ERR,
				       "Failed to request global fault irq: %d\n", faultirq);
			return ret;
		}
	}

	ret = platform_get_irq_byname_optional(pdev, "err");
	if (ret >= 0) {
		errirq = ret;
		ret = devm_request_irq(&pdev->dev, errirq, arm_ras_irq_handler,
				       IRQF_ONESHOT | IRQF_TRIGGER_HIGH,
				       "ras_global_errirq", edev_ctl);

		if (ret) {
			arm_ras_printk(KERN_ERR,
			               "Failed to request global err irq: %d\n", faultirq);
			return ret;
		}
	}

	return 0;
}

static int arm_ras_setup_irq(struct platform_device *pdev,
				  struct arm_ras_device *ras_edac)
{
	u8 proc_flags;
	int ret = 0;

	if (IS_PROCESSOR_NODE(ras_edac->node))
		proc_flags = ras_edac->node->processor.proc.flags;

	if (IS_PROCESSOR_NODE(ras_edac->node) && (~proc_flags & PROC_NODE_GLOBAL))
		ret = arm_ras_setup_irq_dedicated(pdev, ras_edac->node, ras_edac->edev_ctl);
	else
		ret = arm_ras_setup_irq_global(pdev, ras_edac->edev_ctl);

	return ret;
}

static void arm_ras_poll_cache_error(struct edac_device_ctl_info *edev_ctl)
{
	struct arm_ras_device *ras_edac = edev_ctl->pvt_info;
	struct acpi_aest_node *node = ras_edac->node;
	u8 proc_flags;

	if (IS_PROCESSOR_NODE(node))
		proc_flags = node->processor.proc.flags;

	if (IS_PROCESSOR_NODE(node) && (proc_flags & PROC_NODE_GLOBAL))
		arm_ras_check_ecc(edev_ctl);
	else
		arm_ras_run_on_relevant_cpus(ras_edac, arm_ras_check_ecc, edev_ctl);
}

static inline void arm_ras_enable_err_record(void)
{
	write_sysreg_s(ERRXCTLR_ENABLE, SYS_ERXCTLR_EL1);
	write_sysreg_s(ERRXMISC0_CECR | ERRXMISC0_CECO, SYS_ERXMISC0_EL1);
	isb();
}

static inline void arm_ras_init(void *info)
{
	acpi_aest_node_interface intf = *(acpi_aest_node_interface *)info;
	int i;

	for (i = intf.error_record_index; i < intf.error_record_count; i++) {
		if (BIT(i) & intf.error_record_implemented) {
			write_sysreg_s(i, SYS_ERRSELR_EL1);
			/* Make sure we're really interacting with the correct error record. */
			isb();
			arm_ras_enable_err_record();
		}
	}
}

static int arm_ras_pm_notify(struct notifier_block *nb, unsigned long action,
				  void *data)
{
	struct arm_ras_device *ras_edac = container_of(nb, struct arm_ras_device, nb);

	switch (action) {
	case CPU_PM_EXIT:
		arm_ras_init(&ras_edac->node->intf);
		break;

	default:
		return NOTIFY_DONE;
	}

	return NOTIFY_OK;
}

static struct notifier_block arm_ras_pm_nb = {
	.notifier_call = arm_ras_pm_notify,
};

#define MAX_SYSFS_NAME_LEN 64
static int arm_ras_of_probe_common(struct device *dev,
					struct arm_ras_device *ras_edac)
{
	struct device_node *of_node = dev->of_node;
	int ret = 0;

	ret = of_property_read_u32(of_node, "arm,edac-start-err-rec",
				   &ras_edac->node->intf.error_record_index);
	if (ret)
		return ret;

	ret = of_property_read_u32(of_node, "arm,edac-num-err-rec",
				   &ras_edac->node->intf.error_record_count);
	if (ret)
		return ret;

	ret = of_property_read_u64(of_node, "arm,edac-impl-err-rec",
				   &ras_edac->node->intf.error_record_implemented);
	if (ret == -EINVAL) {
		/*
		 * If unspecified, assume the error records are at consecutive indices
		 * in [start-err-rec ; num-err-rec - 1]
		 */
		ras_edac->node->intf.error_record_implemented = ~0ULL;
		return 0;
	}

	return ret;
}

static int arm_ras_of_probe_proc_common(struct device *dev,
					     struct arm_ras_device *ras_edac)
{
	struct acpi_aest_node *node = ras_edac->node;
	u8 *res_type = &node->processor.proc.resource_type;
	u8 *proc_flags = &node->processor.proc.flags;
	struct device_node *of_node = dev->of_node;
	struct device_node *cpu_node;
	int ret, ncpus, i;

	ret = of_property_read_u8(of_node, "arm,cpu-resource-type", res_type);
	if (ret)
		return ret;

	if (of_get_property(of_node, "arm,edac-global-node", NULL))
		*proc_flags |= PROC_NODE_GLOBAL;

	if (of_get_property(of_node, "arm,edac-shared-node", NULL))
		*proc_flags |= PROC_NODE_SHARED;

	/*
	 * ACPI stores a copy of what-would-be ERRDEVAFF for TYPE_SR, but with DT we can simply
	 * pass phandles to the relevant CPUs and not have to come up with MPIDR contents which is
	 * both easier to do and makes the property orders of magnitude more readable.
	 */
	ncpus = of_count_phandle_with_args(of_node, "affined-cpus", NULL);
	for (i = 0; i < ncpus; i++) {
		cpu_node = of_parse_phandle(of_node, "affined-cpus", i);
		if (!cpu_node)
			return -EINVAL;

		if (*proc_flags & ~(PROC_NODE_SHARED)) {
			/* Must be precisely a single entry for a private node. */
			BUG_ON(ncpus != 1);
		}

		cpumask_set_cpu(of_cpu_node_to_id(cpu_node), &ras_edac->cpu_mask);
		of_node_put(cpu_node);

		pr_debug("ncpus = %u, cpumask = 0x%x", ncpus, ras_edac->cpu_mask);
	}

	if (*proc_flags & PROC_NODE_GLOBAL)
		cpumask_copy(&ras_edac->cpu_mask, cpu_possible_mask);

	return ret;
}

static int arm_ras_of_probe_proc_cache(struct device *dev,
					    struct arm_ras_device *ras_edac)
{
	char sysfs_name[MAX_SYSFS_NAME_LEN] = "arm_ras";
	struct device_node *of_node = dev->of_node;
	int cnt, ret, i;
	u32 levels[7];

	cnt = of_property_count_u32_elems(of_node, "cache-levels");
	if (cnt < 0)
		return cnt;
	/* armv8 allows no more than 7 levels of cache */
	if (cnt > 7)
		dev_err_probe(dev, -EINVAL, "More than 7 cache levels specified\n");

	ret = of_property_read_u32_array(of_node, "cache-levels", levels, cnt);
	if (ret)
		return ret;

	/* Make the names unique by including the correlated cache levels. */
	for (i = 0; i < cnt; i++)
		snprintf(sysfs_name, MAX_SYSFS_NAME_LEN, "%s_l%d", sysfs_name, levels[i]);

	snprintf(sysfs_name, MAX_SYSFS_NAME_LEN, "%s_edac", sysfs_name);

	/* TODO: now for ex. <1 3 5> the helper will call the entries L1 L2 L3 :/ */
	ras_edac->edev_ctl = edac_device_alloc_ctl_info(0, sysfs_name, 1, "L", cnt, levels[0],
							NULL, 0, edac_device_alloc_index());
	if (!ras_edac->edev_ctl)
		return -ENOMEM;

	return ret;
}

static int arm_ras_acpi_probe_common(struct device *dev,
				     struct arm_ras_device *ras_edac)
{
	struct acpi_aest_node *node;

	node = *(struct acpi_aest_node **)dev_get_platdata(dev);
	if (!node)
		return -EINVAL;

	ras_edac->node = node;

	return 0;
}

static int arm_ras_acpi_probe_proc_cache(struct device *dev,
					 struct arm_ras_device *ras_edac)
{
	char sysfs_name[MAX_SYSFS_NAME_LEN] = "arm_ras";
	acpi_aest_processor_cache cache = ras_edac->node->processor.proc_sub.cache;

	/*
	 * There's no trivial way to revtrieve cache levels from ACPI without brute-searching
	 * through PPTT for each and every one of references, so the next best thing to do is
	 * using any other unique property, which in this case could be the cache reference,
	 * as it's expected we only have at most a single AEST node per cache.
	 */
	snprintf(sysfs_name, MAX_SYSFS_NAME_LEN, "%s_%u", sysfs_name, cache.cache_reference);

	/*
	 * Looks like ACPI is only expected to pass a single cache reference per node..
	 * This does not seem to be a spec limitation though, as QC SM8250 shares a node for L1/L2.
	 */
	ras_edac->edev_ctl = edac_device_alloc_ctl_info(0, sysfs_name, 1, "cache", 1, 0,
							NULL, 0, edac_device_alloc_index());
	if (!ras_edac->edev_ctl)
		return -ENOMEM;

	return 0;
}

static int arm_ras_probe(struct platform_device *pdev)
{
	struct acpi_aest_node_interface *intf;
	struct arm_ras_device *ras_edac;
	struct device *dev = &pdev->dev;
	struct acpi_aest_node *node;
	struct resource *res;
	int cpu, ret;

	/* We need this to be non-zero, as this indicates at least RASv1 is implemented */
	if (!FIELD_GET(ID_AA64PFR0_EL1_RAS, read_sysreg_s(SYS_ID_AA64PFR0_EL1)))
		return dev_err_probe(dev, -EOPNOTSUPP,
				     "RAS extensions not supported on at least one CPU\n");

	for_each_possible_cpu(cpu) {
		/* If features from ERRXFR_SUPPORTED are absent, this driver will not work.. */
		if (!FIELD_GET(ERRXFR_SUPPORTED, read_sysreg_s(SYS_ERXFR_EL1)))
			return dev_err_probe(dev, -EOPNOTSUPP,
					"RAS extensions not supported on at least one CPU\n");
	}

	ras_edac = devm_kzalloc(dev, sizeof(*ras_edac), GFP_KERNEL);
	if (!ras_edac)
		return -ENOMEM;

	node = devm_kzalloc(dev, sizeof(*node), GFP_KERNEL);
	if (!node)
		return -ENOMEM;

	ras_edac->node = node;
	intf = &ras_edac->node->intf;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res) {
		intf->type = ACPI_AEST_NODE_MEMORY_MAPPED;
		intf->address = res->start;
	} else
		intf->type = ACPI_AEST_NODE_SYSTEM_REGISTER;

	if (intf->type == ACPI_AEST_NODE_SYSTEM_REGISTER) {
		/* Do nothing, in the future tell the wrapper (\/) to behave correctly. */
	}
	else if (intf->type == ACPI_AEST_NODE_MEMORY_MAPPED) {
		/* Unimpl, but should just be a wrapper for r/w system registers w/ a LUT */
		return -EOPNOTSUPP;
	}
	else {
		/* How did we get here? Super secret Arm board? Or broken ACPI?  */
		return -EINVAL;
	}

	if (has_acpi_companion(dev)) {
		ret = arm_ras_acpi_probe_common(&pdev->dev, ras_edac);
		if (ret)
			return ret;

		switch (ras_edac->node->hdr.type) {
		case ACPI_AEST_PROCESSOR_ERROR_NODE:
			/*
			 * With DT, we calculated the CPU mask based on CPU phandles. ACPI only
			 * gives us a copy of ERRDEVAFF and we have to make it do..
			 * Calculate a cpumask based on this and cache it so that we only have
			 * to care about it once per node.
			 */
			set_cpumask_from_errdevaff(ras_edac);

			switch (ras_edac->node->processor.proc.resource_type) {
			case ACPI_AEST_CACHE_RESOURCE:
				ret = arm_ras_acpi_probe_proc_cache(&pdev->dev, ras_edac);
				break;

			default:
				return -EOPNOTSUPP;
				break;
			}
			break;

		default:
			/* Only CPU Ln$ EDAC is supported for now. */
			return -EOPNOTSUPP;
		}
	} else {
		ras_edac->node->hdr.type = (u8)(uintptr_t)of_device_get_match_data(dev);

		ret = arm_ras_of_probe_common(&pdev->dev, ras_edac);
		if (ret)
			return ret;

		switch (ras_edac->node->hdr.type) {
		case ACPI_AEST_PROCESSOR_ERROR_NODE:
			ret = arm_ras_of_probe_proc_common(&pdev->dev, ras_edac);
			if (ret)
				return ret;

			switch (ras_edac->node->processor.proc.resource_type) {
			case ACPI_AEST_CACHE_RESOURCE:
				ret = arm_ras_of_probe_proc_cache(&pdev->dev, ras_edac);
				break;

			default:
				return -EOPNOTSUPP;
			}
			break;

		default:
			/* Only CPU Ln$ EDAC is supported for now. */
			return -EOPNOTSUPP;
		}
	}

	if (ret)
		return ret;

	ras_edac->nb = arm_ras_pm_nb;
	ras_edac->edev_ctl->ctl_name = "arm_ras";
	ras_edac->edev_ctl->dev = dev;
	ras_edac->edev_ctl->dev_name = dev_name(dev);
	ras_edac->edev_ctl->edac_check = arm_ras_poll_cache_error;
	ras_edac->edev_ctl->mod_name = "arm_ras";
	ras_edac->edev_ctl->panic_on_ue = 1; /* TODO: module param? */
	ras_edac->edev_ctl->poll_msec = poll_msec;
	ras_edac->edev_ctl->pvt_info = ras_edac;

	platform_set_drvdata(pdev, ras_edac);

	arm_ras_run_on_relevant_cpus(ras_edac, arm_ras_init, &ras_edac->node->intf);

	ret = edac_device_add_device(ras_edac->edev_ctl);
	if (ret)
		goto out_mem;

	ret = arm_ras_setup_irq(pdev, ras_edac);
	if (ret)
		goto out_dev;

	ret = cpu_pm_register_notifier(&ras_edac->nb);
	if (ret)
		goto out_dev;

	return ret;

out_dev:
	edac_device_del_device(ras_edac->edev_ctl->dev);
out_mem:
	edac_device_free_ctl_info(ras_edac->edev_ctl);

	return ret;
}

static void arm_ras_teardown(struct platform_device *pdev)
{
	struct arm_ras_device *ras_edac = platform_get_drvdata(pdev);
	struct acpi_aest_node *node = ras_edac->node;
	int errirq, faultirq;
	u8 proc_flags;

	if (IS_PROCESSOR_NODE(node))
		proc_flags = node->processor.proc.flags;

	/* Global nodes use devm_ irq registration */
	if (IS_PROCESSOR_NODE(node) && (~proc_flags & PROC_NODE_GLOBAL)) {
		faultirq = platform_get_irq_byname_optional(pdev, "fault");
		if (faultirq >= 0) {
			arm_ras_run_on_relevant_cpus(ras_edac, arm_ras_irq_disable, &faultirq);
			free_percpu_irq(faultirq, &edac_dev);
		}

		errirq = platform_get_irq_byname_optional(pdev, "err");
		if (errirq >= 0) {
			arm_ras_run_on_relevant_cpus(ras_edac, arm_ras_irq_disable, &errirq);
			free_percpu_irq(errirq, &edac_dev);
		}
	}

	cpu_pm_unregister_notifier(&ras_edac->nb);
}

static int arm_ras_remove(struct platform_device *pdev)
{
	struct arm_ras_device *ras_edac = platform_get_drvdata(pdev);
	struct edac_device_ctl_info *edev_ctl = ras_edac->edev_ctl;

	arm_ras_teardown(pdev);

	edac_device_del_device(edev_ctl->dev);
	edac_device_free_ctl_info(edev_ctl);

	return 0;
}

static const struct of_device_id arm_ras_of_match[] = {
	{ .compatible = "arm,ras-edac-cpu", .data = (void *)ACPI_AEST_PROCESSOR_ERROR_NODE },
	{ }
};
MODULE_DEVICE_TABLE(of, arm_ras_of_match);

static struct platform_driver arm_ras_driver = {
	.probe = arm_ras_probe,
	.remove = arm_ras_remove,
	.driver = {
		.name = "arm-ras-edac",
		.of_match_table	= arm_ras_of_match,
	},
};
module_platform_driver(arm_ras_driver);

MODULE_DESCRIPTION("Arm RAS EDAC driver");
MODULE_LICENSE("GPL");