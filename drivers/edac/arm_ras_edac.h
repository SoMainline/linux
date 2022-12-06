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

#ifndef __EDAC_ARM_RAS_EDAC_H__
#define __EDAC_ARM_RAS_EDAC_H__

struct arm_ras_device {
	cpumask_t cpu_mask;
	struct edac_device_ctl_info *edev_ctl;
	struct notifier_block nb;
	struct acpi_aest_node *node;
};

/*
 * True  - Node interface is shared. For processor cache nodes, the sharing is
 *        restricted to the processors that share the indicated cache.
 *
 * False - Node interface is private to the node.
 *
 * INTF_SHARED is only valid for INTERFACE_TYPE_SR.
 */
#define INTF_SHARED		BIT(0)

/*
 * A global node is a single representative of error nodes of this resource
 * type for all processors in the system.
 *
 * True  - This is a global node.
 * False - This is a dedicated node.
 */
#define PROC_NODE_GLOBAL	BIT(0)
/*
 * True  - This node represents a resource that is shared by multiple processors.
 * False - This node represents a resource that is private to the specified processor.
 */
#define PROC_NODE_SHARED	BIT(1)

/*
 * ARM Cortex-A55, Cortex-A75, Cortex-A76 TRM Chapter B3.3
 * ARM DSU TRM Chapter B2.3
 *
 * ED = Error Detection
 * UI = Uncorrected error recovery interrupt
 * FI = Fault handling interrupt
 * CFI = Corrected Fault Handling interrupt
 */
#define ERRXCTLR_ED		BIT(0)
#define ERRXCTLR_UI		BIT(2)
#define ERRXCTLR_FI		BIT(3)
#define ERRXCTLR_RESERVED	GENMASK(7, 4)
#define ERRXCTLR_CFI		BIT(8)
#define ERRXCTLR_RESERVED0	GENMASK(63, 9)
#define ERRXCTLR_ENABLE	(ERRXCTLR_CFI | ERRXCTLR_FI | ERRXCTLR_UI | ERRXCTLR_ED)

/*
 * ARM Cortex-A55, Cortex-A75, Cortex-A76 TRM Chapter B3.4
 * ARM DSU TRM Chapter B2.4
 */
/* Whether error detection is controllable */
#define ERRXFR_ED		GENMASK(1, 0)
/* Enable defered errors. */
#define ERRXFR_DE		GENMASK(3, 2)
/* Error recovery interrupt for uncorrected errors is implemented.  */
#define ERRXFR_UI		GENMASK(5, 4)
/* Fault recovery interrupt for uncorrected errors is implemented. */
#define ERRXFR_FI		GENMASK(7, 6)
/* In-band uncorrected error reporting is implemented. */
#define ERRXFR_UE		GENMASK(9, 8)
/* Whether it's possible to en/disable fault handling interrupts on corrected errors. */
#define ERRXFR_CFI		GENMASK(11, 10)
/* Whether the node implements an 8-bit standard CE counter in ERR0MISC0[39:32]. */
#define ERRXFR_CEC		GENMASK(14, 12)
/* A first repeat counter and a second other counter are implemented. */
#define ERRXFR_RP		BIT(15)
#define ERRXFR_SUPPORTED	(ERRXFR_ED | ERRXFR_DE | \
				 ERRXFR_UI | ERRXFR_FI | \
				 ERRXFR_UE | ERRXFR_CFI | \
				 ERRXFR_CEC | ERRXFR_RP)

/*
 * ARM Cortex-A55, Cortex-A75, Cortex-A76 TRM Chapter B3.5
 * ARM DSU TRM Chapter B2.5
 */
#define ERRXMISC0_CECR		GENMASK_ULL(38, 32)
#define ERRXMISC0_CECO		GENMASK_ULL(46, 40)

/* ARM Cortex-A76 TRM Chapter B3.5 */
#define ERRXMISC0_UNIT		GENMASK(3, 0)
#define ERRXMISC0_LVL		GENMASK(3, 1)

/* ERRXSTATUS.SERR width depends on implementation. */
#define ERRXSTATUS_SERR_4	GENMASK(4, 0)
#define ERRXSTATUS_SERR_7	GENMASK(7, 0)
#define ERRXSTATUS_DE		BIT(23)
#define ERRXSTATUS_CE		GENMASK(25, 24)
#define ERRXSTATUS_MV		BIT(26)
#define ERRXSTATUS_UE		BIT(29)
#define ERRXSTATUS_VALID	BIT(30)

/* Affinity */
#define ERRDEVAFF_F0V		BIT(31)

#define MPIDR_AFF0		GENMASK(7, 0)
#define MPIDR_AFF1		GENMASK(15, 8)
#define MPIDR_AFF2		GENMASK(23, 16)
#define MPIDR_AFF3		GENMASK(39, 32)

#define MPIDR_AFF_HIGHER_LEVEL	0x80 /* == BIT(7) */

#define ARM_RAS_EDAC_MSG_MAX	256

static int poll_msec = 1000;
module_param(poll_msec, int, 0444);

enum {
	INDEX_L1 = 0,
	INDEX_L2,
	INDEX_L3,
	INDEX_L4,
	INDEX_L5,
	INDEX_L6,
	INDEX_L7,
};

/*
 * <shortcut> = <name>		<Arm RAS Supplement reference>
 * CE = Corrected Error		(RKFPDF)
 * DE = Deferred Error		(RXJFMG)
 * UE = Uncorrected Error	(RKJTQQ)
 */
enum {
	TYPE_CE = 0,
	TYPE_DE,
	TYPE_UE,
};

/*
 * <shortcut> = <name> 				<Arm RAS Supplement reference>
 * UC	= Uncontainable Error			(RPHLQQ)
 * UEU	= Unrecoverable Error			(RCTYHC)
 * UER	= Recoverable Error or Signaled Error	(RQTYFD) or (RCNBRY)
 * UEO	= Restartable Error or Latent Error	(RCFZTH) or (RFFTXZ)
 *
 * Related: Figure 3.2: Component error state types
 */
enum {
	UE_SUBTYPE_UC,
	UE_SUBTYPE_UEU,
	UE_SUBTYPE_UER,
	UE_SUBTYPE_UEO,
};

/* Cortex-A55 TRM */
#define ERRXMISC0_LVL		GENMASK(3, 1)
 #define ERRXMISC0_LVL_L1	0b000
 #define ERRXMISC0_LVL_L2	0b001
#define ERRXMISC0_IND		BIT(0)
 #define ERRXMISC0_IND_OTHER	0b0	/* L1_DC, L2$ or TLB */
 #define ERRXMISC0_IND_L1_IC	0b1

/* ARM DSU TRM */
 #define ERRXMISC0_LVL_L3	0b010
 #define ERRXMISC0_IND_L3	0b0

/* Cortex-A76 TRM */
#define ERRXMISC0_UNIT		GENMASK(3, 0)
 #define ERRXMISC0_UNIT_L1_IC	0b0001
 #define ERRXMISC0_UNIT_L2_TLB	0b0010
 #define ERRXMISC0_UNIT_L1_DC	0b0100
 #define ERRXMISC0_UNIT_L2	0b1000

/* Non-zero if FEAT_RAS is at least v1.0 */
#define ID_AA64PFR0_EL1_RAS	GENMASK(31, 28)

#define IS_PROCESSOR_NODE(node) (node->hdr.type == ACPI_AEST_PROCESSOR_ERROR_NODE)

/* Memory-mapped error record group (RAS Supplement 4.3.1.3) */
#define ERRnFR(n)	(0x000 + n * 0x40)
#define ERRnCTLR(n)	(0x008 + n * 0x40)
#define ERRnSTATUS(n)	(0x010 + n * 0x40)
#define ERRnADDR(n)	(0x018 + n * 0x40)
#define ERRnMISC0(n)	(0x020 + n * 0x40)
#define ERRnMISC1(n)	(0x028 + n * 0x40)
#define ERRnMISC2(n)	(0x030 + n * 0x40)
#define ERRnMISC3(n)	(0x038 + n * 0x40)
#define ERRnPFGF(n)	(0x800 + n * 0x40)
#define ERRIMPDEF(n)	(0x800 + n * 0x8)
#define ERRnPFGCTL	(0x808 + n * 0x40)
#define ERRnPFGCDN	(0x810 + n * 0x40)
#define ERRGSR 		0xE00
#define ERRIIDR		0xE10
#define ERRFHICR0	0xE80
#define ERRIRQCR(n)	(0xE80 + n * 0x8)
#define ERRFHICR1	0xE88
#define ERRFHICR2	0xE8C
#define ERRERICR0	0xE90
#define ERRERICR1	0xE98
#define ERRERICR2	0xE9C
#define ERRCRICR0	0xEA0
#define ERRCRICR1	0xEA8
#define ERRCRICR2	0xEAC
#define ERRIRQSR 	0xEF8
#define ERRDEVAFF	0xFA8
#define ERRDEVARCH	0xFBC
#define ERRDEVID	0xFC8
#define ERRPIDR4 	0xFD0
#define ERRPIDR0 	0xFE0
#define ERRPIDR1 	0xFE4
#define ERRPIDR2 	0xFE8
#define ERRPIDR3 	0xFEC
#define ERRCIDR0 	0xFF0
#define ERRCIDR1 	0xFF4
#define ERRCIDR2 	0xFF8
#define ERRCIDR3 	0xFFC

/*
 * Memory-mapped single error record (RAS Supplement 4.3.1.4) features
 * ERRnFR-ERRnMISC3 registers at the same offsets as the error record
 * group memory map with n = 0.
*/

struct error_type {
	void (*fn)(struct edac_device_ctl_info *edev_ctl,
		   int inst_nr, int block_nr, const char *msg);
	const char *msg;
};

#endif
