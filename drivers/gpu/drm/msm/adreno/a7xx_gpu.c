// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2017-2019 The Linux Foundation. All rights reserved.
 * Copyright (c) 2023, Linaro Limited
 */

#include "msm_gem.h"
#include "msm_mmu.h"
#include "msm_gpu_trace.h"
#include "a7xx_gpu.h"
#include "a7xx_gmu.xml.h"

#include <linux/bitfield.h>
#include <linux/devfreq.h>
#include <linux/reset.h>
#include <linux/soc/qcom/llcc-qcom.h>

#define GPU_PAS_ID 13

static inline bool _a7xx_check_idle(struct msm_gpu *gpu)
{
	struct adreno_gpu *adreno_gpu = to_adreno_gpu(gpu);
	struct a7xx_gpu *a7xx_gpu = to_a7xx_gpu(adreno_gpu);

	/* Check that the GMU is idle */
	if (!a7xx_gmu_isidle(&a7xx_gpu->gmu))
		return false;

	/* Check tha the CX master is idle */
	if (gpu_read(gpu, REG_A7XX_RBBM_STATUS) & ~A7XX_RBBM_STATUS_CPAHBBUSYCXMASTER)
		return false;

	return !(gpu_read(gpu, REG_A7XX_RBBM_INT_0_STATUS) &
		A7XX_RBBM_INT_0_MASK_HANGDETECTINTERRUPT);
}

static bool a7xx_idle(struct msm_gpu *gpu, struct msm_ringbuffer *ring)
{
	/* wait for CP to drain ringbuffer: */
	if (!adreno_idle(gpu, ring))
		return false;

	if (spin_until(_a7xx_check_idle(gpu))) {
		DRM_ERROR("%s: %ps: timeout waiting for GPU to idle: status %8.8X irq %8.8X rptr/wptr %d/%d\n",
			gpu->name, __builtin_return_address(0),
			gpu_read(gpu, REG_A7XX_RBBM_STATUS),
			gpu_read(gpu, REG_A7XX_RBBM_INT_0_STATUS),
			gpu_read(gpu, REG_A7XX_CP_RB_RPTR),
			gpu_read(gpu, REG_A7XX_CP_RB_WPTR));
		return false;
	}

	return true;
}

static void update_shadow_rptr(struct msm_gpu *gpu, struct msm_ringbuffer *ring)
{
	struct adreno_gpu *adreno_gpu = to_adreno_gpu(gpu);
	struct a7xx_gpu *a7xx_gpu = to_a7xx_gpu(adreno_gpu);

	/* Expanded APRIV doesn't need to issue the WHERE_AM_I opcode */
	if (a7xx_gpu->has_whereami && !adreno_gpu->base.hw_apriv) {
		OUT_PKT7(ring, CP_WHERE_AM_I, 2);
		OUT_RING(ring, lower_32_bits(shadowptr_rptr(a7xx_gpu, ring)));
		OUT_RING(ring, upper_32_bits(shadowptr_rptr(a7xx_gpu, ring)));
	}
}

static void a7xx_flush(struct msm_gpu *gpu, struct msm_ringbuffer *ring)
{
	uint32_t wptr;
	unsigned long flags;

	update_shadow_rptr(gpu, ring);

	spin_lock_irqsave(&ring->preempt_lock, flags);

	/* Copy the shadow to the actual register */
	ring->cur = ring->next;

	/* Make sure to wrap wptr if we need to */
	wptr = get_wptr(ring);

	spin_unlock_irqrestore(&ring->preempt_lock, flags);

	/* Make sure everything is posted before making a decision */
	mb();

	gpu_write(gpu, REG_A7XX_CP_RB_WPTR, wptr);
}

static void get_stats_counter(struct msm_ringbuffer *ring, u32 counter,
		u64 iova)
{
	OUT_PKT7(ring, CP_REG_TO_MEM, 3);
	OUT_RING(ring, CP_REG_TO_MEM_0_REG(counter) |
		 CP_REG_TO_MEM_0_CNT(2) |
		 CP_REG_TO_MEM_0_64B);
	OUT_RING(ring, lower_32_bits(iova));
	OUT_RING(ring, upper_32_bits(iova));
}

static void a7xx_set_pagetable(struct a7xx_gpu *a7xx_gpu,
		struct msm_ringbuffer *ring, struct msm_file_private *ctx)
{
	phys_addr_t ttbr;
	u32 asid;
	u64 memptr = rbmemptr(ring, ttbr0);

	if (ctx->seqno == a7xx_gpu->base.base.cur_ctx_seqno)
		return;

	if (msm_iommu_pagetable_params(ctx->aspace->mmu, &ttbr, &asid))
		return;

	/* Execute the table update */
	OUT_PKT7(ring, CP_SMMU_TABLE_UPDATE, 4);
	OUT_RING(ring, CP_SMMU_TABLE_UPDATE_0_TTBR0_LO(lower_32_bits(ttbr)));

	OUT_RING(ring,
		 CP_SMMU_TABLE_UPDATE_1_TTBR0_HI(upper_32_bits(ttbr)) |
		 CP_SMMU_TABLE_UPDATE_1_ASID(asid));
	OUT_RING(ring, CP_SMMU_TABLE_UPDATE_2_CONTEXTIDR(0));
	OUT_RING(ring, CP_SMMU_TABLE_UPDATE_3_CONTEXTBANK(0));

	/*
	 * Write the new TTBR0 to the memstore. This is good for debugging.
	 */
	OUT_PKT7(ring, CP_MEM_WRITE, 4);
	OUT_RING(ring, CP_MEM_WRITE_0_ADDR_LO(lower_32_bits(memptr)));
	OUT_RING(ring, CP_MEM_WRITE_1_ADDR_HI(upper_32_bits(memptr)));
	OUT_RING(ring, lower_32_bits(ttbr));
	OUT_RING(ring, (asid << 16) | upper_32_bits(ttbr));

	/*
	 * And finally, trigger a uche flush to be sure there isn't anything
	 * lingering in that part of the GPU
	 */

	OUT_PKT7(ring, CP_EVENT_WRITE, 1);
	OUT_RING(ring, CACHE_INVALIDATE);
}

static void a7xx_submit(struct msm_gpu *gpu, struct msm_gem_submit *submit)
{
	unsigned int index = submit->seqno % MSM_GPU_SUBMIT_STATS_COUNT;
	struct adreno_gpu *adreno_gpu = to_adreno_gpu(gpu);
	struct a7xx_gpu *a7xx_gpu = to_a7xx_gpu(adreno_gpu);
	struct msm_ringbuffer *ring = submit->ring;
	unsigned int i, ibs = 0;

	a7xx_set_pagetable(a7xx_gpu, ring, submit->queue->ctx);

	get_stats_counter(ring, REG_A7XX_RBBM_PERFCTR_CP(0),
		rbmemptr_stats(ring, index, cpcycles_start));

	/*
	 * For PM4 the GMU register offsets are calculated from the base of the
	 * GPU registers so we need to add 0x1a800 to the register value on A630
	 * to get the right value from PM4.
	 */
	get_stats_counter(ring, REG_A7XX_CP_ALWAYS_ON_COUNTER,
		rbmemptr_stats(ring, index, alwayson_start));

	/* Invalidate CCU depth and color */
	OUT_PKT7(ring, CP_EVENT_WRITE, 1);
	OUT_RING(ring, CP_EVENT_WRITE_0_EVENT(PC_CCU_INVALIDATE_DEPTH));

	OUT_PKT7(ring, CP_EVENT_WRITE, 1);
	OUT_RING(ring, CP_EVENT_WRITE_0_EVENT(PC_CCU_INVALIDATE_COLOR));

	/* Submit the commands */
	for (i = 0; i < submit->nr_cmds; i++) {
		switch (submit->cmd[i].type) {
		case MSM_SUBMIT_CMD_IB_TARGET_BUF:
			break;
		case MSM_SUBMIT_CMD_CTX_RESTORE_BUF:
			if (gpu->cur_ctx_seqno == submit->queue->ctx->seqno)
				break;
			fallthrough;
		case MSM_SUBMIT_CMD_BUF:
			OUT_PKT7(ring, CP_INDIRECT_BUFFER_PFE, 3);
			OUT_RING(ring, lower_32_bits(submit->cmd[i].iova));
			OUT_RING(ring, upper_32_bits(submit->cmd[i].iova));
			OUT_RING(ring, submit->cmd[i].size);
			ibs++;
			break;
		}

		/*
		 * Periodically update shadow-wptr if needed, so that we
		 * can see partial progress of submits with large # of
		 * cmds.. otherwise we could needlessly stall waiting for
		 * ringbuffer state, simply due to looking at a shadow
		 * rptr value that has not been updated
		 */
		if ((ibs % 32) == 0)
			update_shadow_rptr(gpu, ring);
	}

	get_stats_counter(ring, REG_A7XX_RBBM_PERFCTR_CP(0),
		rbmemptr_stats(ring, index, cpcycles_end));
	get_stats_counter(ring, REG_A7XX_CP_ALWAYS_ON_COUNTER,
		rbmemptr_stats(ring, index, alwayson_end));

	/* Write the fence to the scratch register */
	OUT_PKT4(ring, REG_A7XX_CP_SCRATCH_REG(2), 1);
	OUT_RING(ring, submit->seqno);

	/*
	 * Execute a CACHE_FLUSH_TS event. This will ensure that the
	 * timestamp is written to the memory and then triggers the interrupt
	 */
	OUT_PKT7(ring, CP_EVENT_WRITE, 4);
	OUT_RING(ring, CP_EVENT_WRITE_0_EVENT(CACHE_FLUSH_TS) |
		CP_EVENT_WRITE_0_IRQ);
	OUT_RING(ring, lower_32_bits(rbmemptr(ring, fence)));
	OUT_RING(ring, upper_32_bits(rbmemptr(ring, fence)));
	OUT_RING(ring, submit->seqno);

	trace_msm_gpu_submit_flush(submit,
		gpu_read64(gpu, REG_A7XX_CP_ALWAYS_ON_COUNTER));

	a7xx_flush(gpu, ring);
}

const struct adreno_reglist a730_hwcg[] = {
	{ REG_A7XX_RBBM_CLOCK_CNTL_SP0, 0x02222222 },
	{ REG_A7XX_RBBM_CLOCK_CNTL2_SP0, 0x02022222 },
	{ REG_A7XX_RBBM_CLOCK_HYST_SP0, 0x0000f3cf },
	{ REG_A7XX_RBBM_CLOCK_DELAY_SP0, 0x00000080 },
	{ REG_A7XX_RBBM_CLOCK_CNTL_TP0, 0x22222220 },
	{ REG_A7XX_RBBM_CLOCK_CNTL2_TP0, 0x22222222 },
	{ REG_A7XX_RBBM_CLOCK_CNTL3_TP0, 0x22222222 },
	{ REG_A7XX_RBBM_CLOCK_CNTL4_TP0, 0x00222222 },
	{ REG_A7XX_RBBM_CLOCK_HYST_TP0, 0x77777777 },
	{ REG_A7XX_RBBM_CLOCK_HYST2_TP0, 0x77777777 },
	{ REG_A7XX_RBBM_CLOCK_HYST3_TP0, 0x77777777 },
	{ REG_A7XX_RBBM_CLOCK_HYST4_TP0, 0x00077777 },
	{ REG_A7XX_RBBM_CLOCK_DELAY_TP0, 0x11111111 },
	{ REG_A7XX_RBBM_CLOCK_DELAY2_TP0, 0x11111111 },
	{ REG_A7XX_RBBM_CLOCK_DELAY3_TP0, 0x11111111 },
	{ REG_A7XX_RBBM_CLOCK_DELAY4_TP0, 0x00011111 },
	{ REG_A7XX_RBBM_CLOCK_CNTL_UCHE, 0x22222222 },
	{ REG_A7XX_RBBM_CLOCK_HYST_UCHE, 0x00000004 },
	{ REG_A7XX_RBBM_CLOCK_DELAY_UCHE, 0x00000002 },
	{ REG_A7XX_RBBM_CLOCK_CNTL_RB0, 0x22222222 },
	{ REG_A7XX_RBBM_CLOCK_CNTL2_RB0, 0x01002222 },
	{ REG_A7XX_RBBM_CLOCK_CNTL_CCU0, 0x00002220 },
	{ REG_A7XX_RBBM_CLOCK_HYST_RB_CCU0, 0x44000f00 },
	{ REG_A7XX_RBBM_CLOCK_CNTL_RAC, 0x25222022 },
	{ REG_A7XX_RBBM_CLOCK_CNTL2_RAC, 0x00555555 },
	{ REG_A7XX_RBBM_CLOCK_DELAY_RAC, 0x00000011 },
	{ REG_A7XX_RBBM_CLOCK_HYST_RAC, 0x00440044 },
	{ REG_A7XX_RBBM_CLOCK_CNTL_TSE_RAS_RBBM, 0x04222222 },
	{ REG_A7XX_RBBM_CLOCK_MODE2_GRAS, 0x00000222 },
	{ REG_A7XX_RBBM_CLOCK_MODE_BV_GRAS, 0x00222222 },
	{ REG_A7XX_RBBM_CLOCK_MODE_GPC, 0x02222223 },
	{ REG_A7XX_RBBM_CLOCK_MODE_VFD, 0x00002222 },
	{ REG_A7XX_RBBM_CLOCK_MODE_BV_GPC, 0x00222222 },
	{ REG_A7XX_RBBM_CLOCK_MODE_BV_VFD, 0x00002222 },
	{ REG_A7XX_RBBM_CLOCK_HYST_TSE_RAS_RBBM, 0x00000000 },
	{ REG_A7XX_RBBM_CLOCK_HYST_GPC, 0x04104004 },
	{ REG_A7XX_RBBM_CLOCK_HYST_VFD, 0x00000000 },
	{ REG_A7XX_RBBM_CLOCK_DELAY_TSE_RAS_RBBM, 0x00004000 },
	{ REG_A7XX_RBBM_CLOCK_DELAY_GPC, 0x00000200 },
	{ REG_A7XX_RBBM_CLOCK_DELAY_VFD, 0x00002222 },
	{ REG_A7XX_RBBM_CLOCK_MODE_HLSQ, 0x00002222 },
	{ REG_A7XX_RBBM_CLOCK_DELAY_HLSQ, 0x00000000 },
	{ REG_A7XX_RBBM_CLOCK_HYST_HLSQ, 0x00000000 },
	{ REG_A7XX_RBBM_CLOCK_DELAY_HLSQ_2, 0x00000002 },
	{ REG_A7XX_RBBM_CLOCK_MODE_BV_LRZ, 0x55555552 },
	{ REG_A7XX_RBBM_CLOCK_MODE_CP, 0x00000223 },
	{ REG_A7XX_RBBM_CLOCK_CNTL, 0x8aa8aa82 },
	{ REG_A7XX_RBBM_ISDB_CNT, 0x00000182 },
	{ REG_A7XX_RBBM_RAC_THRESHOLD_CNT, 0x00000000 },
	{ REG_A7XX_RBBM_SP_HYST_CNT, 0x00000000 },
	{ REG_A7XX_RBBM_CLOCK_CNTL_GMU_GX, 0x00000222 },
	{ REG_A7XX_RBBM_CLOCK_DELAY_GMU_GX, 0x00000111 },
	{ REG_A7XX_RBBM_CLOCK_HYST_GMU_GX, 0x00000555 },
	{},
};

#define GMU_AO_CGC_MODE_CNTL 0x00020000
#define GMU_AO_CGC_DELAY_CNTL 0x00010111
#define GMU_AO_CGC_HYST_CNTL 0x00005555
static void a7xx_set_hwcg(struct msm_gpu *gpu, bool state)
{
	struct adreno_gpu *adreno_gpu = to_adreno_gpu(gpu);
	struct a7xx_gpu *a7xx_gpu = to_a7xx_gpu(adreno_gpu);
	const struct adreno_reglist *reg;
	unsigned int i;
	u32 val, clock_cntl_on;

	if (!adreno_gpu->info->hwcg)
		return;

	clock_cntl_on = 0x8aa8aa82;

	gmu_write(&a7xx_gpu->gmu, REG_A7XX_GPU_GMU_AO_GMU_CGC_MODE_CNTL,
				  state ? GMU_AO_CGC_MODE_CNTL : 0);
	gmu_write(&a7xx_gpu->gmu, REG_A7XX_GPU_GMU_AO_GMU_CGC_DELAY_CNTL,
				  state ? GMU_AO_CGC_DELAY_CNTL : 0);
	gmu_write(&a7xx_gpu->gmu, REG_A7XX_GPU_GMU_AO_GMU_CGC_HYST_CNTL,
				  state ? GMU_AO_CGC_HYST_CNTL : 0);

	val = gpu_read(gpu, REG_A7XX_RBBM_CLOCK_CNTL);

	/* Don't re-program the registers if they are already correct */
	if ((!state && !val) || (state && (val == clock_cntl_on)))
		return;

	for (i = 0; (reg = &adreno_gpu->info->hwcg[i], reg->offset); i++)
		gpu_write(gpu, reg->offset, state ? reg->value : 0);

	gpu_write(gpu, REG_A7XX_RBBM_CLOCK_CNTL, state ? clock_cntl_on : 0);
}

static const u32 a7xx_protect[] = {
	A7XX_PROTECT_RDONLY(0x00000, 0x004ff),
	A7XX_PROTECT_RDONLY(0x0050b, 0x00563),
	A7XX_PROTECT_NORDWR(0x0050e, 0x0050e),
	A7XX_PROTECT_NORDWR(0x00510, 0x00510),
	A7XX_PROTECT_NORDWR(0x00534, 0x00534),
	A7XX_PROTECT_RDONLY(0x005fb, 0x00698),
	A7XX_PROTECT_NORDWR(0x00699, 0x00882),
	A7XX_PROTECT_NORDWR(0x008a0, 0x008a8),
	A7XX_PROTECT_NORDWR(0x008ab, 0x008cf),
	A7XX_PROTECT_RDONLY(0x008d0, 0x00a40),
	A7XX_PROTECT_NORDWR(0x00900, 0x0094d),
	A7XX_PROTECT_NORDWR(0x0098d, 0x00a3f),
	A7XX_PROTECT_NORDWR(0x00a41, 0x00bff),
	A7XX_PROTECT_NORDWR(0x00df0, 0x00df1),
	A7XX_PROTECT_NORDWR(0x00e01, 0x00e01),
	A7XX_PROTECT_NORDWR(0x00e07, 0x00e0f),
	A7XX_PROTECT_NORDWR(0x03c00, 0x03cc3),
	A7XX_PROTECT_RDONLY(0x03cc4, 0x05cc3),
	A7XX_PROTECT_NORDWR(0x08630, 0x087ff),
	A7XX_PROTECT_NORDWR(0x08e00, 0x08e00),
	A7XX_PROTECT_NORDWR(0x08e08, 0x08e08),
	A7XX_PROTECT_NORDWR(0x08e50, 0x08e6f),
	A7XX_PROTECT_NORDWR(0x08e80, 0x09100),
	A7XX_PROTECT_NORDWR(0x09624, 0x097ff),
	A7XX_PROTECT_NORDWR(0x09e40, 0x09e40),
	A7XX_PROTECT_NORDWR(0x09e64, 0x09e71),
	A7XX_PROTECT_NORDWR(0x09e78, 0x09fff),
	A7XX_PROTECT_NORDWR(0x0a630, 0x0a7ff),
	A7XX_PROTECT_NORDWR(0x0ae02, 0x0ae02),
	A7XX_PROTECT_NORDWR(0x0ae50, 0x0ae5f),
	A7XX_PROTECT_NORDWR(0x0ae66, 0x0ae69),
	A7XX_PROTECT_NORDWR(0x0ae6f, 0x0ae72),
	A7XX_PROTECT_NORDWR(0x0b604, 0x0b607),
	A7XX_PROTECT_NORDWR(0x0ec00, 0x0fbff),
	A7XX_PROTECT_RDONLY(0x0fc00, 0x11bff),
	A7XX_PROTECT_NORDWR(0x18400, 0x18453),
	A7XX_PROTECT_RDONLY(0x18454, 0x18458),
	A7XX_PROTECT_NORDWR(0x18459, 0x1a458),
	A7XX_PROTECT_NORDWR(0x1a459, 0x1c458),
	A7XX_PROTECT_NORDWR(0x1c459, 0x1e458),
	A7XX_PROTECT_NORDWR(0x1f400, 0x1f843),
	A7XX_PROTECT_RDONLY(0x1f844, 0x1f8bf),
	A7XX_PROTECT_NORDWR(0x1f860, 0x1f860),
	A7XX_PROTECT_NORDWR(0x1f878, 0x1f8a2),
	/* CP_PROTECT_REG[44, 46] are not set */
	0,
	0,
	0,
	A7XX_PROTECT_NORDWR(0x1f8c0, 0x00000),
};

static void a7xx_set_cp_protect(struct msm_gpu *gpu)
{
	const u32 *regs = a7xx_protect;
	unsigned i, count, count_max;

	/* We expect different sets of data for newer chips! */
	regs = a7xx_protect;
	count = ARRAY_SIZE(a7xx_protect);
	count_max = 48;
	BUILD_BUG_ON(ARRAY_SIZE(a7xx_protect) > 48);

	/*
	 * BIT(0) - Enable access protection to privileged registers
	 * BIT(1) - Enable fault on an access protect violation
	 * BIT(3) - Select the last span to protect from the start
	 * 	    address all the way to the end of the register address space
	 */
	gpu_write(gpu, REG_A7XX_CP_PROTECT_CNTL, BIT(0) | BIT(1) | BIT(3));

	for (i = 0; i < count - 1; i++) {
		if (regs[i])
			gpu_write(gpu, REG_A7XX_CP_PROTECT(i), regs[i]);
	}

	gpu_write(gpu, REG_A7XX_CP_PROTECT(count_max - 1), regs[i]);
}

static int a7xx_cp_init(struct msm_gpu *gpu)
{
	struct msm_ringbuffer *ring = gpu->rb[0];
	u32 mask;

	/* Disable concurrent binning before sending CP init */
	OUT_PKT7(ring, CP_THREAD_CONTROL, 1);
	OUT_RING(ring, BIT(27));

	OUT_PKT7(ring, CP_ME_INIT, 7);

	/* Use multiple HW contexts */
	mask = BIT(0);

	/* Enable error detection */
	mask |= BIT(1);

	/* Set default reset state */
	mask |= BIT(3);

	/* Disable save/restore of performance counters across preemption */
	mask |= BIT(6);

	/* Enable the register init list with the spinlock */
	mask |= BIT(8);

	OUT_RING(ring, mask);

	/* Enable multiple hardware contexts */
	OUT_RING(ring, 0x00000003);

	/* Enable error detection */
	OUT_RING(ring, 0x20000000);

	/* Operation mode mask */
	OUT_RING(ring, 0x00000002);

	/* *Don't* send a power up reg list for concurrent binning (TODO) */
	/* Lo address */
	OUT_RING(ring, 0x00000000);
	/* Hi address */
	OUT_RING(ring, 0x00000000);
	/* BIT(31) set => read the regs from the list */
	OUT_RING(ring, 0x00000000);

	a7xx_flush(gpu, ring);
	return a7xx_idle(gpu, ring) ? 0 : -EINVAL;
}

static int a7xx_ucode_init(struct msm_gpu *gpu)
{
	struct adreno_gpu *adreno_gpu = to_adreno_gpu(gpu);
	struct a7xx_gpu *a7xx_gpu = to_a7xx_gpu(adreno_gpu);

	if (!a7xx_gpu->sqe_bo) {
		a7xx_gpu->sqe_bo = adreno_fw_create_bo(gpu,
			adreno_gpu->fw[ADRENO_FW_SQE], &a7xx_gpu->sqe_iova);

		if (IS_ERR(a7xx_gpu->sqe_bo)) {
			int ret = PTR_ERR(a7xx_gpu->sqe_bo);

			a7xx_gpu->sqe_bo = NULL;
			DRM_DEV_ERROR(&gpu->pdev->dev,
				"Could not allocate SQE ucode: %d\n", ret);

			return ret;
		}

		msm_gem_object_set_name(a7xx_gpu->sqe_bo, "sqefw");
	}

	gpu_write64(gpu, REG_A7XX_CP_SQE_INSTR_BASE, a7xx_gpu->sqe_iova);

	return 0;
}

static int a7xx_zap_shader_init(struct msm_gpu *gpu)
{
	static bool loaded;
	int ret;

	if (loaded)
		return 0;

	ret = adreno_zap_shader_load(gpu, GPU_PAS_ID);

	loaded = !ret;
	return ret;
}

#define A7XX_INT_MASK (A7XX_RBBM_INT_0_MASK_AHBERROR | \
		       A7XX_RBBM_INT_0_MASK_ATBASYNCFIFOOVERFLOW | \
		       A7XX_RBBM_INT_0_MASK_GPCERROR | \
		       A7XX_RBBM_INT_0_MASK_SWINTERRUPT | \
		       A7XX_RBBM_INT_0_MASK_HWERROR | \
		       A7XX_RBBM_INT_0_MASK_PM4CPINTERRUPT | \
		       A7XX_RBBM_INT_0_MASK_RB_DONE_TS | \
		       A7XX_RBBM_INT_0_MASK_CACHE_CLEAN_TS | \
		       A7XX_RBBM_INT_0_MASK_ATBBUSOVERFLOW | \
		       A7XX_RBBM_INT_0_MASK_HANGDETECTINTERRUPT | \
		       A7XX_RBBM_INT_0_MASK_OUTOFBOUNDACCESS | \
		       A7XX_RBBM_INT_0_MASK_UCHETRAPINTERRUPT | \
		       A7XX_RBBM_INT_0_MASK_TSBWRITEERROR)

#define A7XX_APRIV_MASK (A7XX_CP_APRIV_CNTL_ICACHE | \
			A7XX_CP_APRIV_CNTL_RBFETCH | \
			A7XX_CP_APRIV_CNTL_RBPRIVLEVEL | \
			A7XX_CP_APRIV_CNTL_RBRPWB)

#define A7XX_BR_APRIVMASK (A7XX_APRIV_MASK | \
			   A7XX_CP_APRIV_CNTL_CDREAD | \
			   A7XX_CP_APRIV_CNTL_CDWRITE)

static int hw_init(struct msm_gpu *gpu)
{
	struct adreno_gpu *adreno_gpu = to_adreno_gpu(gpu);
	struct a7xx_gpu *a7xx_gpu = to_a7xx_gpu(adreno_gpu);
	int ret;

	/* Make sure the GMU keeps the GPU on while we set it up */
	a7xx_gmu_set_oob(&a7xx_gpu->gmu, GMU_OOB_GPU_SET);

	gpu_write(gpu, REG_A7XX_GBIF_HALT, 0);
	gpu_write(gpu, REG_A7XX_RBBM_GBIF_HALT, 0);
	/* Make extra sure GBIF is not halted before we power on the GMU */
	mb();

	/* VBIF/GBIF start*/
	gpu_write(gpu, REG_A7XX_GBIF_QSB_SIDE0, 0x00071620);
	gpu_write(gpu, REG_A7XX_GBIF_QSB_SIDE1, 0x00071620);
	gpu_write(gpu, REG_A7XX_GBIF_QSB_SIDE2, 0x00071620);
	gpu_write(gpu, REG_A7XX_GBIF_QSB_SIDE3, 0x00071620);
	gpu_write(gpu, REG_A7XX_RBBM_GBIF_CLIENT_QOS_CNTL, 0x2120212);

	gpu_write(gpu, REG_A7XX_UCHE_GBIF_GX_CONFIG, 0x10240e0);

	/* Make all blocks contribute to the GPU BUSY perf counter */
	gpu_write(gpu, REG_A7XX_RBBM_PERFCTR_GPU_BUSY_MASKED, 0xffffffff);

	/* Disable L2 bypass in the UCHE */
	gpu_write64(gpu, REG_A7XX_UCHE_TRAP_BASE, 0x0001fffffffff000ULL);
	gpu_write64(gpu, REG_A7XX_UCHE_WRITE_THRU_BASE, 0x0001fffffffff000ULL);

	gpu_write(gpu, REG_A7XX_UCHE_CACHE_WAYS, BIT(23));

	gpu_write(gpu, REG_A7XX_UCHE_CMDQ_CONFIG,
		       FIELD_PREP(GENMASK(19, 16), 6) |
		       FIELD_PREP(GENMASK(15, 12), 6) |
		       FIELD_PREP(GENMASK(11, 8), 9) |
		       BIT(3) | BIT(2) |
		       FIELD_PREP(GENMASK(1, 0), 2));

	/* Set the AHB default slave response to "ERROR" */
	gpu_write(gpu, REG_A7XX_CP_AHB_CNTL, 0x1);

	/* Turn on performance counters */
	gpu_write(gpu, REG_A7XX_RBBM_PERFCTR_CNTL, 0x1);

	/* Turn on the IFPC counter (countable 4 on XOCLK4) */
	gmu_write(&a7xx_gpu->gmu, REG_A7XX_GMU_CX_GMU_POWER_COUNTER_SELECT_1,
		       FIELD_PREP(GENMASK(7, 0), 0x4));

	gpu_write(gpu, REG_A7XX_RB_NC_MODE_CNTL,
		       BIT(11) | BIT(4) |
		       FIELD_PREP(GENMASK(2, 1), 3));

	gpu_write(gpu, REG_A7XX_TPL1_NC_MODE_CNTL,
		       FIELD_PREP(GENMASK(2, 1), 3));

	gpu_write(gpu, REG_A7XX_SP_NC_MODE_CNTL,
		       FIELD_PREP(GENMASK(5, 4), 2) |
		       FIELD_PREP(GENMASK(2, 1), 3));

	gpu_write(gpu, REG_A7XX_GRAS_NC_MODE_CNTL,
		       FIELD_PREP(GENMASK(8, 5), 3));

	gpu_write(gpu, REG_A7XX_UCHE_MODE_CNTL,
		       FIELD_PREP(GENMASK(22, 21), 3));

	gpu_write(gpu, REG_A7XX_RBBM_INTERFACE_HANG_INT_CNTL,
		       BIT(30) |
		       FIELD_PREP(GENMASK(27, 0), 0xcfffff));

	gpu_write(gpu, REG_A7XX_UCHE_CLIENT_PF, BIT(0));

	/* Protect registers from the CP */
	a7xx_set_cp_protect(gpu);

	/* Enable expanded apriv for targets that support it */
	if (gpu->hw_apriv) {
		gpu_write(gpu, REG_A7XX_CP_APRIV_CNTL, A7XX_BR_APRIVMASK);
		gpu_write(gpu, REG_A7XX_CP_BV_APRIV_CNTL, A7XX_APRIV_MASK);
		gpu_write(gpu, REG_A7XX_CP_LPAC_APRIV_CNTL, A7XX_APRIV_MASK);
	}

	/* Set up SECVID */
	gpu_write(gpu, REG_A7XX_RBBM_SECVID_TSB_CNTL, 0);

	/*
	 * Disable the trusted memory range - we don't actually supported secure
	 * memory rendering at this point in time and we don't want to block off
	 * part of the virtual memory space.
	 */
	gpu_write64(gpu, REG_A7XX_RBBM_SECVID_TSB_TRUSTED_BASE, 0);
	gpu_write(gpu, REG_A7XX_RBBM_SECVID_TSB_TRUSTED_SIZE, 0);

	a7xx_set_hwcg(gpu, true);

	/* Select CP0 to always count cycles */
	gpu_write(gpu, REG_A7XX_CP_PERFCTR_CP_SEL(0), PERF_CP_ALWAYS_COUNT);

	/* Enable interrupts */
	gpu_write(gpu, REG_A7XX_RBBM_INT_0_MASK, A7XX_INT_MASK);

	ret = adreno_hw_init(gpu);
	if (ret)
		goto out;

	/* Always come up on rb 0 */
	a7xx_gpu->cur_ring = gpu->rb[0];
	gpu->cur_ctx_seqno = 0;

	/*
	 * Expanded APRIV and targets that support WHERE_AM_I both need a
	 * privileged buffer to store the RPTR shadow
	 */
	if (adreno_gpu->base.hw_apriv || a7xx_gpu->has_whereami) {
		if (!a7xx_gpu->shadow_bo) {
			a7xx_gpu->shadow = msm_gem_kernel_new(gpu->dev,
				2 * sizeof(u32) * gpu->nr_rings,
				MSM_BO_WC | MSM_BO_MAP_PRIV,
				gpu->aspace, &a7xx_gpu->shadow_bo,
				&a7xx_gpu->shadow_iova);

			if (IS_ERR(a7xx_gpu->shadow))
				return PTR_ERR(a7xx_gpu->shadow);

			msm_gem_object_set_name(a7xx_gpu->shadow_bo, "shadow");
		}

		gpu_write64(gpu, REG_A7XX_CP_RB_RPTR_ADDR, shadowptr_rptr(a7xx_gpu, gpu->rb[0]));
		gpu_write64(gpu, REG_A7XX_CP_BV_RB_RPTR_ADDR, shadowptr_bv_rptr(a7xx_gpu, gpu->rb[0]));
	}

	/* TODO: confirm */
	gpu_write(gpu, REG_A7XX_CP_RB_CNTL, 0x20c);

	/* Set the ringbuffer address */
	gpu_write64(gpu, REG_A7XX_CP_RB_BASE, gpu->rb[0]->iova);

	ret = a7xx_ucode_init(gpu);
	if (ret)
		goto out;

	/* Enable the SQE_to start the CP engine */
	gpu_write(gpu, REG_A7XX_CP_SQE_CNTL, 1);

	ret = a7xx_cp_init(gpu);
	if (ret)
		goto out;

	/*
	 * Try to load a zap shader into the secure world. If successful
	 * we can use the CP to switch out of secure mode. If not then we
	 * have no resource but to try to switch ourselves out manually. If we
	 * guessed wrong then access to the RBBM_SECVID_TRUST_CNTL register will
	 * be blocked and a permissions violation will soon follow.
	 */
	ret = a7xx_zap_shader_init(gpu);
	if (!ret) {
		OUT_PKT7(gpu->rb[0], CP_SET_SECURE_MODE, 1);
		OUT_RING(gpu->rb[0], 0x00000000);

		a7xx_flush(gpu, gpu->rb[0]);
		if (!a7xx_idle(gpu, gpu->rb[0]))
			return -EINVAL;
	} else if (ret == -ENODEV) {
		/*
		 * This device does not use zap shader (but print a warning
		 * just in case someone got their dt wrong.. hopefully they
		 * have a debug UART to realize the error of their ways...
		 * if you mess this up you are about to crash horribly)
		 */
		dev_warn_once(gpu->dev->dev,
			"Zap shader not enabled - using SECVID_TRUST_CNTL instead\n");
		gpu_write(gpu, REG_A7XX_RBBM_SECVID_TRUST_CNTL, 0x0);
		ret = 0;
	} else {
		return ret;
	}

out:
	/*
	 * Tell the GMU that we are done touching the GPU and it can start power
	 * management
	 */
	a7xx_gmu_clear_oob(&a7xx_gpu->gmu, GMU_OOB_GPU_SET);

	return ret;
}

static int a7xx_hw_init(struct msm_gpu *gpu)
{
	struct adreno_gpu *adreno_gpu = to_adreno_gpu(gpu);
	struct a7xx_gpu *a7xx_gpu = to_a7xx_gpu(adreno_gpu);
	int ret;

	mutex_lock(&a7xx_gpu->gmu.lock);
	ret = hw_init(gpu);
	mutex_unlock(&a7xx_gpu->gmu.lock);

	return ret;
}

static void a7xx_dump(struct msm_gpu *gpu)
{
	DRM_DEV_INFO(&gpu->pdev->dev, "status: %08x\n", gpu_read(gpu, REG_A7XX_RBBM_STATUS));
	adreno_dump(gpu);
}

static void a7xx_recover(struct msm_gpu *gpu)
{
	struct adreno_gpu *adreno_gpu = to_adreno_gpu(gpu);
	struct a7xx_gpu *a7xx_gpu = to_a7xx_gpu(adreno_gpu);
	int i, active_submits;

	adreno_dump_info(gpu);

	for (i = 0; i < 8; i++)
		DRM_DEV_INFO(&gpu->pdev->dev, "CP_SCRATCH_REG%d: %u\n", i,
			gpu_read(gpu, REG_A7XX_CP_SCRATCH_REG(i)));

	if (hang_debug)
		a7xx_dump(gpu);

	/*
	 * To handle recovery specific sequences during the rpm suspend we are
	 * about to trigger
	 */
	a7xx_gpu->hung = true;

	/* Halt SQE first */
	gpu_write(gpu, REG_A7XX_CP_SQE_CNTL, 3);

	/*
	 * Turn off keep alive that might have been enabled by the hang
	 * interrupt
	 */
	gmu_write(&a7xx_gpu->gmu, REG_A7XX_GMU_GMU_PWR_COL_KEEPALIVE, 0);

	pm_runtime_dont_use_autosuspend(&gpu->pdev->dev);

	/* active_submit won't change until we make a submission */
	mutex_lock(&gpu->active_lock);
	active_submits = gpu->active_submits;

	/*
	 * Temporarily clear active_submits count to silence a WARN() in the
	 * runtime suspend cb
	 */
	gpu->active_submits = 0;

	/* Drop the rpm refcount from active submits */
	if (active_submits)
		pm_runtime_put(&gpu->pdev->dev);

	/* And the final one from recover worker */
	pm_runtime_put_sync(&gpu->pdev->dev);

	/* Call into gpucc driver to poll for cx gdsc collapse */
	reset_control_reset(gpu->cx_collapse);

	pm_runtime_use_autosuspend(&gpu->pdev->dev);

	if (active_submits)
		pm_runtime_get(&gpu->pdev->dev);

	pm_runtime_get_sync(&gpu->pdev->dev);

	gpu->active_submits = active_submits;
	mutex_unlock(&gpu->active_lock);

	msm_gpu_hw_init(gpu);
	a7xx_gpu->hung = false;
}

static const char *a7xx_uche_fault_block(struct msm_gpu *gpu, u32 mid)
{
	static const char *uche_clients[7] = {
		"VFD", "SP", "VSC", "VPC", "HLSQ", "PC", "LRZ",
	};
	u32 val;

	if (mid < 1 || mid > 3)
		return "UNKNOWN";

	/*
	 * The source of the data depends on the mid ID read from FSYNR1.
	 * and the client ID read from the UCHE block
	 */
	val = gpu_read(gpu, REG_A7XX_UCHE_CLIENT_PF);

	/* mid = 3 is most precise and refers to only one block per client */
	if (mid == 3)
		return uche_clients[val & 7];

	/* For mid=2 the source is TP or VFD except when the client id is 0 */
	if (mid == 2)
		return ((val & 7) == 0) ? "TP" : "TP|VFD";

	/* For mid=1 just return "UCHE" as a catchall for everything else */
	return "UCHE";
}

static const char *a7xx_fault_block(struct msm_gpu *gpu, u32 id)
{
	if (id == 0)
		return "CP";
	else if (id == 4)
		return "CCU";
	else if (id == 6)
		return "CDP Prefetch";

	return a7xx_uche_fault_block(gpu, id);
}

#define ARM_SMMU_FSR_TF                 BIT(1)
#define ARM_SMMU_FSR_PF			BIT(3)
#define ARM_SMMU_FSR_EF			BIT(4)

static int a7xx_fault_handler(void *arg, unsigned long iova, int flags, void *data)
{
	struct msm_gpu *gpu = arg;
	struct adreno_smmu_fault_info *info = data;
	const char *type = "UNKNOWN";
	const char *block;
	bool do_devcoredump = info && !READ_ONCE(gpu->crashstate);

	/*
	 * If we aren't going to be resuming later from fault_worker, then do
	 * it now.
	 */
	if (!do_devcoredump) {
		gpu->aspace->mmu->funcs->resume_translation(gpu->aspace->mmu);
	}

	/*
	 * Print a default message if we couldn't get the data from the
	 * adreno-smmu-priv
	 */
	if (!info) {
		pr_warn_ratelimited("*** gpu fault: iova=%.16lx flags=%d (%u,%u,%u,%u)\n",
			iova, flags,
			gpu_read(gpu, REG_A7XX_CP_SCRATCH_REG(4)),
			gpu_read(gpu, REG_A7XX_CP_SCRATCH_REG(5)),
			gpu_read(gpu, REG_A7XX_CP_SCRATCH_REG(6)),
			gpu_read(gpu, REG_A7XX_CP_SCRATCH_REG(7)));

		return 0;
	}

	if (info->fsr & ARM_SMMU_FSR_TF)
		type = "TRANSLATION";
	else if (info->fsr & ARM_SMMU_FSR_PF)
		type = "PERMISSION";
	else if (info->fsr & ARM_SMMU_FSR_EF)
		type = "EXTERNAL";

	block = a7xx_fault_block(gpu, info->fsynr1 & 0xff);

	pr_warn_ratelimited("*** gpu fault: ttbr0=%.16llx iova=%.16lx dir=%s type=%s source=%s (%u,%u,%u,%u)\n",
			info->ttbr0, iova,
			flags & IOMMU_FAULT_WRITE ? "WRITE" : "READ",
			type, block,
			gpu_read(gpu, REG_A7XX_CP_SCRATCH_REG(4)),
			gpu_read(gpu, REG_A7XX_CP_SCRATCH_REG(5)),
			gpu_read(gpu, REG_A7XX_CP_SCRATCH_REG(6)),
			gpu_read(gpu, REG_A7XX_CP_SCRATCH_REG(7)));

	if (do_devcoredump) {
		/* Turn off the hangcheck timer to keep it from bothering us */
		del_timer(&gpu->hangcheck_timer);

		gpu->fault_info.ttbr0 = info->ttbr0;
		gpu->fault_info.iova  = iova;
		gpu->fault_info.flags = flags;
		gpu->fault_info.type  = type;
		gpu->fault_info.block = block;

		kthread_queue_work(gpu->worker, &gpu->fault_work);
	}

	return 0;
}

static void a7xx_cp_hw_err_irq(struct msm_gpu *gpu)
{
	u32 status = gpu_read(gpu, REG_A7XX_CP_INTERRUPT_STATUS);

	if (status & A7XX_CP_INTERRUPT_STATUS_OPCODEERROR) {
		u32 val;

		gpu_write(gpu, REG_A7XX_CP_SQE_STAT_ADDR, 1);
		val = gpu_read(gpu, REG_A7XX_CP_SQE_STAT_DATA);
		dev_err_ratelimited(&gpu->pdev->dev,
			"CP | opcode error | possible opcode=0x%8.8X\n",
			val);
	}

	if (status & A7XX_CP_INTERRUPT_STATUS_UCODEERROR)
		dev_err_ratelimited(&gpu->pdev->dev,
			"CP ucode error interrupt\n");

	if (status & A7XX_CP_INTERRUPT_STATUS_CPHWFAULT)
		dev_err_ratelimited(&gpu->pdev->dev, "CP | HW fault | status=0x%8.8X\n",
			gpu_read(gpu, REG_A7XX_CP_HW_FAULT));

	if (status & A7XX_CP_INTERRUPT_STATUS_REGISTERPROTECTION) {
		u32 val = gpu_read(gpu, REG_A7XX_CP_PROTECT_STATUS);

		dev_err_ratelimited(&gpu->pdev->dev,
			"CP | protected mode error | %s | addr=0x%8.8X | status=0x%8.8X\n",
			val & (1 << 20) ? "READ" : "WRITE",
			(val & 0x3ffff), val);
	}

	if (status & A7XX_CP_INTERRUPT_STATUS_VSDPARITYERROR)
		dev_err_ratelimited(&gpu->pdev->dev, "CP VSD decoder parity error\n");

	if (status & A7XX_CP_INTERRUPT_STATUS_ILLEGALINSTRUCTION)
		dev_err_ratelimited(&gpu->pdev->dev, "CP illegal instruction error\n");

}

static void a7xx_fault_detect_irq(struct msm_gpu *gpu)
{
	struct adreno_gpu *adreno_gpu = to_adreno_gpu(gpu);
	struct a7xx_gpu *a7xx_gpu = to_a7xx_gpu(adreno_gpu);
	struct msm_ringbuffer *ring = gpu->funcs->active_ring(gpu);

	/*
	 * If stalled on SMMU fault, we could trip the GPU's hang detection,
	 * but the fault handler will trigger the devcore dump, and we want
	 * to otherwise resume normally rather than killing the submit, so
	 * just bail.
	 */
	if (gpu_read(gpu, REG_A7XX_RBBM_STATUS3) & A7XX_RBBM_STATUS3_SMMU_STALLED_ON_FAULT)
		return;

	/*
	 * Force the GPU to stay on until after we finish
	 * collecting information
	 */
	gmu_write(&a7xx_gpu->gmu, REG_A7XX_GMU_GMU_PWR_COL_KEEPALIVE, 1);

	DRM_DEV_ERROR(&gpu->pdev->dev,
		"gpu fault ring %d fence %x status %8.8X rb %4.4x/%4.4x ib1 %16.16llX/%4.4x ib2 %16.16llX/%4.4x\n",
		ring ? ring->id : -1, ring ? ring->fctx->last_fence : 0,
		gpu_read(gpu, REG_A7XX_RBBM_STATUS),
		gpu_read(gpu, REG_A7XX_CP_RB_RPTR),
		gpu_read(gpu, REG_A7XX_CP_RB_WPTR),
		gpu_read64(gpu, REG_A7XX_CP_IB1_BASE),
		gpu_read(gpu, REG_A7XX_CP_IB1_REM_SIZE),
		gpu_read64(gpu, REG_A7XX_CP_IB2_BASE),
		gpu_read(gpu, REG_A7XX_CP_IB2_REM_SIZE));

	/* Turn off the hangcheck timer to keep it from bothering us */
	del_timer(&gpu->hangcheck_timer);

	kthread_queue_work(gpu->worker, &gpu->recover_work);
}

static irqreturn_t a7xx_irq(struct msm_gpu *gpu)
{
	struct msm_drm_private *priv = gpu->dev->dev_private;
	u32 status = gpu_read(gpu, REG_A7XX_RBBM_INT_0_STATUS);

	gpu_write(gpu, REG_A7XX_RBBM_INT_CLEAR_CMD, status);

	if (priv->disable_err_irq)
		status &= A7XX_RBBM_INT_0_MASK_CACHE_CLEAN_TS;

	if (status & A7XX_RBBM_INT_0_MASK_HANGDETECTINTERRUPT)
		a7xx_fault_detect_irq(gpu);

	if (status & A7XX_RBBM_INT_0_MASK_AHBERROR)
		dev_err_ratelimited(&gpu->pdev->dev, "CP | AHB bus error\n");

	if (status & A7XX_RBBM_INT_0_MASK_HWERROR)
		a7xx_cp_hw_err_irq(gpu);

	if (status & A7XX_RBBM_INT_0_MASK_ATBASYNCFIFOOVERFLOW)
		dev_err_ratelimited(&gpu->pdev->dev, "RBBM | ATB ASYNC overflow\n");

	if (status & A7XX_RBBM_INT_0_MASK_ATBBUSOVERFLOW)
		dev_err_ratelimited(&gpu->pdev->dev, "RBBM | ATB bus overflow\n");

	if (status & A7XX_RBBM_INT_0_MASK_OUTOFBOUNDACCESS)
		dev_err_ratelimited(&gpu->pdev->dev, "UCHE | Out of bounds access\n");

	if (status & A7XX_RBBM_INT_0_MASK_CACHE_CLEAN_TS)
		msm_gpu_retire(gpu);

	return IRQ_HANDLED;
}

static void a7xx_llc_deactivate(struct a7xx_gpu *a7xx_gpu)
{
	llcc_slice_deactivate(a7xx_gpu->llc_slice);
	llcc_slice_deactivate(a7xx_gpu->htw_llc_slice);
}

static void a7xx_llc_activate(struct a7xx_gpu *a7xx_gpu)
{
	struct adreno_gpu *adreno_gpu = &a7xx_gpu->base;
	struct msm_gpu *gpu = &adreno_gpu->base;
	u32 gpu_scid;

	if (IS_ERR(a7xx_gpu->llc_mmio))
		return;

	if (!llcc_slice_activate(a7xx_gpu->llc_slice)) {
		gpu_scid = llcc_get_slice_id(a7xx_gpu->llc_slice);

		/* 6 blocks at 5 bits per block */
		gpu_write(gpu, REG_A7XX_GBIF_SCACHE_CNTL1,
			  FIELD_PREP(GENMASK(29, 25), gpu_scid) |
			  FIELD_PREP(GENMASK(24, 20), gpu_scid) |
			  FIELD_PREP(GENMASK(19, 15), gpu_scid) |
			  FIELD_PREP(GENMASK(14, 10), gpu_scid) |
			  FIELD_PREP(GENMASK(9, 5), gpu_scid) |
			  FIELD_PREP(GENMASK(4, 0), gpu_scid));
	}

	if (!llcc_slice_activate(a7xx_gpu->htw_llc_slice))
		return;

	gpu_write(gpu, REG_A7XX_GBIF_SCACHE_CNTL0,
		  FIELD_PREP(GENMASK(14, 10), gpu_scid) |
		  BIT(8));
}

static void a7xx_llc_slices_destroy(struct a7xx_gpu *a7xx_gpu)
{
	llcc_slice_putd(a7xx_gpu->llc_slice);
	llcc_slice_putd(a7xx_gpu->htw_llc_slice);
}

static void a7xx_llc_slices_init(struct platform_device *pdev,
		struct a7xx_gpu *a7xx_gpu)
{
	/* True as of msm-5.10 */
	a7xx_gpu->have_mmu500 = true;

	a7xx_gpu->llc_mmio = msm_ioremap(pdev, "cx_mem");

	a7xx_gpu->llc_slice = llcc_slice_getd(LLCC_GPU);
	a7xx_gpu->htw_llc_slice = llcc_slice_getd(LLCC_GPUHTW);

	if (IS_ERR_OR_NULL(a7xx_gpu->llc_slice) && IS_ERR_OR_NULL(a7xx_gpu->htw_llc_slice))
		a7xx_gpu->llc_mmio = ERR_PTR(-EINVAL);
}

static int a7xx_pm_resume(struct msm_gpu *gpu)
{
	struct adreno_gpu *adreno_gpu = to_adreno_gpu(gpu);
	struct a7xx_gpu *a7xx_gpu = to_a7xx_gpu(adreno_gpu);
	int ret;

	gpu->needs_hw_init = true;

	trace_msm_gpu_resume(0);

	mutex_lock(&a7xx_gpu->gmu.lock);

	ret = a7xx_gmu_resume(a7xx_gpu);
	mutex_unlock(&a7xx_gpu->gmu.lock);
	if (ret)
		return ret;

	msm_devfreq_resume(gpu);

	a7xx_llc_activate(a7xx_gpu);

	return ret;
}

static int a7xx_pm_suspend(struct msm_gpu *gpu)
{
	struct adreno_gpu *adreno_gpu = to_adreno_gpu(gpu);
	struct a7xx_gpu *a7xx_gpu = to_a7xx_gpu(adreno_gpu);
	int i, ret;

	trace_msm_gpu_suspend(0);

	a7xx_llc_deactivate(a7xx_gpu);

	msm_devfreq_suspend(gpu);

	mutex_lock(&a7xx_gpu->gmu.lock);
	ret = a7xx_gmu_stop(a7xx_gpu);
	mutex_unlock(&a7xx_gpu->gmu.lock);
	if (ret)
		return ret;

	if (a7xx_gpu->shadow_bo) {
		for (i = 0; i < gpu->nr_rings; i++) {
			a7xx_gpu->shadow[2 * i] = 0;
			a7xx_gpu->shadow[2 * i + 1] = 0;
		}
	}

	gpu->suspend_count++;

	return 0;
}

static int a7xx_get_timestamp(struct msm_gpu *gpu, uint64_t *value)
{
	struct adreno_gpu *adreno_gpu = to_adreno_gpu(gpu);
	struct a7xx_gpu *a7xx_gpu = to_a7xx_gpu(adreno_gpu);

	mutex_lock(&a7xx_gpu->gmu.lock);

	/* Force the GPU power on so we can read this register */
	a7xx_gmu_set_oob(&a7xx_gpu->gmu, GMU_OOB_PERFCOUNTER_SET);

	*value = gpu_read64(gpu, REG_A7XX_CP_ALWAYS_ON_COUNTER);

	a7xx_gmu_clear_oob(&a7xx_gpu->gmu, GMU_OOB_PERFCOUNTER_SET);

	mutex_unlock(&a7xx_gpu->gmu.lock);

	return 0;
}

static struct msm_ringbuffer *a7xx_active_ring(struct msm_gpu *gpu)
{
	struct adreno_gpu *adreno_gpu = to_adreno_gpu(gpu);
	struct a7xx_gpu *a7xx_gpu = to_a7xx_gpu(adreno_gpu);

	return a7xx_gpu->cur_ring;
}

static void a7xx_destroy(struct msm_gpu *gpu)
{
	struct adreno_gpu *adreno_gpu = to_adreno_gpu(gpu);
	struct a7xx_gpu *a7xx_gpu = to_a7xx_gpu(adreno_gpu);

	if (a7xx_gpu->sqe_bo) {
		msm_gem_unpin_iova(a7xx_gpu->sqe_bo, gpu->aspace);
		drm_gem_object_put(a7xx_gpu->sqe_bo);
	}

	if (a7xx_gpu->shadow_bo) {
		msm_gem_unpin_iova(a7xx_gpu->shadow_bo, gpu->aspace);
		drm_gem_object_put(a7xx_gpu->shadow_bo);
	}

	a7xx_llc_slices_destroy(a7xx_gpu);

	a7xx_gmu_remove(a7xx_gpu);

	adreno_gpu_cleanup(adreno_gpu);

	kfree(a7xx_gpu);
}

static u64 a7xx_gpu_busy(struct msm_gpu *gpu, unsigned long *out_sample_rate)
{
	struct adreno_gpu *adreno_gpu = to_adreno_gpu(gpu);
	struct a7xx_gpu *a7xx_gpu = to_a7xx_gpu(adreno_gpu);
	u64 busy_cycles;

	/* 19.2MHz */
	*out_sample_rate = 19200000;

	busy_cycles = gmu_read64(&a7xx_gpu->gmu,
			REG_A7XX_GMU_CX_GMU_POWER_COUNTER_XOCLK_0_L,
			REG_A7XX_GMU_CX_GMU_POWER_COUNTER_XOCLK_0_H);

	return busy_cycles;
}

static void a7xx_gpu_set_freq(struct msm_gpu *gpu, struct dev_pm_opp *opp,
			      bool suspended)
{
	struct adreno_gpu *adreno_gpu = to_adreno_gpu(gpu);
	struct a7xx_gpu *a7xx_gpu = to_a7xx_gpu(adreno_gpu);

	mutex_lock(&a7xx_gpu->gmu.lock);
	a7xx_gmu_set_freq(gpu, opp, suspended);
	mutex_unlock(&a7xx_gpu->gmu.lock);
}

static struct msm_gem_address_space *
a7xx_create_address_space(struct msm_gpu *gpu, struct platform_device *pdev)
{
	struct adreno_gpu *adreno_gpu = to_adreno_gpu(gpu);
	struct a7xx_gpu *a7xx_gpu = to_a7xx_gpu(adreno_gpu);
	unsigned long quirks = 0;

	/*
	 * This allows GPU to set the bus attributes required to use system
	 * cache on behalf of the iommu page table walker.
	 */
	if (!IS_ERR_OR_NULL(a7xx_gpu->htw_llc_slice))
		quirks |= IO_PGTABLE_QUIRK_ARM_OUTER_WBWA;

	return adreno_iommu_create_address_space(gpu, pdev, quirks);
}

static struct msm_gem_address_space *
a7xx_create_private_address_space(struct msm_gpu *gpu)
{
	struct msm_mmu *mmu;

	mmu = msm_iommu_pagetable_create(gpu->aspace->mmu);

	if (IS_ERR(mmu))
		return ERR_CAST(mmu);

	return msm_gem_address_space_create(mmu,
		"gpu", 0x100000000ULL,
		adreno_private_address_space_size(gpu));
}

static uint32_t a7xx_get_rptr(struct msm_gpu *gpu, struct msm_ringbuffer *ring)
{
	struct adreno_gpu *adreno_gpu = to_adreno_gpu(gpu);
	struct a7xx_gpu *a7xx_gpu = to_a7xx_gpu(adreno_gpu);

	if (adreno_gpu->base.hw_apriv || a7xx_gpu->has_whereami)
		return a7xx_gpu->shadow[ring->id];

	return ring->memptrs->rptr = gpu_read(gpu, REG_A7XX_CP_RB_RPTR);
}

static u32 fuse_to_supp_hw(struct device *dev, struct adreno_rev rev, u32 fuse)
{
	u32 val = UINT_MAX;

	if (val == UINT_MAX) {
		DRM_DEV_ERROR(dev,
			"missing support for speed-bin: %u. Some OPPs may not be supported by hardware\n",
			fuse);
		return UINT_MAX;
	}

	return (1 << val);
}

static int a7xx_set_supported_hw(struct device *dev, struct adreno_rev rev)
{
	u32 supp_hw;
	u32 speedbin;
	int ret;

	ret = adreno_read_speedbin(dev, &speedbin);
	/*
	 * -ENOENT means that the platform doesn't support speedbin which is
	 * fine
	 */
	if (ret == -ENOENT) {
		return 0;
	} else if (ret) {
		dev_err_probe(dev, ret,
			      "failed to read speed-bin. Some OPPs may not be supported by hardware\n");
		return ret;
	}

	supp_hw = fuse_to_supp_hw(dev, rev, speedbin);

	ret = devm_pm_opp_set_supported_hw(dev, &supp_hw, 1);
	if (ret)
		return ret;

	return 0;
}

static const struct adreno_gpu_funcs funcs = {
	.base = {
		.get_param = adreno_get_param,
		.set_param = adreno_set_param,
		.hw_init = a7xx_hw_init,
		.pm_suspend = a7xx_pm_suspend,
		.pm_resume = a7xx_pm_resume,
		.recover = a7xx_recover,
		.submit = a7xx_submit,
		.active_ring = a7xx_active_ring,
		.irq = a7xx_irq,
		.destroy = a7xx_destroy,
		.gpu_busy = a7xx_gpu_busy,
		.gpu_get_freq = a7xx_gmu_get_freq,
		.gpu_set_freq = a7xx_gpu_set_freq,
		.create_address_space = a7xx_create_address_space,
		.create_private_address_space = a7xx_create_private_address_space,
		.get_rptr = a7xx_get_rptr,
		// .progress = a7xx_progress,
	},
	.get_timestamp = a7xx_get_timestamp,
};

struct msm_gpu *a7xx_gpu_init(struct drm_device *dev)
{
	struct msm_drm_private *priv = dev->dev_private;
	struct platform_device *pdev = priv->gpu_pdev;
	struct adreno_platform_config *config = pdev->dev.platform_data;
	struct device_node *node;
	struct a7xx_gpu *a7xx_gpu;
	struct adreno_gpu *adreno_gpu;
	struct msm_gpu *gpu;
	int ret;

	a7xx_gpu = kzalloc(sizeof(*a7xx_gpu), GFP_KERNEL);
	if (!a7xx_gpu)
		return ERR_PTR(-ENOMEM);

	adreno_gpu = &a7xx_gpu->base;
	gpu = &adreno_gpu->base;

	adreno_gpu->registers = NULL;

	/* As of msm-5.10, this is assumed true for all A7xx - might change for lower SKUs */
	adreno_gpu->base.hw_apriv = true;
	a7xx_llc_slices_init(pdev, a7xx_gpu);

	ret = a7xx_set_supported_hw(&pdev->dev, config->rev);
	if (ret) {
		a7xx_destroy(&(a7xx_gpu->base.base));
		return ERR_PTR(ret);
	}

	ret = adreno_gpu_init(dev, pdev, adreno_gpu, &funcs, 1);
	if (ret) {
		a7xx_destroy(&(a7xx_gpu->base.base));
		return ERR_PTR(ret);
	}

	/* Check if there is a GMU phandle and set it up */
	node = of_parse_phandle(pdev->dev.of_node, "qcom,gmu", 0);
	/* FIXME: How do we gracefully handle this? Should we allow nogmu operation for testing? */
	BUG_ON(!node);

	ret = a7xx_gmu_init(a7xx_gpu, node);
	of_node_put(node);
	if (ret) {
		a7xx_destroy(&(a7xx_gpu->base.base));
		return ERR_PTR(ret);
	}

	if (gpu->aspace)
		msm_mmu_set_fault_handler(gpu->aspace->mmu, gpu, a7xx_fault_handler);

	return gpu;
}
