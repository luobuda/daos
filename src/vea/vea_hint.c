/**
 * (C) Copyright 2018-2023 Intel Corporation.
 *
 * SPDX-License-Identifier: BSD-2-Clause-Patent
 */
#define D_LOGFAC	DD_FAC(vos)

#include <daos/common.h>
#include "vea_internal.h"

void
hint_get(struct vea_hint_context *hint, uint64_t *off)
{
	if (hint != NULL) {
		D_ASSERT(off != NULL);
		*off = hint->vhc_off;
	}
}

void
hint_update(struct vea_hint_context *hint, uint64_t off, uint64_t *seq)
{
	if (hint != NULL) {
		D_ASSERT(seq != NULL);
		hint->vhc_off = off;
		hint->vhc_seq++;
		*seq = hint->vhc_seq;
	}
}

/*
事务 A 的 3 次 reserve:  seq 11,       14, 15
事务 B（并发）的 2 次:    seq    12, 13
                          └───────────── 插进来了
*/
static inline bool
is_rsrv_interleaved(uint64_t seq_min, uint64_t seq_max, unsigned int seq_cnt)
{
	// 判断seq是否有交错，[seq_min,seq_max]是不是连续的
	unsigned int diff = seq_max - seq_min + 1;

	D_ASSERTF(diff >= seq_cnt, "["DF_U64", "DF_U64"] %u\n",
		  seq_min, seq_max, seq_cnt);
	return diff > seq_cnt;
}

int
hint_cancel(struct vea_hint_context *hint, uint64_t off, uint64_t seq_min,
	    uint64_t seq_max, unsigned int seq_cnt)
{
	if (hint == NULL)
		return 0;

	D_ASSERT(hint->vhc_pd != NULL);
	if (hint->vhc_seq == seq_max &&
	    !is_rsrv_interleaved(seq_min, seq_max, seq_cnt)) {
		/*
		 * This is the last reserve, and no interleaved reserve, revert
		 * the hint offset to the first offset with min sequence.
		 */
		// 是最后一批reserve，且连续，可以安全回退到off
		// 只有"整段 [seq_min, seq_max] 都是我的"时，才能把vhc_off 安全地退回
		hint->vhc_off = off;
		return 0;
	} else if (hint->vhc_seq >= seq_max) {
		/*
		 * Subsequent reserve detected, abort hint cancel. It could
		 * result in un-allocated holes on out of order hint cancels,
		 * not a big deal.
		 */
		return 0;
	}

	D_ERROR("unexpected transient hint "DF_U64" ["DF_U64", "DF_U64"] %u\n",
		hint->vhc_seq, seq_min, seq_max, seq_cnt);

	return -DER_INVAL;
}

/*
	cancel 用的是内存 vhc_seq，publish 用的是持久
	vhd_seq —— 因为回退的是内存 hint，推进的是持久 hint
*/

int
hint_tx_publish(struct umem_instance *umm, struct vea_hint_context *hint,
		uint64_t off, uint64_t seq_min, uint64_t seq_max,
		unsigned int seq_cnt)
{
	int	rc;

	D_ASSERT(umem_tx_inprogress(umm) ||
		 umm->umm_id == UMEM_CLASS_VMEM);

	if (hint == NULL)
		return 0;

	D_ASSERT(hint->vhc_pd != NULL);

	if (hint->vhc_pd->vhd_seq == seq_min ||
	    hint->vhc_pd->vhd_seq == seq_max)
		/* 内建一致性检查,持久化的seq不可能和内存递增分配的seq_min/seq_max相等，
			相等说明之前publish过，理论上不可能，重复publish了
		*/
		goto error;

	if (hint->vhc_pd->vhd_seq > seq_max) {
		/* 已经有更晚的批次推进过 hint → 不覆盖 */
		/* Subsequent reserve is already published */
		return 0;
	} else if (hint->vhc_pd->vhd_seq < seq_min ||
		   is_rsrv_interleaved(seq_min, seq_max, seq_cnt)) {
		// 持久化的vhd_seq比seq_min还小,持久hint还停在我之前，安全推进
		// 在 [seq_min, seq_max] 内 + interleaved 交叉也无法确定独占，但仍尽力推进（可能留洞）
		rc = umem_tx_add_ptr(umm, hint->vhc_pd, sizeof(*hint->vhc_pd));
		if (rc != 0)
			return rc;

		hint->vhc_pd->vhd_off = off; // 更新持久化df，VEA_BITMAP_CHUNK_HINT_KEY对应
		hint->vhc_pd->vhd_seq = seq_max;
		return 0;
	} else {
		// 在 [seq_min, seq_max] 内 + 不interleaved
		// 交叉时这批 seq 是连续的，持久 hint 可能落在中间 → 异常
		// 正常连续时走hint->vhc_pd->vhd_seq < seq_min条件更新hint
	}
error:
	D_ERROR("unexpected persistent hint "DF_U64" ["DF_U64", "DF_U64"] %u\n",
		hint->vhc_pd->vhd_seq, seq_min, seq_max, seq_cnt);

	return -DER_INVAL;
}
