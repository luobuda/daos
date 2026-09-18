/**
 * (C) Copyright 2018-2023 Intel Corporation.
 * (C) Copyright 2025 Hewlett Packard Enterprise Development LP
 *
 * SPDX-License-Identifier: BSD-2-Clause-Patent
 */

#ifndef __VEA_INTERNAL_H__
#define __VEA_INTERNAL_H__

#include <gurt/list.h>
#include <gurt/heap.h>
#include <daos/mem.h>
#include <daos/btree.h>
#include <daos/common.h>
#include <daos_srv/vea.h>

#define VEA_MAGIC	(0xea201804)
#define VEA_BLK_SZ	(4 * 1024)	/* 4K */
#define VEA_TREE_ODR	20

/* Min bitmap allocation class */
#define VEA_MIN_BITMAP_CLASS	1
/* Max bitmap allocation class */
#define VEA_MAX_BITMAP_CLASS	64

/* Bitmap chunk size */
#define VEA_BITMAP_MIN_CHUNK_BLKS	256				/* 1MiB */
#define VEA_BITMAP_MAX_CHUNK_BLKS	(VEA_MAX_BITMAP_CLASS * 256)	/* 64 MiB */

/*

VEA按4k管理空间，偏移都是/4k的

初始化
 vos_pool_create_ex
   - vea_format

 pool_open_post
   - vea_load

vea_reserve() 从不单独使用——它必然配一个publish 或 cancel
分配:  vea_reserve ──► [写数据] ──► vea_tx_publish (提交) / vea_cancel (中止)
释放:  vea_free  (一步到位)  
 
VEA空间申请
 vos_update_begin： Prepare IO sink buffers
   - vos_space_hold：当前空间使用情况，判断空间是否足够，不足直接DER_NOSPACE
   - dkey_update_begin
     - akey_update_begin
	   - DAOS_IOD_SINGLE ——> vos_reserve_single -> reserve_space -> vos_reserve_blocks
	   - DAOS_IOD_ARRAY ——> vos_reserve_recx -> vos_reserve_blocks
	   
	      - vos_reserve_blocks
		    - vea_reserve
			  - reserve_single

 vos_update_end
   - vos_tx_end
     - vos_publish_blocks
	   - vea_tx_publish
	     - process_resrvd_list
		   - process_free_entry
	   - vea_cancel  # IO流程失败直接将空间返回给free
  
  
VEA空间释放 agg，punch，discard，将是否空间插入agg tree

svt_free_payload/evt_dop_bio_free
  - vos_bio_addr_free
    - vea_free 
	
agg tree ——> free tree
  flush_ult
     - vos_flush_pool
	   - vea_flush
	    - trigger_aging_flush
	

nvme空间释放
vos_blob_unmap_cb
  - bio_blob_unmap_sgl
   - blob_unmap_sgl
    - spdk_blob_io_unmap

*/

/* Common free bitmap structure for both SCM & in-memory index */
struct vea_free_bitmap {
	/* Block offset of the bitmap 当前chunk的起始偏移*/
	uint64_t	vfb_blk_off;
	/* Block count of the bitmap 当前chunk的4k blk大小*/		
	uint32_t	vfb_blk_cnt;
	/* Allocation class of bitmap 当前chunk负责分配的blk类型(区间大小)*/				
	uint16_t	vfb_class;
	/* Bitmap size 有多少个uint64_t*/
	uint16_t	vfb_bitmap_sz;
	/* Bitmaps of this chunk */			
	uint64_t	vfb_bitmaps[0];				
};

/* Per I/O stream hint context */
struct vea_hint_context {
	struct vea_hint_df	*vhc_pd;
	/* In-memory hint block offset */
	uint64_t		 vhc_off;
	/* In-memory hint sequence */
	uint64_t		 vhc_seq;
};

/* Free extent informat stored in the in-memory compound free extent index */
struct vea_extent_entry {
	/*
	 * Always keep it as first item, since vfe_blk_off is the direct key
	 * of DBTREE_CLASS_IV
	 */
	struct vea_free_extent	 vee_ext;
	/* Link to one of vsc_extent_lru */
	d_list_t		 vee_link; // 插入的size tree节点链表
	/* Back reference to sized tree entry */
	struct vea_sized_class	*vee_sized_class; // 指向size tree节点
	/* Link to vfc_heap */
	struct d_binheap_node	 vee_node; // heap节点
};

enum {
	VEA_BITMAP_STATE_PUBLISHED, // 已经插入到持久化树
	VEA_BITMAP_STATE_PUBLISHING, // 正在插入持久化树
	VEA_BITMAP_STATE_NEW,       // 新分配的chunk
};

/* Bitmap entry */
struct vea_bitmap_entry {
	/* Link to one of vfc_bitmap_lru[] */
	d_list_t		 vbe_link;
	/* Bitmap published state */
	int			 vbe_published_state;
	/*
	 * Free entries sorted by offset, for coalescing the just recent
	 * free blocks inside this bitmap chunk.
	 */
	daos_handle_t		 vbe_agg_btr; // 最近释放的blocks按offset排序，方便聚合
	/* Point to persistent free bitmap entry */
	struct vea_free_bitmap	*vbe_md_bitmap;
	/* free bitmap, always keep it as last item*/
	struct vea_free_bitmap	 vbe_bitmap;
};

enum {
	VEA_FREE_ENTRY_EXTENT,
	VEA_FREE_ENTRY_BITMAP,
};

/* freed entry stored in aggregation tree */
struct vea_free_entry {
	struct vea_free_extent	 vfe_ext;
	/* Back pointer bitmap entry */
	struct vea_bitmap_entry	*vfe_bitmap;
	/* Link to one vsi_agg_lru */
	d_list_t		 vfe_link;
};

#define VEA_LARGE_EXT_MB	64	/* Large extent threshold in MB */
#define VEA_HINT_OFF_INVAL	0	/* Invalid hint offset */

/* Value entry of sized free extent tree (vfc_size_btr) */
struct vea_sized_class {
	/* Small extents LRU list */
	d_list_t		vsc_extent_lru; // 相同大小的Small extents用链表串起来
};

#define VEA_BITMAP_CHUNK_HINT_KEY	(~(0ULL))
/*
 * Large free extents (>VEA_LARGE_EXT_MB) are tracked in max a heap, small
 * free extents (<= VEA_LARGE_EXT_MB) are tracked in a size tree.
 */
struct vea_free_class {
	/* Max heap for tracking the largest free extent */
	struct d_binheap	vfc_heap;
	/* Small free extent tree */
	// 小的区间按照区间的blk_cnt作为key组织b+tree，相同区间的extent用链表串起来vea_sized_class
	daos_handle_t		vfc_size_btr;
	/* Size threshold for large extent */
	uint32_t		vfc_large_thresh; // 64M
	/* Bitmap LRU list for different bitmap allocation class*/
	// 1~64种blk cnt个数类型各一个链表，即256K以下的extent分配都优先用bitmap
	d_list_t		vfc_bitmap_lru[VEA_MAX_BITMAP_CLASS]; // 这块 chunk是部分free的
	/* Empty bitmap list for different allocation class */
	d_list_t		vfc_bitmap_empty[VEA_MAX_BITMAP_CLASS]; // 这块chunk是全free的
	// 一个chunk三种状态，新/全空（所有 bit 为 0）、用过一部分、还有空闲、已满（没有一个 bit 为 0）
};

enum {
	/* Number of hint reserve */
	STAT_RESRV_HINT		= 0,
	/* Number of large reserve */
	STAT_RESRV_LARGE	= 1,
	/* Number of small extents reserve */
	STAT_RESRV_SMALL	= 2,
	/* Number of bitmap reserve */
	STAT_RESRV_BITMAP	= 3,
	/* Max reserve type */
	STAT_RESRV_TYPE_MAX	= 4,
	/* Number of large(> VEA_LARGE_EXT_MB) free frags available for allocation */
	STAT_FRAGS_LARGE	= 4,
	/* Number of small free extent frags available for allocation */
	STAT_FRAGS_SMALL	= 5,
	/* Number of frags in aging buffer (to be unmapped) */
	STAT_FRAGS_AGING	= 6,
	/* Number of bitmaps */
	STAT_FRAGS_BITMAP	= 7,
	/* Max frag type */
	STAT_FRAGS_TYPE_MAX	= 4,
	/* Number of extent blocks available for allocation */
	STAT_FREE_EXTENT_BLKS	= 8,
	/* Number of bitmap blocks available for allocation */
	STAT_FREE_BITMAP_BLKS	= 9,
	STAT_MAX		= 10,
};

struct vea_metrics {
	struct d_tm_node_t	*vm_rsrv[STAT_RESRV_TYPE_MAX];
	struct d_tm_node_t	*vm_frags[STAT_FRAGS_TYPE_MAX];
	struct d_tm_node_t	*vm_free_blks;
};

#define MAX_FLUSH_FRAGS	256


/*

vsi_free_btr：部空闲 extent
  键 / 序: vfe_blk_off

vfc_heap: > vfc_large_thresh（默认64MB）的大 extent
  键 / 序: max-heap by blk_cnt

vfc_size_btr：<= 64MB 的小 extent，按精确大小分桶
  键 / 序: 按blk_cnt个数 → struct vea_sized_class{vsc_extent_lru}

vsi_free_btr = vfc_heap ⊎ vfc_size_btr ，三者是同一批vea_extent_entry对象的三个视图

vsi_bitmap_btr（持久副本 vsi_md_bitmap_btr): 全部 bitmap
  键 / 序: vfb_blk_off

vfc_bitmap_lru[class-1]: 部分占用的 chunk
  键 / 序: class 分桶

vfc_bitmap_empty[class-1]: 全空的 chunk
  键 / 序: class 分桶

vsi_bitmap_btr = vfc_bitmap_lru ⊎ vfc_bitmap_empty
*/

/* In-memory compound index */
struct vea_space_info {
	/* Instance for the pmemobj pool on SCM */
	struct umem_instance		*vsi_umem;
	/*
	 * Stage callback data used by PMDK transaction.
	 *
	 * No public API offered by PMDK to get transaction stage callback
	 * data, so we have to pass it around.
	 */
	struct umem_tx_stage_data	*vsi_txd;
	/* Free space information stored on SCM */
	struct vea_space_df		*vsi_md;
	/* Open handles for the persistent free extent tree */
	daos_handle_t			 vsi_md_free_btr;
	/* Open handles for the persistent bitmap tree */
	daos_handle_t			 vsi_md_bitmap_btr;
	/* Free extent tree sorted by offset, for all free extents. */
	daos_handle_t			 vsi_free_btr;
	/* Bitmap tree, for small allocation */
	daos_handle_t			 vsi_bitmap_btr;
	/* Hint context for bitmap chunk allocation */
	struct vea_hint_context		*vsi_bitmap_hint_context;
	/* Index for searching free extent by size & age */
	struct vea_free_class		 vsi_class;
	/* LRU to aggregate just recent freed extents or bitmap blocks */
	d_list_t			 vsi_agg_lru;
	/*
	 * Free entries sorted by offset, for coalescing the just recent
	 * free extents.
	 */
	daos_handle_t			 vsi_agg_btr;
	/* Unmap context to perform unmap against freed extent */
	struct vea_unmap_context	 vsi_unmap_ctxt;
	/* Statistics */
	uint64_t			 vsi_stat[STAT_MAX];
	/* Metrics */
	struct vea_metrics		*vsi_metrics;
	/* Last aging buffer flush timestamp */
	uint32_t			 vsi_flush_time;
	bool				 vsi_flush_scheduled;
};

struct free_commit_cb_arg {
	struct vea_space_info	*fca_vsi;
	struct vea_free_entry	 fca_vfe;
};

static inline uint32_t
get_current_age(void)
{
	uint64_t age = 0;

	age = daos_gettime_coarse();
	return (uint32_t)age;
}

enum vea_free_flags {
	VEA_FL_NO_MERGE		= (1 << 0),
	VEA_FL_NO_ACCOUNTING	= (1 << 1),
};

static inline bool
is_bitmap_feature_enabled(struct vea_space_info *vsi)
{
	return vsi->vsi_md->vsd_compat & VEA_COMPAT_FEATURE_BITMAP;
}

static inline int
alloc_free_bitmap_size(uint16_t bitmap_sz)
{
	return sizeof(struct vea_free_bitmap) + (bitmap_sz << 3);
}

static inline uint32_t
bitmap_free_blocks(struct vea_free_bitmap *vfb)
{
	uint32_t	free_blocks;
	int		diff;

	int free_bits = daos_count_free_bits(vfb->vfb_bitmaps, vfb->vfb_bitmap_sz);

	free_blocks = free_bits * vfb->vfb_class;
	diff = vfb->vfb_bitmap_sz * 64 * vfb->vfb_class - vfb->vfb_blk_cnt;

	D_ASSERT(diff == 0);

	return free_blocks;
}

static inline bool
is_bitmap_empty(uint64_t *bitmap, int bitmap_sz)
{
	int i;

	for (i = 0; i < bitmap_sz; i++)
		if (bitmap[i])
			return false;

	return true;
}

/* vea_init.c */
void destroy_free_class(struct vea_free_class *vfc);
int create_free_class(struct vea_free_class *vfc, struct vea_space_df *md);
void unload_space_info(struct vea_space_info *vsi);
int load_space_info(struct vea_space_info *vsi);

/* vea_util.c */
int verify_free_entry(uint64_t *off, struct vea_free_extent *vfe);
int verify_bitmap_entry(struct vea_free_bitmap *vfb);
int ext_adjacent(struct vea_free_extent *cur, struct vea_free_extent *next);
int verify_resrvd_ext(struct vea_resrvd_ext *resrvd);
int vea_dump(struct vea_space_info *vsi, bool transient);
int vea_verify_alloc(struct vea_space_info *vsi, bool transient,
		     uint64_t off, uint32_t cnt, bool is_bitmap);
void dec_stats(struct vea_space_info *vsi, unsigned int type, uint64_t nr);
void inc_stats(struct vea_space_info *vsi, unsigned int type, uint64_t nr);

/* vea_alloc.c */
int reserve_hint(struct vea_space_info *vsi, uint32_t blk_cnt,
		 struct vea_resrvd_ext *resrvd);
int reserve_single(struct vea_space_info *vsi, uint32_t blk_cnt,
		   struct vea_resrvd_ext *resrvd);
int persistent_alloc(struct vea_space_info *vsi, struct vea_free_entry *vfe);
int
bitmap_tx_add_ptr(struct umem_instance *vsi_umem, uint64_t *bitmap,
		  uint32_t bit_at, uint32_t bits_nr);
int
bitmap_set_range(struct umem_instance *vsi_umem, struct vea_free_bitmap *bitmap,
		 uint64_t blk_off, uint32_t blk_cnt, bool clear);

/* vea_free.c */
void extent_free_class_remove(struct vea_space_info *vsi, struct vea_extent_entry *entry);
int extent_free_class_add(struct vea_space_info *vsi, struct vea_extent_entry *entry);
int compound_free_extent(struct vea_space_info *vsi, struct vea_free_extent *vfe,
			 unsigned int flags);
int compound_free(struct vea_space_info *vsi, struct vea_free_entry *vfe, unsigned int flags);
int persistent_free(struct vea_space_info *vsi, struct vea_free_entry *vfe);
int aggregated_free(struct vea_space_info *vsi, struct vea_free_entry *vfe);
int trigger_aging_flush(struct vea_space_info *vsi, bool force,
			uint32_t nr_flush, uint32_t *nr_flushed);
int bitmap_entry_insert(struct vea_space_info *vsi, struct vea_free_bitmap *vfb,
			int state, struct vea_bitmap_entry **ret_entry, unsigned int flags);
int free_type(struct vea_space_info *vsi, uint64_t blk_off, uint32_t blk_cnt,
	      struct vea_bitmap_entry **bitmap_entry);
void
free_commit_cb(void *data, bool noop);

/* vea_hint.c */
void hint_get(struct vea_hint_context *hint, uint64_t *off);
void hint_update(struct vea_hint_context *hint, uint64_t off, uint64_t *seq);
int hint_cancel(struct vea_hint_context *hint, uint64_t off, uint64_t seq_min,
		uint64_t seq_max, unsigned int seq_cnt);
int hint_tx_publish(struct umem_instance *umm, struct vea_hint_context *hint,
		    uint64_t off, uint64_t seq_min, uint64_t seq_max,
		    unsigned int seq_cnt);

#endif /* __VEA_INTERNAL_H__ */
