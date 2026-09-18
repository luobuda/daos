/**
 * (C) Copyright 2018-2023 Intel Corporation.
 *
 * SPDX-License-Identifier: BSD-2-Clause-Patent
 */
#define D_LOGFAC	DD_FAC(vos)

#include <daos/common.h>
#include <daos/dtx.h>
#include <daos/btree_class.h>
#include "vea_internal.h"

void
destroy_free_class(struct vea_free_class *vfc)
{
	/* Destroy the in-memory sized free extent tree */
	if (daos_handle_is_valid(vfc->vfc_size_btr)) {
		dbtree_destroy(vfc->vfc_size_btr, NULL);
		vfc->vfc_size_btr = DAOS_HDL_INVAL;
	}

	d_binheap_destroy_inplace(&vfc->vfc_heap);
}

static bool
heap_node_cmp(struct d_binheap_node *a, struct d_binheap_node *b)
{
	struct vea_extent_entry *nodea, *nodeb;

	nodea = container_of(a, struct vea_extent_entry, vee_node);
	nodeb = container_of(b, struct vea_extent_entry, vee_node);

	/* Max heap, the largest free extent is heap root */
	return nodea->vee_ext.vfe_blk_cnt > nodeb->vee_ext.vfe_blk_cnt;
}

static struct d_binheap_ops heap_ops = {
	.hop_enter	= NULL,
	.hop_exit	= NULL,
	.hop_compare	= heap_node_cmp,
};

int
create_free_class(struct vea_free_class *vfc, struct vea_space_df *md)
{
	struct umem_attr	uma;
	int			rc;
	int			i;

	vfc->vfc_size_btr = DAOS_HDL_INVAL;
	rc = d_binheap_create_inplace(DBH_FT_NOLOCK, 0, NULL, &heap_ops,
				      &vfc->vfc_heap);
	if (rc != 0)
		return -DER_NOMEM;

	D_ASSERT(md->vsd_blk_sz > 0 && md->vsd_blk_sz <= (1U << 20));
	vfc->vfc_large_thresh = (VEA_LARGE_EXT_MB << 20) / md->vsd_blk_sz;

	memset(&uma, 0, sizeof(uma));
	uma.uma_id = UMEM_CLASS_VMEM;
	/* Create in-memory sized free extent tree */
	rc = dbtree_create(DBTREE_CLASS_IFV, BTR_FEAT_UINT_KEY, VEA_TREE_ODR, &uma, NULL,
			   &vfc->vfc_size_btr);
	if (rc != 0) {
		destroy_free_class(vfc);
		goto out;
	}

	for (i = 0; i < VEA_MAX_BITMAP_CLASS; i++) {
		D_INIT_LIST_HEAD(&vfc->vfc_bitmap_lru[i]);
		D_INIT_LIST_HEAD(&vfc->vfc_bitmap_empty[i]);
	}

out:
	return rc;
}

void
unload_space_info(struct vea_space_info *vsi)
{
	if (daos_handle_is_valid(vsi->vsi_md_free_btr)) {
		dbtree_close(vsi->vsi_md_free_btr);
		vsi->vsi_md_free_btr = DAOS_HDL_INVAL;
	}

	if (daos_handle_is_valid(vsi->vsi_md_bitmap_btr)) {
		dbtree_close(vsi->vsi_md_bitmap_btr);
		vsi->vsi_md_bitmap_btr = DAOS_HDL_INVAL;
	}

	if (vsi->vsi_bitmap_hint_context) {
		vea_hint_unload(vsi->vsi_bitmap_hint_context);
		vsi->vsi_bitmap_hint_context = NULL;
	}
}

// 从持久化存储中加载vea
static int
load_free_entry(daos_handle_t ih, d_iov_t *key, d_iov_t *val, void *arg)
{
	struct vea_free_extent *vfe;
	struct vea_space_info *vsi;
	uint64_t *off;
	int rc;

	vsi = (struct vea_space_info *)arg;
	off = (uint64_t *)key->iov_buf;
	vfe = (struct vea_free_extent *)val->iov_buf;

	rc = verify_free_entry(off, vfe); // key value的off匹配
	if (rc != 0)
		return rc;
	// 插入vsi_free_btr同时分类到heap + size tree
	rc = compound_free_extent(vsi, vfe, VEA_FL_NO_MERGE);
	if (rc != 0)
		return rc;

	return 0;
}
// 从持久化存储中加载vea bitmap
static int
load_bitmap_entry(daos_handle_t ih, d_iov_t *key, d_iov_t *val, void *arg)
{
	struct vea_free_bitmap *vfb;
	struct vea_space_info *vsi;
	struct vea_bitmap_entry *bitmap_entry;
	uint64_t *off;
	int rc;

	vsi = (struct vea_space_info *)arg;
	off = (uint64_t *)key->iov_buf;
	if (*off == VEA_BITMAP_CHUNK_HINT_KEY) // 哨兵条目
		return 0;

	vfb = (struct vea_free_bitmap *)val->iov_buf;
	rc = verify_bitmap_entry(vfb);
	if (rc != 0)
		return rc;
	// 插入到vsi_bitmap_btr
	rc = bitmap_entry_insert(vsi, vfb, VEA_BITMAP_STATE_PUBLISHED, &bitmap_entry, 0);
	bitmap_entry->vbe_md_bitmap = vfb;

	return rc;
}

/*
 纯 DRAM操作，它不负责从 NVMe 加载任何东西

 从 NVMe 把元数据搬进 DRAM 的工作，在dav_obj_open_v2() 里由两个阶段完成：
  1. so_wal_replay —— 按 WAL 记录按需加载被改过的zone（顺带把增量重放上去）
  2. heap_load_nonevictable_zones() ——一次性加载所有不可驱逐 zone（VEA的全部元数据都在其中）
  VEA属于non-evictable MBs

  持久状态 = 上次 checkpoint（meta blob）+
  ▎ 之后的改动（WAL）。DRAM 是它的子集缓存，粒度zone(16MB)/page(4KB)：可驱逐 zone
  ▎ 的页（含脏页，驱逐前先写回 meta blob）随时可不在内存，下次访问 fault-in；只有 NEMB
  ▎ 上的分配和被 pin 的页必须常驻。因此 SSD 元数据 > DRAM 是设计前提，不需要（也不可能）把完整
  ▎ checkpoint 装进内存。

  */
int
load_space_info(struct vea_space_info *vsi)
{
	struct umem_attr uma = {0};
	int rc;
	struct vea_hint_df *df;
	uint64_t offset;
	d_iov_t key, val;

	D_ASSERT(vsi->vsi_umem != NULL);
	D_ASSERT(vsi->vsi_md != NULL);

	/* Open SCM free extent tree */
	uma.uma_id = vsi->vsi_umem->umm_id;
	uma.uma_pool = vsi->vsi_umem->umm_pool;

	D_ASSERT(daos_handle_is_inval(vsi->vsi_md_free_btr));
	rc = dbtree_open_inplace(&vsi->vsi_md->vsd_free_tree, &uma,
				 &vsi->vsi_md_free_btr);
	if (rc != 0)
		goto error;

	/* Open SCM bitmap tree */
	rc = dbtree_open_inplace(&vsi->vsi_md->vsd_bitmap_tree, &uma,
				 &vsi->vsi_md_bitmap_btr);
	if (rc != 0)
		goto error;

	/* Build up in-memory compound free extent index */
	rc = dbtree_iterate(vsi->vsi_md_free_btr, DAOS_INTENT_DEFAULT, false,
			    load_free_entry, (void *)vsi);
	if (rc != 0)
		goto error;

	/* Build up in-memory bitmap tree */
	rc = dbtree_iterate(vsi->vsi_md_bitmap_btr, DAOS_INTENT_DEFAULT, false,
			    load_bitmap_entry, (void *)vsi);
	if (rc != 0)
		goto error;

	if (!is_bitmap_feature_enabled(vsi))
		return 0;

	offset = VEA_BITMAP_CHUNK_HINT_KEY;
	d_iov_set(&key, &offset, sizeof(offset));
	d_iov_set(&val, NULL, 0);
	rc = dbtree_fetch(vsi->vsi_md_bitmap_btr, BTR_PROBE_EQ, DAOS_INTENT_DEFAULT,
			  &key, NULL, &val);
	if (rc)
		goto error;

	df = (struct vea_hint_df *)val.iov_buf; // 树上的地址，后续直接通过地址修改
	rc = vea_hint_load(df, &vsi->vsi_bitmap_hint_context);
	if (rc)
		goto error;

	return 0;
error:
	unload_space_info(vsi);
	return rc;
}
