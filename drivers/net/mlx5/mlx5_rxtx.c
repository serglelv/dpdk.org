/*-
 *   BSD LICENSE
 *
 *   Copyright 2015 6WIND S.A.
 *   Copyright 2015 Mellanox.
 *
 *   Redistribution and use in source and binary forms, with or without
 *   modification, are permitted provided that the following conditions
 *   are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in
 *       the documentation and/or other materials provided with the
 *       distribution.
 *     * Neither the name of 6WIND S.A. nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *   A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *   OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *   DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *   THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *   OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <assert.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>

/* Verbs header. */
/* ISO C doesn't support unnamed structs/unions, disabling -pedantic. */
#ifdef PEDANTIC
#pragma GCC diagnostic ignored "-pedantic"
#endif
#include <infiniband/verbs.h>
#include <infiniband/mlx5_hw.h>
#include <infiniband/arch.h>
#ifdef PEDANTIC
#pragma GCC diagnostic error "-pedantic"
#endif

/* DPDK headers don't like -pedantic. */
#ifdef PEDANTIC
#pragma GCC diagnostic ignored "-pedantic"
#endif
#include <rte_mbuf.h>
#include <rte_mempool.h>
#include <rte_prefetch.h>
#include <rte_common.h>
#include <rte_branch_prediction.h>
#include <rte_memory.h>
#include <rte_memcpy.h>
#ifdef PEDANTIC
#pragma GCC diagnostic error "-pedantic"
#endif

#include "mlx5.h"
#include "mlx5_utils.h"
#include "mlx5_rxtx.h"
#include "mlx5_autoconf.h"
#include "mlx5_defs.h"
#include "mlx5_hw.h"

static inline int
check_cqe64(volatile struct mlx5_cqe64 *cqe,
	    unsigned int cqes_n, const uint16_t ci)
	    __attribute__((always_inline));

/**
 * Check if the CQE is valid
 *
 * @param  cqe
 *   The Pointer to the CQE
 * @param cqes_n
 *   Completion Queue size
 * @param ci
 *   Pointer to the consumer index
 *
 * @return
 *   0 on success, 1 on failure.
 */
static inline int
check_cqe64(volatile struct mlx5_cqe64 *cqe,
	    unsigned int cqes_n, const uint16_t ci)
{
	uint16_t idx = ci & cqes_n;
	uint8_t op_own = cqe->op_own;
	uint8_t op_owner = MLX5_CQE_OWNER(op_own);
	uint8_t op_code = MLX5_CQE_OPCODE(op_own);

	if (unlikely((op_owner != (!!(idx))) || (op_code == MLX5_CQE_INVALID)))
		return 1; /* No CQE */
#ifndef NDEBUG
	else if (unlikely((op_code != MLX5_CQE_RESP_SEND) &&
			  (op_code != MLX5_CQE_REQ)))
		rte_panic("Error on CQE opcode %x\n", op_code);
#endif /* NDEBUG */
	return 0;
}

#if MLX5_PMD_MAX_INLINE == 0
/**
 * Manage TX compressed completions.
 *
 * @param txq
 *   Pointer to TX queue structure.
 * @param  cqe
 *   Poniter to the compressed CQE.
 * @param cq_ci
 *   Pointer to the current consumer index.
 *
 * @retrun
 *   The wqe_ci corresponding to the last packet sent.
 */
static inline uint16_t
txq_complete_compressed(struct ftxq *txq, volatile struct mlx5_cqe64 *cqe,
			uint16_t *cq_ci, uint16_t cqe_n)
{
	volatile struct mlx5_mini_cqe8 (*mc)[8];
	uint32_t cqe_cnt = ntohl(cqe->byte_cnt);
	uint16_t mc_pos;
	uint16_t wqe_ci;


	/* The position of the compressed array is after the title CQE, here
	 * the title CQE is the variable cqe, the next position are in
	 * multiple of 8 (8 bytes per mini CQE in a single 64 byte CQE)
	 * if they exists. */
	mc_pos = ((*cq_ci) + 1 + (cqe_cnt & 0xf8)) & cqe_n;
	/* First get the last wqe_ci from the mini cqe. */
	mc = (volatile struct mlx5_mini_cqe8 (*)[8])
		(uintptr_t)&(*txq->cqes)[mc_pos];
	wqe_ci = ntohs((*mc)[(cqe_cnt - 1) & 7].s_wqe_info.wqe_counter);
	/* Reset the owner bit in all CQE. */
	while(cqe_cnt) {
		cqe = &(*txq->cqes)[(*cq_ci) & cqe_n];
		cqe->op_own = MLX5_CQE_INVALIDATE;
		++(*cq_ci);
		--cqe_cnt;
	}

	return wqe_ci;
}

/**
 * Manage TX completions.
 *
 * When sending a burst, mlx5_tx_burst() posts several WRs.
 * To improve performance, a completion event is only required once every
 * MLX5_PMD_TX_PER_COMP_REQ sends. Doing so discards completion information
 * for other WRs, but this information would not be used anyway.
 *
 * @param txq
 *   Pointer to TX queue structure.
 */
static void
txq_complete(struct ftxq *txq)
{
	const unsigned int elts_n = txq->elts_n;
	const unsigned int cqe_cnt = txq->cqe_cnt;
	const unsigned int cqe_n = cqe_cnt + 1;
	uint16_t elts_free = txq->elts_tail;
	uint16_t elts_tail;
	uint16_t cq_ci = txq->cq_ci;
	unsigned int wqe_ci = (unsigned int)-1;
	int ret = 0;

	while (ret == 0) {
		unsigned int idx = cq_ci & cqe_cnt;
		volatile struct mlx5_cqe64 *cqe = &(*txq->cqes)[idx];

		ret = check_cqe64(cqe, cqe_n, cq_ci);
		if (ret == 1)
			break;

		if (MLX5_CQE_FORMAT(cqe->op_own) == MLX5_COMPRESSED)
			wqe_ci = txq_complete_compressed(txq, cqe,
							 &cq_ci, cqe_cnt);
		else if ((MLX5_CQE_OPCODE(cqe->op_own) == MLX5_CQE_REQ)) {
			wqe_ci = ntohs(cqe->wqe_counter);
			++cq_ci;
		}
	}
	if (unlikely(wqe_ci == (unsigned int)-1))
		return;
	/* Free buffers. */
	elts_tail = wqe_ci & (elts_n - 1);
	while (elts_free != elts_tail) {
		struct rte_mbuf *elt = (*txq->elts)[elts_free];
		unsigned int elts_free_next =
			(elts_free + 1) & (elts_n - 1);
		struct rte_mbuf *elt_next = (*txq->elts)[elts_free_next];

		(*txq->elts)[elts_free] = NULL;
		RTE_MBUF_PREFETCH_TO_FREE(elt_next);
		/* Faster than rte_pktmbuf_free(). */
		do {
			struct rte_mbuf *next = NEXT(elt);

			rte_pktmbuf_free_seg(elt);
			elt = next;
		} while (elt != NULL);
		elts_free = elts_free_next;
	}
	txq->cq_ci = cq_ci;
	txq->elts_tail = elts_tail;
	/* Update the consumer index. */
	rte_wmb();
	*txq->cq_db = htonl(txq->cq_ci);
}

#else /* MLX5_PMD_MAX_INLINE == 0 */

/**
 * Manage TX compressed completions.
 *
 * @param txq
 *   Pointer to TX queue structure.
 * @param  cqe
 *   Poniter to the compressed CQE.
 * @param cq_ci
 *   Pointer to the current consumer index.
 *
 * @retrun
 *   Number of received completions.
 */
static inline uint32_t
txq_complete_compressed(struct ftxq *txq, volatile struct mlx5_cqe64 *cqe,
			uint16_t *cq_ci, uint16_t cqe_n)
{
	uint32_t cqe_cnt = ntohl(cqe->byte_cnt);
	uint32_t cqe_polled;

	cqe_polled = cqe_cnt;
	/* Reset the owner bit in all CQE. */
	while(cqe_cnt) {
		cqe = &(*txq->cqes)[(*cq_ci) & cqe_n];
		cqe->op_own = MLX5_CQE_INVALIDATE;
		++(*cq_ci);
		--cqe_cnt;
	}

	return cqe_polled;
}

/**
 * Manage TX completions.
 *
 * When sending a burst, mlx5_tx_burst() posts several WRs.
 * To improve performance, a completion event is only required once every
 * MLX5_PMD_TX_PER_COMP_REQ sends. Doing so discards completion information
 * for other WRs, but this information would not be used anyway.
 *
 * @param txq
 *   Pointer to TX queue structure.
 */
static void
txq_complete(struct ftxq *txq)
{
	const unsigned int elts_n = txq->elts_n;
	const unsigned int cqe_cnt = txq->cqe_cnt;
	const unsigned int cqe_n = cqe_cnt + 1;
	uint16_t elts_free = txq->elts_tail;
	uint16_t elts_tail = txq->elts_tail;
	uint16_t cq_ci = txq->cq_ci;
	unsigned int npolled = 0;
	int ret = 0;

	while (ret == 0) {
		unsigned int idx = cq_ci & cqe_cnt;
		volatile struct mlx5_cqe64 *cqe = &(*txq->cqes)[idx];

		ret = check_cqe64(cqe, cqe_n, cq_ci);
		if (ret == 1)
			break;

		if (MLX5_CQE_FORMAT(cqe->op_own) == MLX5_COMPRESSED)
			npolled += txq_complete_compressed(txq, cqe,
							   &cq_ci, cqe_cnt);
		else if ((MLX5_CQE_OPCODE(cqe->op_own) == MLX5_CQE_REQ)) {
			npolled++;
			++cq_ci;
		}
	}
	if (unlikely(npolled == 0))
		return;
	/* Free buffers. */
	elts_tail += npolled * txq->elts_comp_cd_init;
	if (elts_tail >= elts_n)
		elts_tail -= elts_n;
	while (elts_free != elts_tail) {
		struct rte_mbuf *elt = (*txq->elts)[elts_free];
		unsigned int elts_free_next =
			(elts_free + 1) & (elts_n - 1);
		struct rte_mbuf *elt_next = (*txq->elts)[elts_free_next];

		(*txq->elts)[elts_free] = NULL;
		RTE_MBUF_PREFETCH_TO_FREE(elt_next);
		/* Faster than rte_pktmbuf_free(). */
		do {
			struct rte_mbuf *next = NEXT(elt);

			rte_pktmbuf_free_seg(elt);
			elt = next;
		} while (elt != NULL);
		elts_free = elts_free_next;
	}
	txq->cq_ci = cq_ci;
	txq->elts_tail = elts_tail;
	/* Update the consumer index. */
	rte_wmb();
	*txq->cq_db = htonl(txq->cq_ci);
}
#endif /* MLX5_PMD_MAX_INLINE == 0 */

/**
 * Get Memory Pool (MP) from mbuf. If mbuf is indirect, the pool from which
 * the cloned mbuf is allocated is returned instead.
 *
 * @param buf
 *   Pointer to mbuf.
 *
 * @return
 *   Memory pool where data is located for given mbuf.
 */
static struct rte_mempool *
txq_mb2mp(struct rte_mbuf *buf)
{
	if (unlikely(RTE_MBUF_INDIRECT(buf)))
		return rte_mbuf_from_indirect(buf)->pool;
	return buf->pool;
}

static inline uint32_t
txq_mp2mr(struct ftxq *txq, const struct rte_mempool *mp)
	__attribute__((always_inline));

/**
 * Get Memory Region (MR) <-> Memory Pool (MP) association from txq->mp2mr[].
 * Add MP to txq->mp2mr[] if it's not registered yet. If mp2mr[] is full,
 * remove an entry first.
 *
 * @param txq
 *   Pointer to TX queue structure.
 * @param[in] mp
 *   Memory Pool for which a Memory Region lkey must be returned.
 *
 * @return
 *   mr->lkey on success, (uint32_t)-1 on failure.
 */
static inline uint32_t
txq_mp2mr(struct ftxq *txq, const struct rte_mempool *mp)
{
	unsigned int i;
	uint32_t lkey = (uint32_t)-1;

	for (i = 0; (i != RTE_DIM(txq->mp2mr)); ++i) {
		if (unlikely(txq->mp2mr[i].mp == NULL)) {
			/* Unknown MP, add a new MR for it. */
			break;
		}
		if (txq->mp2mr[i].mp == mp) {
			assert(txq->mp2mr[i].lkey != (uint32_t)-1);
			assert(htonl(txq->mp2mr[i].mr->lkey) ==
			       txq->mp2mr[i].lkey);
			lkey = txq->mp2mr[i].lkey;
			break;
		}
	}
	if (unlikely(lkey == (uint32_t)-1))
		lkey = txq_mp2mr_reg(txq, mp, i);

	return lkey;
}

#if MLX5_PMD_MAX_INLINE == 0
static inline void
mlx5_wqe_write(struct ftxq *txq, volatile struct mlx5_wqe64 *wqe,
	       uintptr_t addr, uint32_t length, uint32_t lkey)
{
	/* Copy the first bytes into the inline header */
	rte_memcpy((void *)(uintptr_t)wqe->eseg.inline_hdr_start,
		   (void *)(uintptr_t)addr, MLX5_ETH_INLINE_HEADER_SIZE);
	addr += MLX5_ETH_INLINE_HEADER_SIZE;
	length -= MLX5_ETH_INLINE_HEADER_SIZE;

	wqe->dseg.byte_count = htonl(length);
	wqe->dseg.lkey = lkey;
	wqe->dseg.addr = htonll(addr);

	/* Write only data[0] which is the single element which changes.
	 * Other fields are already initialised in txq_alloc_elts. */
	wqe->ctrl.data[0] = htonl((txq->wqe_ci << 8) | MLX5_OPCODE_SEND);

	/* Increase the consumer index. */
	++txq->wqe_ci;
}

static inline void
mlx5_wqe_write_vlan(struct ftxq *txq, volatile struct mlx5_wqe64 *wqe,
		    uintptr_t addr, uint32_t length, uint32_t lkey,
		    uint16_t vlan_tci)
{
	uint32_t vlan = htonl(0x81000000 | vlan_tci);

	wqe->eseg.inline_hdr_sz = htons(MLX5_ETH_VLAN_INLINE_HEADER_SIZE);
	/*
	 * Copy 12 bytes of source & destination MAC address.
	 * Copy 4 bytes of vlan.
	 * Copy 2 bytes of ether type.
	 */
	memcpy((void *)(uintptr_t)wqe->eseg.inline_hdr_start,
	       (void *)(uintptr_t)addr,
	       12);
	memcpy((void *)((uintptr_t)wqe->eseg.inline_hdr_start + 12),
	       &vlan,
	       sizeof(vlan));
	memcpy((void *)((uintptr_t)wqe->eseg.inline_hdr_start + 16),
	       (void *)((uintptr_t)addr + 12),
	       2);

	addr += MLX5_ETH_VLAN_INLINE_HEADER_SIZE - sizeof(vlan);
	length -= MLX5_ETH_VLAN_INLINE_HEADER_SIZE - sizeof(vlan);

	wqe->dseg.byte_count = htonl(length);
	wqe->dseg.lkey = lkey;
	wqe->dseg.addr = htonll(addr);

	/* Write only data[0] which is the single element which changes.
	 * Other fields are already initialised in txq_alloc_elts. */
	wqe->ctrl.data[0] = htonl((txq->wqe_ci << 8) | MLX5_OPCODE_SEND);

	/* Increase the consumer index. */
	++txq->wqe_ci;
}

#else /* MLX5_PMD_MAX_INLINE == 0 */

static inline void
mlx5_wqe_write(struct ftxq *txq, volatile struct mlx5_wqe64 *wqe,
	       uintptr_t addr, uint32_t length, uint32_t lkey)
{
	wqe->eseg = (struct mlx5_wqe_eth_seg) {
		.rsvd0 = 0,
		.cs_flags = 0,
		.rsvd1 = 0,
		.mss = 0,
		.rsvd2 = 0,
	};

	wqe->eseg.inline_hdr_sz = htons(MLX5_ETH_INLINE_HEADER_SIZE);
	/* Copy the first 16 bytes into the inline header */
	rte_memcpy((void *)(uintptr_t)wqe->eseg.inline_hdr_start,
		   (void *)(uintptr_t)addr, MLX5_ETH_INLINE_HEADER_SIZE);
	addr += MLX5_ETH_INLINE_HEADER_SIZE;
	length -= MLX5_ETH_INLINE_HEADER_SIZE;

	wqe->dseg.byte_count = htonl(length);
	wqe->dseg.lkey = lkey;
	wqe->dseg.addr = htonll(addr);

	wqe->ctrl.data[0] = htonl((txq->wqe_ci << 8) | MLX5_OPCODE_SEND);
	wqe->ctrl.data[1] = htonl(txq->qp_num_8s | 4);
	if (unlikely(--txq->elts_comp == 0)) {
		wqe->ctrl.data[2] = htonl(8);
		txq->elts_comp = txq->elts_comp_cd_init;
	} else
		wqe->ctrl.data[2] = htonl(0);
	wqe->ctrl.data[3] = 0;

	/* Increase the consumer index. */
	++txq->wqe_ci;
}

static inline void
mlx5_wqe_write_vlan(struct ftxq *txq, volatile struct mlx5_wqe64 *wqe,
		    uintptr_t addr, uint32_t length, uint32_t lkey,
		    uint16_t vlan_tci)
{
	wqe->eseg = (struct mlx5_wqe_eth_seg) {
		.rsvd0 = 0,
		.cs_flags = 0,
		.rsvd1 = 0,
		.mss = 0,
		.rsvd2 = 0,
	};

	uint32_t vlan = htonl(0x81000000 | vlan_tci);

	wqe->eseg.inline_hdr_sz = htons(MLX5_ETH_VLAN_INLINE_HEADER_SIZE);
	/*
	 * Copy 12 bytes of source & destination MAC address.
	 * Copy 4 bytes of vlan.
	 * Copy 2 bytes of ether type.
	 */
	rte_memcpy((void *)(uintptr_t)wqe->eseg.inline_hdr_start,
		   (void *)(uintptr_t)addr, 12);
	rte_memcpy((void *)((uintptr_t)wqe->eseg.inline_hdr_start + 12),
		   &vlan, sizeof(vlan));
	rte_memcpy((void *)((uintptr_t)wqe->eseg.inline_hdr_start + 16),
		   (void *)((uintptr_t)addr + 12), 2);

	addr += MLX5_ETH_VLAN_INLINE_HEADER_SIZE - sizeof(vlan);
	length -= MLX5_ETH_VLAN_INLINE_HEADER_SIZE - sizeof(vlan);

	wqe->dseg.byte_count = htonl(length);
	wqe->dseg.lkey = lkey;
	wqe->dseg.addr = htonll(addr);

	wqe->ctrl.data[0] = htonl((txq->wqe_ci << 8) | MLX5_OPCODE_SEND);
	wqe->ctrl.data[1] = htonl(txq->qp_num_8s | 4);
	if (unlikely(--txq->elts_comp == 0)) {
		wqe->ctrl.data[2] = htonl(8);
		txq->elts_comp = txq->elts_comp_cd_init;
	} else
		wqe->ctrl.data[2] = htonl(0);
	wqe->ctrl.data[3] = 0;

	/* Increase the consumer index. */
	++txq->wqe_ci;
}

static inline void
mlx5_wqe_write_inline(struct ftxq *txq, volatile struct mlx5_wqe64 *wqe,
		      uintptr_t addr, uint32_t length)
{
	uint32_t size;
	uint32_t wqes_cnt = 1;
	volatile struct mlx5_wqe64_inl *inl_wqe =
		(volatile struct mlx5_wqe64_inl *)wqe;

	wqe->eseg = (struct mlx5_wqe_eth_seg) {
		.rsvd0 = 0,
		.cs_flags = 0,
		.rsvd1 = 0,
		.mss = 0,
		.rsvd2 = 0,
	};

	wqe->eseg.inline_hdr_sz = htons(MLX5_ETH_INLINE_HEADER_SIZE);
	/* Copy the first 16 bytes into the inline header */
	rte_memcpy((void *)(uintptr_t)wqe->eseg.inline_hdr_start,
		   (void *)(uintptr_t)addr,
		   MLX5_ETH_INLINE_HEADER_SIZE);
	addr += MLX5_ETH_INLINE_HEADER_SIZE;
	length -= MLX5_ETH_INLINE_HEADER_SIZE;

	size = (sizeof(wqe->ctrl.ctrl) +
		sizeof(wqe->eseg) +
		sizeof(wqe->dseg.byte_count) +
		length + 15) / 16;

	wqe->dseg.byte_count = htonl(length | MLX5_INLINE_SEG);
	rte_memcpy((void *)(uintptr_t)&inl_wqe->wqe.data[MLX5_WQE64_INL_DATA_OFFSET],
		   (void *)addr, MLX5_WQE64_INL_DATA);
	addr += MLX5_WQE64_INL_DATA;
	length -= MLX5_WQE64_INL_DATA;

	while (length) {
		volatile struct mlx5_wqe64_inl *wqe_next =
			(volatile struct mlx5_wqe64_inl *)
			&(*txq->wqes)[(txq->wqe_ci + wqes_cnt) &
				      (txq->wqe_cnt - 1)];
		uint32_t copy_bytes = (length > sizeof(*wqe)) ?
				      sizeof(*wqe) :
				      length;

		rte_mov64((uint8_t*)(uintptr_t)&wqe_next->wqe.data,
			  (uint8_t*)addr);
		addr += copy_bytes;
		length -= copy_bytes;
		wqes_cnt++;
	}

	wqe->ctrl.data[0] = htonl((txq->wqe_ci << 8) | MLX5_OPCODE_SEND);
	wqe->ctrl.data[1] = htonl(txq->qp_num_8s | (size & 0x3f));
	if (unlikely(--txq->elts_comp == 0)) {
		wqe->ctrl.data[2] = htonl(8);
		txq->elts_comp = txq->elts_comp_cd_init;
	} else
		wqe->ctrl.data[2] = htonl(0);
	wqe->ctrl.data[3] = 0;

	/* Increase the consumer index. */
	txq->wqe_ci += wqes_cnt;
}

static inline void
mlx5_wqe_write_inline_vlan(struct ftxq *txq, volatile struct mlx5_wqe64 *wqe,
			   uintptr_t addr, uint32_t length, uint16_t vlan_tci)
{
	uint32_t size;
	uint32_t wqes_cnt = 1;
	uint32_t vlan = htonl(0x81000000 | vlan_tci);
	volatile struct mlx5_wqe64_inl *inl_wqe =
		(volatile struct mlx5_wqe64_inl *)wqe;

	wqe->eseg = (struct mlx5_wqe_eth_seg) {
		.rsvd0 = 0,
		.cs_flags = 0,
		.rsvd1 = 0,
		.mss = 0,
		.rsvd2 = 0,
	};

	wqe->eseg.inline_hdr_sz = htons(MLX5_ETH_VLAN_INLINE_HEADER_SIZE);
	/*
	 * Copy 12 bytes of source & destination MAC address.
	 * Copy 4 bytes of vlan.
	 * Copy 2 bytes of ether type.
	 */
	rte_memcpy((uint8_t*)(uintptr_t)wqe->eseg.inline_hdr_start,
		   (uint8_t*)addr, 12);
	rte_memcpy((uint8_t*)(uintptr_t)wqe->eseg.inline_hdr_start + 12,
		   &vlan, sizeof(vlan));
	rte_memcpy((uint8_t*)(uintptr_t)wqe->eseg.inline_hdr_start + 16,
		   ((uint8_t*)addr + 12), 2);

	addr += MLX5_ETH_VLAN_INLINE_HEADER_SIZE - sizeof(vlan);
	length -= MLX5_ETH_VLAN_INLINE_HEADER_SIZE - sizeof(vlan);

	size = (sizeof(wqe->ctrl.ctrl) +
		sizeof(wqe->eseg) +
		sizeof(wqe->dseg.byte_count) +
		length + 15) / 16;

	wqe->dseg.byte_count = htonl(length | MLX5_INLINE_SEG);
	rte_memcpy((void *)(uintptr_t)&inl_wqe->wqe.data[MLX5_WQE64_INL_DATA_OFFSET],
		   (void *)addr, MLX5_WQE64_INL_DATA);
	addr += MLX5_WQE64_INL_DATA;
	length -= MLX5_WQE64_INL_DATA;

	while (length) {
		volatile struct mlx5_wqe64_inl *wqe_next =
			(volatile struct mlx5_wqe64_inl *)
			&(*txq->wqes)[(txq->wqe_ci + wqes_cnt) &
				      (txq->wqe_cnt - 1)];
		uint32_t copy_bytes = (length > sizeof(*wqe)) ?
				      sizeof(*wqe) :
				      length;

		rte_mov64((uint8_t*)(uintptr_t)wqe_next->wqe.data,
			  (uint8_t*)addr);
		addr += copy_bytes;
		length -= copy_bytes;
		wqes_cnt++;
	}

	wqe->ctrl.data[0] = htonl((txq->wqe_ci << 8) | MLX5_OPCODE_SEND);
	wqe->ctrl.data[1] = htonl(txq->qp_num_8s | (size & 0x3f));
	if (unlikely(--txq->elts_comp == 0)) {
		wqe->ctrl.data[2] = htonl(8);
		txq->elts_comp = txq->elts_comp_cd_init;
	} else
		wqe->ctrl.data[2] = htonl(0);
	wqe->ctrl.data[3] = 0;

	/* Increase the consumer index. */
	txq->wqe_ci += wqes_cnt;
}
#endif /* MLX5_PMD_MAX_INLINE == 0 */

static inline void
mlx5_tx_dbrec(struct ftxq *txq) {
	uint8_t *dst = (uint8_t *)((uintptr_t)txq->bf_reg + txq->bf_offset);
	uint32_t data[4] = {
		htonl((txq->wqe_ci << 8) | MLX5_OPCODE_SEND),
		htonl(txq->qp_num_8s),
		0,
		0};
	rte_wmb();
	*txq->qp_db = htonl(txq->wqe_ci);
	/* This wc_wmb ensures ordering between DB record and BF copy */
	rte_wmb();
	rte_mov16(dst, (uint8_t*)data);
	txq->bf_offset ^= txq->bf_buf_size;
}

static inline void __attribute__((always_inline))
tx_prefetch_cqe(struct ftxq *txq, uint16_t ci) {
	volatile struct mlx5_cqe64 *cqe;

	cqe = &(*txq->cqes)[ci & (txq->elts_n - 1)];
	rte_prefetch0(cqe);
}

/**
 * DPDK callback for TX.
 *
 * @param dpdk_txq
 *   Generic pointer to TX queue structure.
 * @param[in] pkts
 *   Packets to transmit.
 * @param pkts_n
 *   Number of packets in array.
 *
 * @return
 *   Number of packets successfully transmitted (<= pkts_n).
 */
uint16_t
mlx5_tx_burst(void *dpdk_txq, struct rte_mbuf **pkts, uint16_t pkts_n)
{
	struct ftxq *txq = (struct ftxq *)dpdk_txq;
	uint16_t elts_head = txq->elts_head;
	const unsigned int elts_n = txq->elts_n;
	unsigned int i;
	unsigned int max;
	volatile struct mlx5_wqe64 *wqe;
	struct rte_mbuf *buf = pkts[0];

	/* Prefetch first packet cacheline. */
	tx_prefetch_cqe(txq, txq->cq_ci);
	tx_prefetch_cqe(txq, txq->cq_ci + 1);
	rte_prefetch0(buf);
	/* Start processing. */
	txq_complete(txq);
	max = (elts_n - (elts_head - txq->elts_tail));
	if (max > elts_n)
		max -= elts_n;
	assert(max >= 1);
	assert(max <= elts_n);
	/* Always leave one free entry in the ring. */
	--max;
	if (max == 0)
		return 0;
	if (max > pkts_n)
		max = pkts_n;
	for (i = 0; (i != max); ++i) {
		struct rte_mbuf *buf_next = pkts[i + 1];
		unsigned int elts_head_next = (elts_head + 1) & (elts_n - 1);
#ifdef MLX5_PMD_SOFT_COUNTERS
		unsigned int sent_size = 0;
#endif
		uintptr_t addr;
		uint32_t length;
		uint32_t lkey;
		uintptr_t buf_next_addr;

		wqe = &(*txq->wqes)[txq->wqe_ci &  (txq->wqe_cnt - 1)];
		rte_prefetch0(wqe);
		if (i + 1 < max)
			rte_prefetch0(buf_next);
		/* Should we enable HW CKSUM offload */
		if (buf->ol_flags &
		    (PKT_TX_IP_CKSUM | PKT_TX_TCP_CKSUM | PKT_TX_UDP_CKSUM)) {
			wqe->eseg.cs_flags = MLX5_ETH_WQE_L3_CSUM |
				MLX5_ETH_WQE_L4_CSUM;
#if 0 /* Currently IBV_EXP_QP_BURST_TUNNEL is not used anywhere else. */
			/* HW does not support checksum offloads at arbitrary
			 * offsets but automatically recognizes the packet
			 * type. For inner L3/L4 checksums, only VXLAN (UDP)
			 * tunnels are currently supported. */
			if (RTE_ETH_IS_TUNNEL_PKT(buf->packet_type))
				send_flags |= IBV_EXP_QP_BURST_TUNNEL;
#endif
		}
		/* Retrieve buffer information. */
		addr = rte_pktmbuf_mtod(buf, uintptr_t);
		length = DATA_LEN(buf);
		/* Update element. */
		(*txq->elts)[elts_head] = buf;
		/* Prefetch next buffer data. */
		if (i + 1 < max) {
			buf_next_addr =
				rte_pktmbuf_mtod(buf_next, uintptr_t);
			rte_prefetch0((volatile void *)
				      (uintptr_t)buf_next_addr);
		}
#if MLX5_PMD_MAX_INLINE > 0
		if (length <= MLX5_PMD_MAX_INLINE)
			if (buf->ol_flags & PKT_TX_VLAN_PKT)
				mlx5_wqe_write_inline_vlan(txq, wqe,
							   addr, length,
							   buf->vlan_tci);
			else
				mlx5_wqe_write_inline(txq, wqe, addr, length);
		else
#endif
		{
			/* Retrieve Memory Region key for this memory pool. */
			lkey = txq_mp2mr(txq, txq_mb2mp(buf));
			if (buf->ol_flags & PKT_TX_VLAN_PKT)
				mlx5_wqe_write_vlan(txq, wqe, addr, length,
						    lkey, buf->vlan_tci);
			else
				mlx5_wqe_write(txq, wqe, addr, length, lkey);
		}
#ifdef MLX5_PMD_SOFT_COUNTERS
			sent_size += length;
#endif
		elts_head = elts_head_next;
		buf = buf_next;
#ifdef MLX5_PMD_SOFT_COUNTERS
		/* Increment sent bytes counter. */
		txq->stats.obytes += sent_size;
#endif
	}
	/* Take a shortcut if nothing must be sent. */
	if (unlikely(i == 0))
		return 0;
#ifdef MLX5_PMD_SOFT_COUNTERS
	/* Increment sent packets counter. */
	txq->stats.opackets += i;
#endif
	/* Ring QP doorbell. */
	mlx5_tx_dbrec(txq);
	txq->elts_head = elts_head;
	return i;
}

/**
 * Translate RX completion flags to packet type.
 *
 * @param  cqe
 *   The Pointer to the CQE
 *
 * @return
 *   Packet type for struct rte_mbuf.
 */
static inline uint32_t
rxq_cq_to_pkt_type(volatile struct mlx5_cqe64 *cqe)
{
	uint32_t pkt_type;

#if 0 /* Currently IBV_EXP_CQ_RX_TUNNEL_PACKET is not used. */
	if (flags & IBV_EXP_CQ_RX_TUNNEL_PACKET)
		pkt_type =
			TRANSPOSE(flags,
				  IBV_EXP_CQ_RX_OUTER_IPV4_PACKET,
				  RTE_PTYPE_L3_IPV4) |
			TRANSPOSE(flags,
				  IBV_EXP_CQ_RX_OUTER_IPV6_PACKET,
				  RTE_PTYPE_L3_IPV6) |
			TRANSPOSE(flags,
				  IBV_EXP_CQ_RX_IPV4_PACKET,
				  RTE_PTYPE_INNER_L3_IPV4) |
			TRANSPOSE(flags,
				  IBV_EXP_CQ_RX_IPV6_PACKET,
				  RTE_PTYPE_INNER_L3_IPV6);
	else
#endif
		pkt_type =
			TRANSPOSE(cqe->l4_hdr_type_etc,
				  MLX5_CQE_L3_HDR_TYPE_IPV6,
				  RTE_PTYPE_L3_IPV6) |
			TRANSPOSE(cqe->l4_hdr_type_etc,
				  MLX5_CQE_L3_HDR_TYPE_IPV4,
				  RTE_PTYPE_L3_IPV4);
	return pkt_type;
}

/**
 * Translate RX completion flags to offload flags.
 *
 * @param  cqe
 *   The Pointer to the CQE
 *
 * @return
 *   Offload flags (ol_flags) for struct rte_mbuf.
 */
static inline uint32_t
rxq_cq_to_ol_flags(volatile struct mlx5_cqe64 *cqe)
{
	uint32_t ol_flags = 0;
	uint8_t l3_hdr = (cqe->l4_hdr_type_etc) & MLX5_CQE_L3_HDR_TYPE_MASK;
	uint8_t l4_hdr = (cqe->l4_hdr_type_etc) & MLX5_CQE_L4_HDR_TYPE_MASK;

	if ((l3_hdr == MLX5_CQE_L3_HDR_TYPE_IPV4) ||
	    (l3_hdr == MLX5_CQE_L3_HDR_TYPE_IPV6))
		ol_flags |=
			(!(cqe->hds_ip_ext & MLX5_CQE_L3_OK) *
			 PKT_RX_IP_CKSUM_BAD);

	if ((l4_hdr == MLX5_CQE_L4_HDR_TYPE_TCP) ||
	    (l4_hdr == MLX5_CQE_L4_HDR_TYPE_TCP_EMP_ACK) ||
	    (l4_hdr == MLX5_CQE_L4_HDR_TYPE_TCP_ACK) ||
	    (l4_hdr == MLX5_CQE_L4_HDR_TYPE_UDP))
		ol_flags |=
			(!(cqe->hds_ip_ext & MLX5_CQE_L4_OK) *
			 PKT_RX_L4_CKSUM_BAD);
#if 0 /* Currently IBV_EXP_CQ_RX_TUNNEL_PACKET is not used. */
	/*
	 * PKT_RX_IP_CKSUM_BAD and PKT_RX_L4_CKSUM_BAD are used in place
	 * of PKT_RX_EIP_CKSUM_BAD because the latter is not functional
	 * (its value is 0).
	 */
	if ((flags & IBV_EXP_CQ_RX_TUNNEL_PACKET) && (rxq->csum_l2tun))
		ol_flags |=
			TRANSPOSE(~flags,
				  IBV_EXP_CQ_RX_OUTER_IP_CSUM_OK,
				  PKT_RX_IP_CKSUM_BAD) |
			TRANSPOSE(~flags,
				  IBV_EXP_CQ_RX_OUTER_TCP_UDP_CSUM_OK,
				  PKT_RX_L4_CKSUM_BAD);
#endif
	return ol_flags;
}

static inline int
mlx5_rx_poll_len(struct frxq *rxq, volatile struct mlx5_cqe64 *cqe,
		 uint16_t cqe_cnt)
{
	struct rxq_zip *zip = &rxq->zip;
	uint16_t cqe_n = cqe_cnt + 1;
	int len = 0;

	/* Process the compressed data present in the CQE and its mini arrays. */
	if (likely(zip->ai)) {
		volatile struct mlx5_mini_cqe8 (*mc)[8] =
			(volatile struct mlx5_mini_cqe8 (*)[8])
			(uintptr_t)&(*rxq->cqes)[zip->ca & cqe_cnt];

		len = ntohl((*mc)[zip->ai & 7].byte_cnt);
		if ((++zip->ai & 7) == 0) {
			/* Increase the consumer index to jump the number of
			 * CQE consumed.  The hardware leave holes in CQ ring
			 * for the software. */
			zip->ca = zip->na;
			zip->na += 8;
		}

		if (unlikely(rxq->zip.ai == rxq->zip.cqe_cnt)) {
			uint16_t idx = rxq->cq_ci;
			uint16_t end = zip->cq_ci;

			while (idx != end) {
				(*rxq->cqes)[idx & cqe_cnt].op_own =
					MLX5_CQE_INVALIDATE;
				++idx;
			}
			rxq->cq_ci = zip->cq_ci;
			zip->ai = 0;
		}
	/* No compressed data, get the next CQE and verify if it compressed. */
	} else {
		int ret;
		int8_t op_own;

		ret = check_cqe64(cqe, cqe_n, rxq->cq_ci);
		if (unlikely(ret == 1))
			return 0;
		++rxq->cq_ci;
		op_own = cqe->op_own;

		if (MLX5_CQE_FORMAT(op_own) == MLX5_COMPRESSED) {
			volatile struct mlx5_mini_cqe8 (*mc)[8] =
				(volatile struct mlx5_mini_cqe8 (*)[8])
				(uintptr_t)&(*rxq->cqes)[rxq->cq_ci &
							 cqe_cnt];

			/* Update big endian fields to little endian ones. */
			zip->cqe_cnt = ntohl(cqe->byte_cnt);

			/* Current mini array position is the one returned by
			 * check_cqe64() which increased by itself the
			 * consumer index.
			 *
			 * The next position if available of the mini array
			 * will be in 7 position from the current array
			 * position.  It is a special case, after this one,
			 * the next mini array will 8 CQEs after.
			 */
			zip->ca = rxq->cq_ci & cqe_cnt;
			zip->na = zip->ca + 7;

			/* Compute the next non compressed CQE. */
			--rxq->cq_ci;
			zip->cq_ci = rxq->cq_ci + zip->cqe_cnt;

			/* Get the packet size and return it. */
			len = ntohl((*mc)[0].byte_cnt);
			zip->ai = 1;
		} else
			len = ntohl(cqe->byte_cnt);
	}

	return len;
}

/**
 * DPDK callback for RX.
 *
 * The following function is the same as mlx5_rx_burst_sp(), except it doesn't
 * manage scattered packets. Improves performance when MRU is lower than the
 * size of the first segment.
 *
 * @param dpdk_rxq
 *   Generic pointer to RX queue structure.
 * @param[out] pkts
 *   Array to store received packets.
 * @param pkts_n
 *   Maximum number of packets in array.
 *
 * @return
 *   Number of packets successfully received (<= pkts_n).
 */
uint16_t
mlx5_rx_burst(void *dpdk_rxq, struct rte_mbuf **pkts, uint16_t pkts_n)
{
	struct frxq *rxq = dpdk_rxq;
	unsigned int pkts_ret = 0;
	unsigned int i;
	unsigned int elts_n = rxq->elts_n;
	unsigned int wqe_cnt = elts_n - 1;
	const unsigned int cqe_cnt = rxq->cqe_cnt;
	uint16_t idx = rxq->idx;

	for (i = 0; (i != pkts_n); ++i) {
		struct rte_mbuf *rep;
		struct rte_mbuf *pkt;
		unsigned int len;
		volatile struct mlx5_wqe_data_seg *wqe =
			&(*rxq->wqes)[idx & wqe_cnt];
		volatile struct mlx5_cqe64 *cqe =
			&(*rxq->cqes)[rxq->cq_ci & cqe_cnt];

		pkt = (*rxq->elts)[idx];
		rte_prefetch0(cqe);
		rep = __rte_mbuf_raw_alloc(rxq->mp);
		if (unlikely(rep == NULL)) {
			wqe->addr = htonll((uintptr_t)pkt->buf_addr +
					   RTE_PKTMBUF_HEADROOM);
			idx = (idx + 1) & (elts_n - 1);
			continue;
		}
		SET_DATA_OFF(rep, RTE_PKTMBUF_HEADROOM);
		NB_SEGS(rep) = 1;
		PORT(rep) = rxq->port_id;
		NEXT(rep) = NULL;
		len = mlx5_rx_poll_len(rxq, cqe, cqe_cnt);
		if (unlikely(len == 0)) {
			if (rep)
				__rte_mbuf_raw_free(rep);
			break;
		}
		assert(len >= (unsigned int)(rxq->crc_present << 2));
		/* Fill NIC descriptor with the new buffer.  The lkey and size
		 * of the buffers are already known, only the buffer address
		 * changes. */
		wqe->addr = htonll((uintptr_t)rep->buf_addr +
				   RTE_PKTMBUF_HEADROOM);
		(*rxq->elts)[idx] = rep;
		/* Update pkt information. */
		if (rxq->csum | rxq->vlan_strip | rxq->crc_present) {
			if (rxq->csum) {
				pkt->packet_type =
					rxq_cq_to_pkt_type(cqe);
				pkt->ol_flags =
					rxq_cq_to_ol_flags(cqe);
			}
			if (cqe->l4_hdr_type_etc & MLX5_CQE_VLAN_STRIPPED) {
				pkt->ol_flags |= PKT_RX_VLAN_PKT;
				pkt->vlan_tci = ntohs(cqe->vlan_info);
			}
			if (rxq->crc_present)
				len -= CRC_SIZE;
		}
		PKT_LEN(pkt) = len;
		DATA_LEN(pkt) = len;
#ifdef MLX5_PMD_SOFT_COUNTERS
		/* Increment bytes counter. */
		rxq->stats.ibytes += len;
#endif
		/* Return packet. */
		*(pkts++) = pkt;
		++pkts_ret;
		idx = (idx + 1) & (elts_n - 1);
	}
	if (unlikely(i == 0))
		return 0;
	/* Repost WRs. */
#ifdef DEBUG_RECV
	DEBUG("%p: reposting %u WRs", (void *)rxq, i);
#endif
	/* Update the consumer index. */
	rxq->rq_ci += i;
	rte_wmb();
	*rxq->cq_db = htonl(rxq->cq_ci);
	rte_wmb();
	*rxq->rq_db = htonl(rxq->rq_ci);
	rxq->idx = idx;
#ifdef MLX5_PMD_SOFT_COUNTERS
	/* Increment bytes counter. */
	rxq->stats.ipackets += pkts_ret;
#endif

	return pkts_ret;
}

/**
 * Dummy DPDK callback for TX.
 *
 * This function is used to temporarily replace the real callback during
 * unsafe control operations on the queue, or in case of error.
 *
 * @param dpdk_txq
 *   Generic pointer to TX queue structure.
 * @param[in] pkts
 *   Packets to transmit.
 * @param pkts_n
 *   Number of packets in array.
 *
 * @return
 *   Number of packets successfully transmitted (<= pkts_n).
 */
uint16_t
removed_tx_burst(void *dpdk_txq, struct rte_mbuf **pkts, uint16_t pkts_n)
{
	(void)dpdk_txq;
	(void)pkts;
	(void)pkts_n;
	return 0;
}

/**
 * Dummy DPDK callback for RX.
 *
 * This function is used to temporarily replace the real callback during
 * unsafe control operations on the queue, or in case of error.
 *
 * @param dpdk_rxq
 *   Generic pointer to RX queue structure.
 * @param[out] pkts
 *   Array to store received packets.
 * @param pkts_n
 *   Maximum number of packets in array.
 *
 * @return
 *   Number of packets successfully received (<= pkts_n).
 */
uint16_t
removed_rx_burst(void *dpdk_rxq, struct rte_mbuf **pkts, uint16_t pkts_n)
{
	(void)dpdk_rxq;
	(void)pkts;
	(void)pkts_n;
	return 0;
}
