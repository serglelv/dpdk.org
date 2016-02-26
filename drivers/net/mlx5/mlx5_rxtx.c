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
#ifdef PEDANTIC
#pragma GCC diagnostic error "-pedantic"
#endif

#include "mlx5.h"
#include "mlx5_utils.h"
#include "mlx5_rxtx.h"
#include "mlx5_autoconf.h"
#include "mlx5_defs.h"

static inline volatile struct mlx5_cqe64 *
get_cqe64(volatile struct mlx5_cqe64 cqes[],
	  unsigned int cqes_n, uint16_t *ci)
	  __attribute__((always_inline));

static inline int
rx_poll_len(struct frxq *rxq) __attribute__((always_inline));

static volatile struct mlx5_cqe64 *
get_cqe64(volatile struct mlx5_cqe64 cqes[],
	  unsigned int cqes_n, uint16_t *ci)
{
	volatile struct mlx5_cqe64 *cqe;
	uint16_t idx = *ci;
	uint8_t op_own;

	cqe = &cqes[idx & (cqes_n - 1)];
	op_own = cqe->op_own;

	if (unlikely((op_own & MLX5_CQE_OWNER_MASK) == !(idx & cqes_n))) {
		return NULL;
	} else if (unlikely(op_own & 0x80)) {
		switch (op_own >> 4) {
			case MLX5_CQE_INVALID:
				return NULL; /* No CQE */
			case MLX5_CQE_REQ_ERR:
				return cqe;
			case MLX5_CQE_RESP_ERR:
				++(*ci);
				return NULL;
			default:
				return NULL;
		}
	}

	if (cqe) {
		*ci = idx + 1;
		return cqe;
	}

	return NULL;
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
 *
 * @return
 *   0 on success, -1 on failure.
 */
static int
txq_complete(struct ftxq *txq)
{
	unsigned int elts_tail = txq->elts_tail;
	unsigned int elts_free = txq->elts_tail;
	const unsigned int elts_n = txq->elts_n;
	volatile struct mlx5_cqe64 *cqe;
	unsigned int wcs_n = 0;
	unsigned int max = txq->elts_comp_npr;

	while (wcs_n != max) {
		cqe = get_cqe64((*txq->cqes), elts_n, &txq->cq_ci);
		if (cqe)
			++wcs_n;
		else
			break;
	}
	if (unlikely(wcs_n == 0))
		return 0;

	elts_tail += wcs_n * txq->elts_comp_cd_init;
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

	txq->elts_tail = elts_tail;

	/* Update the consumer index. */
	rte_wmb();
	*txq->cq_db = htonl(txq->cq_ci);

	return 0;
}

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
			assert(txq->mp2mr[i].mr->lkey == txq->mp2mr[i].lkey);
			lkey = txq->mp2mr[i].lkey;
			break;
		}
	}
	if (unlikely(lkey == (uint32_t)-1))
		lkey = txq_mp2mr_reg(txq, mp, i);

	return lkey;
}

static inline void
mlx5_wqe_write(struct ftxq *txq, volatile struct mlx5_wqe64 *wqe,
	       uintptr_t addr, uint32_t length, uint32_t lkey,
	       uint32_t send_flags)
{

	if (send_flags & IBV_EXP_QP_BURST_IP_CSUM)
		wqe->eseg.cs_flags = MLX5_ETH_WQE_L3_CSUM |
			MLX5_ETH_WQE_L4_CSUM;

	/* Copy the first 16 bytes into the inline header */
	memcpy((void *)(uintptr_t)wqe->eseg.inline_hdr_start,
	       (void *)(uintptr_t)addr,
	       MLX5_ETH_INLINE_HEADER_SIZE);
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

/*
 * Avoid using memcpy() to copy to BlueFlame page, since memcpy()
 * implementations may use move-string-buffer assembler instructions,
 * which do not guarantee order of copying.
 */
#if defined(__x86_64__)
#define COPY_64B_NT(dst, src)		\
	__asm__ __volatile__ (		\
	" movdqa   (%1),%%xmm0\n"	\
	" movdqa 16(%1),%%xmm1\n"	\
	" movdqa 32(%1),%%xmm2\n"	\
	" movdqa 48(%1),%%xmm3\n"	\
	" movntdq %%xmm0,   (%0)\n"	\
	" movntdq %%xmm1, 16(%0)\n"	\
	" movntdq %%xmm2, 32(%0)\n"	\
	" movntdq %%xmm3, 48(%0)\n"	\
	: : "r" (dst), "r" (src) : "memory");	\
	dst += 8;			\
	src += 8
#else
#define COPY_64B_NT(dst, src)	\
	*dst++ = *src++;	\
	*dst++ = *src++;	\
	*dst++ = *src++;	\
	*dst++ = *src++;	\
	*dst++ = *src++;	\
	*dst++ = *src++;	\
	*dst++ = *src++;	\
	*dst++ = *src++

#endif

static inline void
mlx5_tx_dbrec(struct ftxq *txq, volatile struct mlx5_wqe64 *wqe) {
	volatile uintptr_t *dst = (volatile uintptr_t *)
		((uintptr_t)txq->bf_reg + txq->bf_offset);

	rte_wmb();
	*txq->qp_db = htonl(txq->wqe_ci);
	/* This wc_wmb ensures ordering between DB record and BF copy */
	rte_wmb();
	COPY_64B_NT(dst, wqe);
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
	unsigned int elts_head = txq->elts_head;
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
		uint32_t send_flags = 0;

		wqe = &(*txq->wqes)[elts_head];
		rte_prefetch0(wqe);
		if (i + 1 < max)
			rte_prefetch0(buf_next);
		/* Should we enable HW CKSUM offload */
		if (buf->ol_flags &
		    (PKT_TX_IP_CKSUM | PKT_TX_TCP_CKSUM | PKT_TX_UDP_CKSUM)) {
			send_flags |= IBV_EXP_QP_BURST_IP_CSUM;
			/* HW does not support checksum offloads at arbitrary
			 * offsets but automatically recognizes the packet
			 * type. For inner L3/L4 checksums, only VXLAN (UDP)
			 * tunnels are currently supported. */
			if (RTE_ETH_IS_TUNNEL_PKT(buf->packet_type))
				send_flags |= IBV_EXP_QP_BURST_TUNNEL;
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
		/* Retrieve Memory Region key for this memory pool. */
		lkey = txq_mp2mr(txq, txq_mb2mp(buf));
		mlx5_wqe_write(txq, wqe, addr, length, lkey, send_flags);
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
	mlx5_tx_dbrec(txq, wqe);
	txq->elts_head = elts_head;
	return i;
}

/**
 * Translate RX completion flags to packet type.
 *
 * @param flags
 *   RX completion flags returned by poll_length_flags().
 *
 * @return
 *   Packet type for struct rte_mbuf.
 */
static inline uint32_t
rxq_cq_to_pkt_type(uint32_t flags)
{
	uint32_t pkt_type;

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
		pkt_type =
			TRANSPOSE(flags,
				  IBV_EXP_CQ_RX_IPV4_PACKET,
				  RTE_PTYPE_L3_IPV4) |
			TRANSPOSE(flags,
				  IBV_EXP_CQ_RX_IPV6_PACKET,
				  RTE_PTYPE_L3_IPV6);
	return pkt_type;
}

/**
 * Get the size of the received packet.
 *
 * @param  rxq
 *   The RX queue.
 *
 * @return
 *   The packet size in bytes
 */
static inline int __attribute__((always_inline))
rx_poll_len(struct frxq *rxq)
{
	volatile struct mlx5_cqe64 *cqe;

	cqe = get_cqe64(*rxq->cqes, rxq->elts_n, &rxq->cq_ci);
	if (cqe)
		return ntohl(cqe->byte_cnt);

	return 0;
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
	uint16_t idx = rxq->idx;

	for (i = 0; (i != pkts_n); ++i) {
		struct rte_mbuf *rep;
		struct rte_mbuf *pkt;
		unsigned int len;
		volatile struct mlx5_wqe_data_seg *wqe = &(*rxq->wqes)[idx & wqe_cnt];

		pkt = (*rxq->elts)[idx];
		rte_prefetch0(&(*rxq->cqes)[rxq->cq_ci & wqe_cnt]);
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
		len = rx_poll_len(rxq);
		if (unlikely(len == 0)) {
			if (rep)
				__rte_mbuf_raw_free(rep);
			break;
		}
		/* Fill NIC descriptor with the new buffer.  The lkey and size
		 * of the buffers are already known, only the buffer address
		 * changes. */
		wqe->addr = htonll((uintptr_t)rep->buf_addr +
				   RTE_PKTMBUF_HEADROOM);
		(*rxq->elts)[idx] = rep;
		/* Update pkt information. */
		PKT_LEN(pkt) = len;
		DATA_LEN(pkt) = len;

#ifdef MLX5_PMD_SOFT_COUNTERS
		/* Increment bytes counter. */
		rxq->stats.ibytes += i;
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
