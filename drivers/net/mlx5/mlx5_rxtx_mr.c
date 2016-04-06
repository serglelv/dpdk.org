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

/* Verbs header. */
/* ISO C doesn't support unnamed structs/unions, disabling -pedantic. */
#ifdef PEDANTIC
#pragma GCC diagnostic ignored "-pedantic"
#endif
#include <infiniband/verbs.h>
#ifdef PEDANTIC
#pragma GCC diagnostic error "-pedantic"
#endif

/* DPDK headers don't like -pedantic. */
#ifdef PEDANTIC
#pragma GCC diagnostic ignored "-pedantic"
#endif
#include <rte_mempool.h>
#ifdef PEDANTIC
#pragma GCC diagnostic error "-pedantic"
#endif

#include "mlx5.h"
#include "mlx5_rxtx.h"

/**
 * Register mempool as a memory region.
 *
 * @param pd
 *   Pointer to protection domain.
 * @param mp
 *   Pointer to memory pool.
 *
 * @return
 *   Memory region pointer, NULL in case of error.
 */
struct ibv_mr *
mlx5_mp2mr(struct ibv_pd *pd, const struct rte_mempool *mp)
{
	const struct rte_memseg *ms = rte_eal_get_physmem_layout();
	uintptr_t start = mp->elt_va_start;
	uintptr_t end = mp->elt_va_end;
	unsigned int i;

	DEBUG("mempool %p area start=%p end=%p size=%zu",
	      (const void *)mp, (void *)start, (void *)end,
	      (size_t)(end - start));
	/* Round start and end to page boundary if found in memory segments. */
	for (i = 0; (i < RTE_MAX_MEMSEG) && (ms[i].addr != NULL); ++i) {
		uintptr_t addr = (uintptr_t)ms[i].addr;
		size_t len = ms[i].len;
		unsigned int align = ms[i].hugepage_sz;

		if ((start > addr) && (start < addr + len))
			start = RTE_ALIGN_FLOOR(start, align);
		if ((end > addr) && (end < addr + len))
			end = RTE_ALIGN_CEIL(end, align);
	}
	DEBUG("mempool %p using start=%p end=%p size=%zu for MR",
	      (const void *)mp, (void *)start, (void *)end,
	      (size_t)(end - start));
	return ibv_reg_mr(pd,
			  (void *)start,
			  end - start,
			  IBV_ACCESS_LOCAL_WRITE);
}

/**
 * Register a Memory Region (MR) <-> Memory Pool (MP) association from
 * txq->mp2mr[].   Add MP to txq->mp2mr[]. If mp2mr[] is full, remove an entry
 * first.
 *
 * @param txq
 *   Pointer to TX queue structure.
 * @param[in] mp
 *   Memory Pool for which a Memory Region lkey must be returned.
 * @param idx
 *   The index of the next available entry (idx > mp2mr_n is handled).
 *
 * @return
 *   mr->lkey on success, (uint32_t)-1 on failure.
 *
 * should only be called by txq_mp2mr().
 */
uint32_t
txq_mp2mr_reg(struct ftxq *txq, const struct rte_mempool *mp, unsigned int idx)
{
	struct ibv_mr *mr;
	struct txq *stxq = container_of(txq, struct txq, ftxq);

	/* Add a new entry, register MR first. */
	DEBUG("%p: discovered new memory pool \"%s\" (%p)",
	      (void *)txq, mp->name, (const void *)mp);
	mr = mlx5_mp2mr(stxq->priv->pd, mp);
	if (unlikely(mr == NULL)) {
		DEBUG("%p: unable to configure MR, ibv_reg_mr() failed.",
		      (void *)txq);
		return (uint32_t)-1;
	}
	if (unlikely(idx == RTE_DIM(txq->mp2mr))) {
		/* Table is full, remove oldest entry. */
		DEBUG("%p: MR <-> MP table full, dropping oldest entry.",
		      (void *)txq);
		--idx;
		claim_zero(ibv_dereg_mr(txq->mp2mr[0].mr));
		memmove(&txq->mp2mr[0], &txq->mp2mr[1],
			(sizeof(txq->mp2mr) - sizeof(txq->mp2mr[0])));
	}
	/* Store the new entry. */
	txq->mp2mr[idx].mp = mp;
	txq->mp2mr[idx].mr = mr;
	txq->mp2mr[idx].lkey = htonl(mr->lkey);
	DEBUG("%p: new MR lkey for MP \"%s\" (%p): 0x%08" PRIu32,
	      (void *)txq, mp->name, (const void *)mp, txq->mp2mr[idx].lkey);

	return txq->mp2mr[idx].lkey;
}

struct txq_mp2mr_mbuf_check_data {
	const struct rte_mempool *mp;
	int ret;
};

/**
 * Callback function for rte_mempool_obj_iter() to check whether a given
 * mempool object looks like a mbuf.
 *
 * @param[in, out] arg
 *   Context data (struct txq_mp2mr_mbuf_check_data). Contains mempool pointer
 *   and return value.
 * @param[in] start
 *   Object start address.
 * @param[in] end
 *   Object end address.
 * @param index
 *   Unused.
 *
 * @return
 *   Nonzero value when object is not a mbuf.
 */
static void
txq_mp2mr_mbuf_check(void *arg, void *start, void *end,
		     uint32_t index __rte_unused)
{
	struct txq_mp2mr_mbuf_check_data *data = arg;
	struct rte_mbuf *buf =
		(void *)((uintptr_t)start + data->mp->header_size);

	(void)index;
	/* Check whether mbuf structure fits element size and whether mempool
	 * pointer is valid. */
	if (((uintptr_t)end >= (uintptr_t)(buf + 1)) &&
	    (buf->pool == data->mp))
		data->ret = 0;
	else
		data->ret = -1;
}

/**
 * Iterator function for rte_mempool_walk() to register existing mempools and
 * fill the MP to MR cache of a TX queue.
 *
 * @param[in] mp
 *   Memory Pool to register.
 * @param *arg
 *   Pointer to TX queue structure.
 */
void
txq_mp2mr_iter(const struct rte_mempool *mp, void *arg)
{
	struct ftxq *txq = arg;
	struct txq_mp2mr_mbuf_check_data data = {
		.mp = mp,
		.ret = -1,
	};
	unsigned int i;

	/* Discard empty mempools. */
	if (mp->size == 0)
		return;
	/* Register mempool only if the first element looks like a mbuf. */
	rte_mempool_obj_iter((void *)mp->elt_va_start,
			     1,
			     mp->header_size + mp->elt_size + mp->trailer_size,
			     1,
			     mp->elt_pa,
			     mp->pg_num,
			     mp->pg_shift,
			     txq_mp2mr_mbuf_check,
			     &data);
	if (data.ret)
		return;

	for (i = 0; (i != RTE_DIM(txq->mp2mr)); ++i) {
		if (unlikely(txq->mp2mr[i].mp == NULL)) {
			/* Unknown MP, add a new MR for it. */
			break;
		}
		if (txq->mp2mr[i].mp == mp)
			return;
	}

	txq_mp2mr_reg(txq, mp, i);
}
