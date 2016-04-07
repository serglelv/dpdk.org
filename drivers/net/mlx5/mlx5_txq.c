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

#include <stddef.h>
#include <assert.h>
#include <errno.h>
#include <string.h>
#include <stdint.h>

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
#include <rte_mbuf.h>
#include <rte_malloc.h>
#include <rte_ethdev.h>
#include <rte_common.h>
#ifdef PEDANTIC
#pragma GCC diagnostic error "-pedantic"
#endif

#include "mlx5_utils.h"
#include "mlx5_defs.h"
#include "mlx5.h"
#include "mlx5_rxtx.h"
#include "mlx5_autoconf.h"
#include "mlx5_defs.h"

/**
 * Get the minimum number of Queues necessary to activate for inline feature.
 *
 * @return
 *   the number of queues to activate the inline feature.
 */
int
txq_min_queue_inline(void)
{
	return mlx5_getenv_int("MLX5_TXQ_MIN_QUEUE_INLINE");
}

/**
 * Allocate TX queue elements.
 *
 * @param txq
 *   Pointer to TX queue structure.
 * @param elts_n
 *   Number of elements to allocate.
 */
static void
txq_alloc_elts(struct txq *txq, unsigned int elts_n)
{
	unsigned int i;
	unsigned int comp = txq->ftxq.elts_comp_cd_init;

	for (i = 0; (i != elts_n); ++i)
		(*txq->ftxq.elts)[i] = NULL;

	for (i = 0; (i != txq->ftxq.wqe_cnt); ++i) {
		volatile struct mlx5_wqe64 *wqe = &(*txq->ftxq.wqes)[i];

		memset((void *)(uintptr_t)wqe, 0, sizeof(struct mlx5_wqe64));
		wqe->eseg.inline_hdr_sz = htons(MLX5_ETH_INLINE_HEADER_SIZE);
		wqe->ctrl.data[1] = htonl(txq->ftxq.qp_num_8s | 4);
		/* Store the completion request in the WQE. */
		if (--comp == 0) {
			wqe->ctrl.data[2] = htonl(8);
			comp = txq->ftxq.elts_comp_cd_init;
		}
		else
			wqe->ctrl.data[2] = 0;
	}
	DEBUG("%p: allocated and configured %u WRs", (void *)txq, elts_n);
	txq->ftxq.elts_head = 0;
	txq->ftxq.elts_tail = 0;
	txq->ftxq.elts_comp = txq->ftxq.elts_comp_cd_init;
}

/**
 * Free TX queue elements.
 *
 * @param txq
 *   Pointer to TX queue structure.
 */
static void
txq_free_elts(struct txq *txq)
{
	unsigned int i;
	unsigned int elts_n = txq->ftxq.elts_n;

	DEBUG("%p: freeing WRs", (void *)txq);
	txq->ftxq.elts_n = 0;
	for (i = 0; (i != elts_n); ++i) {
		if ((*txq->ftxq.elts)[i] == NULL)
			continue;
		rte_pktmbuf_free((*txq->ftxq.elts)[i]);
		(*txq->ftxq.elts)[i] = NULL;
	}
}

/**
 * Clean up a TX queue.
 *
 * Destroy objects, free allocated memory and reset the structure for reuse.
 *
 * @param txq
 *   Pointer to TX queue structure.
 */
void
txq_cleanup(struct txq *txq)
{
	struct ibv_exp_release_intf_params params;
	size_t i;

	DEBUG("cleaning up %p", (void *)txq);
	txq_free_elts(txq);
	if (txq->if_qp != NULL) {
		assert(txq->priv != NULL);
		assert(txq->priv->ctx != NULL);
		assert(txq->qp != NULL);
		params = (struct ibv_exp_release_intf_params){
			.comp_mask = 0,
		};
		claim_zero(ibv_exp_release_intf(txq->priv->ctx,
						txq->if_qp,
						&params));
	}
	if (txq->if_cq != NULL) {
		assert(txq->priv != NULL);
		assert(txq->priv->ctx != NULL);
		assert(txq->cq != NULL);
		params = (struct ibv_exp_release_intf_params){
			.comp_mask = 0,
		};
		claim_zero(ibv_exp_release_intf(txq->priv->ctx,
						txq->if_cq,
						&params));
	}
	if (txq->qp != NULL)
		claim_zero(ibv_destroy_qp(txq->qp));
	if (txq->cq != NULL)
		claim_zero(ibv_destroy_cq(txq->cq));
	if (txq->rd != NULL) {
		struct ibv_exp_destroy_res_domain_attr attr = {
			.comp_mask = 0,
		};

		assert(txq->priv != NULL);
		assert(txq->priv->ctx != NULL);
		claim_zero(ibv_exp_destroy_res_domain(txq->priv->ctx,
						      txq->rd,
						      &attr));
	}
	for (i = 0; (i != RTE_DIM(txq->ftxq.mp2mr)); ++i) {
		if (txq->ftxq.mp2mr[i].mp == NULL)
			break;
		assert(txq->ftxq.mp2mr[i].mr != NULL);
		claim_zero(ibv_dereg_mr(txq->ftxq.mp2mr[i].mr));
	}
	memset(txq, 0, sizeof(*txq));
}

/**
 *  Initialise fast Tx queue elements.
 *
 * @param txq
 *   Pointer to the receive queue.
 */
static inline void
txq_ftxq_setup(struct txq *tmpl, struct txq *txq)
{
	struct mlx5_qp *qp = to_mqp(tmpl->qp);
	struct ibv_cq *ibcq = tmpl->cq;
	struct mlx5_cq *cq = to_mxxx(cq, cq);

	tmpl->ftxq.cqe_cnt = ibcq->cqe;
	tmpl->ftxq.qp_num_8s = qp->ctrl_seg.qp_num << 8;
	tmpl->ftxq.wqes =
		(volatile struct mlx5_wqe64 (*)[])
		(uintptr_t)qp->gen_data.sqstart;
	tmpl->ftxq.wqe_cnt = qp->sq.wqe_cnt;
	tmpl->ftxq.qp_db = &qp->gen_data.db[MLX5_SND_DBR];
	tmpl->ftxq.bf_reg = qp->gen_data.bf->reg;
	tmpl->ftxq.bf_offset = qp->gen_data.bf->offset;
	tmpl->ftxq.bf_buf_size = qp->gen_data.bf->buf_size;
	tmpl->ftxq.cq_db = cq->dbrec;
	tmpl->ftxq.cqes =
		(volatile struct mlx5_cqe64 (*)[])
		(uintptr_t)cq->active_buf->buf;
	tmpl->ftxq.elts =
		(struct rte_mbuf *(*)[tmpl->ftxq.elts_n])
		((uintptr_t)txq + sizeof(*txq));
}

/**
 * Configure a TX queue.
 *
 * @param dev
 *   Pointer to Ethernet device structure.
 * @param txq
 *   Pointer to TX queue structure.
 * @param desc
 *   Number of descriptors to configure in queue.
 * @param socket
 *   NUMA socket on which memory must be allocated.
 * @param[in] conf
 *   Thresholds parameters.
 *
 * @return
 *   0 on success, errno value on failure.
 */
int
txq_setup(struct rte_eth_dev *dev, struct txq *txq, uint16_t desc,
	  unsigned int socket, const struct rte_eth_txconf *conf)
{
	struct priv *priv = mlx5_get_priv(dev);
	struct txq tmpl = {
		.priv = priv,
		.socket = socket
	};
	union {
		struct ibv_exp_query_intf_params params;
		struct ibv_exp_qp_init_attr init;
		struct ibv_exp_res_domain_init_attr rd;
		struct ibv_exp_cq_init_attr cq;
		struct ibv_exp_qp_attr mod;
		struct ibv_exp_cq_attr cq_attr;
	} attr;
	enum ibv_exp_query_intf_status status;
	int ret = 0;

	(void)conf; /* Thresholds configuration (ignored). */
	tmpl.ftxq.elts_n = desc;
	/* Request send completion every MLX5_PMD_TX_PER_COMP_REQ packets or
	 * at least 4 times per ring. */
	tmpl.ftxq.elts_comp_cd_init =
		((MLX5_PMD_TX_PER_COMP_REQ < (desc / 4)) ?
		 MLX5_PMD_TX_PER_COMP_REQ : (desc / 4));

	/* MRs will be registered in mp2mr[] later. */
	attr.rd = (struct ibv_exp_res_domain_init_attr){
		.comp_mask = (IBV_EXP_RES_DOMAIN_THREAD_MODEL |
			      IBV_EXP_RES_DOMAIN_MSG_MODEL),
		.thread_model = IBV_EXP_THREAD_SINGLE,
		.msg_model = IBV_EXP_MSG_HIGH_BW,
	};
	tmpl.rd = ibv_exp_create_res_domain(priv->ctx, &attr.rd);
	if (tmpl.rd == NULL) {
		ret = ENOMEM;
		ERROR("%p: RD creation failure: %s",
		      (void *)dev, strerror(ret));
		goto error;
	}
	attr.cq = (struct ibv_exp_cq_init_attr){
		.comp_mask = IBV_EXP_CQ_INIT_ATTR_RES_DOMAIN,
		.res_domain = tmpl.rd,
	};
	tmpl.cq = ibv_exp_create_cq(priv->ctx,
				    (desc / tmpl.ftxq.elts_comp_cd_init) - 1,
				    NULL, NULL, 0, &attr.cq);
	if (tmpl.cq == NULL) {
		ret = ENOMEM;
		ERROR("%p: CQ creation failure: %s",
		      (void *)dev, strerror(ret));
		goto error;
	}
	DEBUG("priv->device_attr.max_qp_wr is %d",
	      priv->device_attr.max_qp_wr);
	DEBUG("priv->device_attr.max_sge is %d",
	      priv->device_attr.max_sge);
	attr.init = (struct ibv_exp_qp_init_attr){
		/* CQ to be associated with the send queue. */
		.send_cq = tmpl.cq,
		/* CQ to be associated with the receive queue. */
		.recv_cq = tmpl.cq,
		.cap = {
			/* Max number of outstanding WRs. */
			.max_send_wr = ((priv->device_attr.max_qp_wr < desc) ?
					priv->device_attr.max_qp_wr :
					desc),
			/* Max number of scatter/gather elements in a WR. */
			.max_send_sge = 1,
		},
		.qp_type = IBV_QPT_RAW_PACKET,
		/* Do *NOT* enable this, completions events are managed per
		 * TX burst. */
		.sq_sig_all = 0,
		.pd = priv->pd,
		.res_domain = tmpl.rd,
		.comp_mask = (IBV_EXP_QP_INIT_ATTR_PD |
			      IBV_EXP_QP_INIT_ATTR_RES_DOMAIN),
	};
	if (MLX5_PMD_MAX_INLINE &&
	    (priv->txqs_n >= (unsigned int)txq_min_queue_inline()))
		attr.init.cap.max_inline_data = MLX5_PMD_MAX_INLINE;
	tmpl.qp = ibv_exp_create_qp(priv->ctx, &attr.init);
	if (tmpl.qp == NULL) {
		ret = (errno ? errno : EINVAL);
		ERROR("%p: QP creation failure: %s",
		      (void *)dev, strerror(ret));
		goto error;
	}
	attr.mod = (struct ibv_exp_qp_attr){
		/* Move the QP to this state. */
		.qp_state = IBV_QPS_INIT,
		/* Primary port number. */
		.port_num = priv->port
	};
	ret = ibv_exp_modify_qp(tmpl.qp, &attr.mod,
				(IBV_EXP_QP_STATE | IBV_EXP_QP_PORT));
	if (ret) {
		ERROR("%p: QP state to IBV_QPS_INIT failed: %s",
		      (void *)dev, strerror(ret));
		goto error;
	}
	txq_ftxq_setup(&tmpl, txq);
	txq_alloc_elts(&tmpl, desc);
	attr.mod = (struct ibv_exp_qp_attr){
		.qp_state = IBV_QPS_RTR
	};
	ret = ibv_exp_modify_qp(tmpl.qp, &attr.mod, IBV_EXP_QP_STATE);
	if (ret) {
		ERROR("%p: QP state to IBV_QPS_RTR failed: %s",
		      (void *)dev, strerror(ret));
		goto error;
	}
	attr.mod.qp_state = IBV_QPS_RTS;
	ret = ibv_exp_modify_qp(tmpl.qp, &attr.mod, IBV_EXP_QP_STATE);
	if (ret) {
		ERROR("%p: QP state to IBV_QPS_RTS failed: %s",
		      (void *)dev, strerror(ret));
		goto error;
	}
	attr.params = (struct ibv_exp_query_intf_params){
		.intf_scope = IBV_EXP_INTF_GLOBAL,
		.intf = IBV_EXP_INTF_CQ,
		.obj = tmpl.cq,
	};
	tmpl.if_cq = ibv_exp_query_intf(priv->ctx, &attr.params, &status);
	if (tmpl.if_cq == NULL) {
		ret = EINVAL;
		ERROR("%p: CQ interface family query failed with status %d",
		      (void *)dev, status);
		goto error;
	}
	attr.params = (struct ibv_exp_query_intf_params){
		.intf_scope = IBV_EXP_INTF_GLOBAL,
		.intf = IBV_EXP_INTF_QP_BURST,
		.obj = tmpl.qp,
#if defined(HAVE_EXP_QP_BURST_CREATE_ENABLE_MULTI_PACKET_SEND_WR)
		/* Multi packet send WR can only be used outside of VF. */
		.family_flags =
			(!priv->vf ?
			 IBV_EXP_QP_BURST_CREATE_ENABLE_MULTI_PACKET_SEND_WR :
			 0),
#endif
	};
	tmpl.if_qp = ibv_exp_query_intf(priv->ctx, &attr.params, &status);
	if (tmpl.if_qp == NULL) {
		ret = EINVAL;
		ERROR("%p: QP interface family query failed with status %d",
		      (void *)dev, status);
		goto error;
	}
	/* Clean up txq in case we're reinitializing it. */
	DEBUG("%p: cleaning-up old txq just in case", (void *)txq);
	txq_cleanup(txq);
	*txq = tmpl;
	DEBUG("%p: txq updated with %p", (void *)txq, (void *)&tmpl);
	/* Pre-register known mempools. */
	rte_mempool_walk(txq_mp2mr_iter, &txq->ftxq);
	assert(ret == 0);
	return 0;
error:
	txq_cleanup(&tmpl);
	assert(ret > 0);
	return ret;
}

/**
 * DPDK callback to configure a TX queue.
 *
 * @param dev
 *   Pointer to Ethernet device structure.
 * @param idx
 *   TX queue index.
 * @param desc
 *   Number of descriptors to configure in queue.
 * @param socket
 *   NUMA socket on which memory must be allocated.
 * @param[in] conf
 *   Thresholds parameters.
 *
 * @return
 *   0 on success, negative errno value on failure.
 */
int
mlx5_tx_queue_setup(struct rte_eth_dev *dev, uint16_t idx, uint16_t desc,
		    unsigned int socket, const struct rte_eth_txconf *conf)
{
	struct priv *priv = dev->data->dev_private;
	struct ftxq *ftxq = (*priv->txqs)[idx];
	struct txq *txq = NULL;
	int ret;

	if (ftxq)
		txq = container_of(ftxq, struct txq, ftxq);
	if (mlx5_is_secondary())
		return -E_RTE_SECONDARY;

	priv_lock(priv);
	if (desc & (desc - 1)) {
		desc = 1 << log2above(desc - 1);
		WARN("%p: increased number of descriptor in RX queue %u"
		     " to the next power of two value (%d)",
		     (void *)dev, idx, desc);
	}
	DEBUG("%p: configuring queue %u for %u descriptors",
	      (void *)dev, idx, desc);
	if (idx >= priv->txqs_n) {
		ERROR("%p: queue index out of range (%u >= %u)",
		      (void *)dev, idx, priv->txqs_n);
		priv_unlock(priv);
		return -EOVERFLOW;
	}
	if (txq != NULL) {
		DEBUG("%p: reusing already allocated queue index %u (%p)",
		      (void *)dev, idx, (void *)txq);
		if (priv->started) {
			priv_unlock(priv);
			return -EEXIST;
		}
		(*priv->txqs)[idx] = NULL;
		txq_cleanup(txq);
	} else {
		txq = rte_calloc_socket("TXQ", 1, sizeof(*txq) +
				desc * sizeof(struct rte_mbuf *), 0, socket);
		if (txq == NULL) {
			ERROR("%p: unable to allocate queue index %u",
			      (void *)dev, idx);
			priv_unlock(priv);
			return -ENOMEM;
		}
	}
	ret = txq_setup(dev, txq, desc, socket, conf);
	if (ret)
		rte_free(txq);
	else {
		txq->ftxq.stats.idx = idx;
		DEBUG("%p: adding TX queue %p to list",
		      (void *)dev, (void *)txq);
		(*priv->txqs)[idx] = &txq->ftxq;
		/* Update send callback. */
		priv_select_tx_function(priv);
	}
	priv_unlock(priv);
	return -ret;
}

/**
 * DPDK callback to release a TX queue.
 *
 * @param dpdk_txq
 *   Generic TX queue pointer.
 */
void
mlx5_tx_queue_release(void *dpdk_txq)
{
	struct ftxq *ftxq = (struct ftxq *)dpdk_txq;
	struct txq *txq = container_of(ftxq, struct txq, ftxq);
	struct priv *priv;
	unsigned int i;

	if (mlx5_is_secondary())
		return;

	if (txq == NULL)
		return;
	priv = txq->priv;
	priv_lock(priv);
	for (i = 0; (i != priv->txqs_n); ++i)
		if ((*priv->txqs)[i] == ftxq) {
			DEBUG("%p: removing TX queue %p from list",
			      (void *)priv->dev, (void *)txq);
			(*priv->txqs)[i] = NULL;
			break;
		}
	txq_cleanup(txq);
	rte_free(txq);
	priv_unlock(priv);
}

/**
 * DPDK callback for TX in secondary processes.
 *
 * This function configures all queues from primary process information
 * if necessary before reverting to the normal TX burst callback.
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
mlx5_tx_burst_secondary_setup(void *dpdk_txq, struct rte_mbuf **pkts,
			      uint16_t pkts_n)
{
	struct ftxq *ftxq = dpdk_txq;
	struct txq *txq = container_of(ftxq, struct txq, ftxq);
	struct priv *priv = mlx5_secondary_data_setup(txq->priv);
	struct priv *primary_priv;
	unsigned int index;

	if (priv == NULL)
		return 0;
	primary_priv =
		mlx5_secondary_data[priv->dev->data->port_id].primary_priv;
	/* Look for queue index in both private structures. */
	for (index = 0; index != priv->txqs_n; ++index)
		if (((*primary_priv->txqs)[index] == ftxq) ||
		    ((*priv->txqs)[index] == ftxq))
			break;
	if (index == priv->txqs_n)
		return 0;
	ftxq = (*priv->txqs)[index];
	return priv->dev->tx_pkt_burst(ftxq, pkts, pkts_n);
}
