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

#ifndef RTE_PMD_MLX5_HW_H_
#define RTE_PMD_MLX5_HW_H_

/* Verbs header. */
/* ISO C doesn't support unnamed structs/unions, disabling -pedantic. */
#ifdef PEDANTIC
#pragma GCC diagnostic ignored "-pedantic"
#endif
#include <infiniband/mlx5_hw.h>
#ifdef PEDANTIC
#pragma GCC diagnostic error "-pedantic"
#endif

/* Get CQE owner bit. */
#define MLX5_CQE_OWNER(op_own) ((op_own) & MLX5_CQE_OWNER_MASK)

/* Get CQE format. */
#define MLX5_CQE_FORMAT(op_own) (((op_own) & MLX5E_CQE_FORMAT_MASK) >> 2)

/* Get CQE op code. */
#define MLX5_CQE_OPCODE(op_own) ((op_own) >> 4)

/* Get CQE Solicited Event. */
#define MLX5_CQE_SE(op_own) (((op_own) >> 1) & 1)

/* Invalidate a CQE. */
#define MLX5_CQE_INVALIDATE (MLX5_CQE_INVALID << 4)

#define MLX5_CQE_VLAN_STRIPPED	0x1

struct mlx5_cqe_comp {
	uint8_t rsvd0;
	uint8_t scqe_idx;
	uint16_t wqe_id;
	uint16_t cq_ci_narray;
	uint16_t cq_ci_carray;
	uint8_t	rsvd4[4];
	uint32_t rx_hash_res;
	uint8_t rx_hash_type;
	uint8_t ml_path;
	uint16_t cqe_ci;
	uint16_t checksum;
	uint16_t slid;
	uint32_t flags_rqpn;
	uint8_t hds_ip_ext;
	uint8_t l4_hdr_type_etc;
	__be16 vlan_info;
	uint32_t srqn_uidx;
	uint32_t imm_inval_pkey;
	uint16_t rsvd40;
	uint16_t cq_ci;
	uint32_t cqe_cnt;
	__be64	 timestamp;
	union {
		uint32_t sop_drop_qpn;
		struct {
			uint8_t	sop;
			uint8_t qpn[3];
		} sop_qpn;
	} sop;
	/*
	 * In Striding RQ (Multi-Packet RQ) wqe_counter provides
	 * the WQE stride index (to calc pointer to start of the message)
	 */
	uint16_t wqe_cnt;
	uint8_t signature;
	uint8_t op_own;
} __attribute__((aligned(64)));

union mlx5_rx_cqe {
	struct mlx5_cqe64 cqe64;
	struct mlx5_cqe_comp zip;
};

#endif /* RTE_PMD_MLX5_HW_H_ */
