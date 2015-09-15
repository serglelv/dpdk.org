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

#ifndef RTE_PMD_MLX5_UTILS_H_
#define RTE_PMD_MLX5_UTILS_H_

#include <stddef.h>
#include <stdio.h>
#include <limits.h>
#include <assert.h>
#include <errno.h>

#include "mlx5_defs.h"

/* Bit-field manipulation. */
#define BITFIELD_DECLARE(bf, type, size)				\
	type bf[(((size_t)(size) / (sizeof(type) * CHAR_BIT)) +		\
		 !!((size_t)(size) % (sizeof(type) * CHAR_BIT)))]
#define BITFIELD_DEFINE(bf, type, size)					\
	BITFIELD_DECLARE((bf), type, (size)) = { 0 }
#define BITFIELD_SET(bf, b)						\
	(assert((size_t)(b) < (sizeof(bf) * CHAR_BIT)),			\
	 (void)((bf)[((b) / (sizeof((bf)[0]) * CHAR_BIT))] |=		\
		((size_t)1 << ((b) % (sizeof((bf)[0]) * CHAR_BIT)))))
#define BITFIELD_RESET(bf, b)						\
	(assert((size_t)(b) < (sizeof(bf) * CHAR_BIT)),			\
	 (void)((bf)[((b) / (sizeof((bf)[0]) * CHAR_BIT))] &=		\
		~((size_t)1 << ((b) % (sizeof((bf)[0]) * CHAR_BIT)))))
#define BITFIELD_ISSET(bf, b)						\
	(assert((size_t)(b) < (sizeof(bf) * CHAR_BIT)),			\
	 !!(((bf)[((b) / (sizeof((bf)[0]) * CHAR_BIT))] &		\
	     ((size_t)1 << ((b) % (sizeof((bf)[0]) * CHAR_BIT))))))

/* Debugging */
#ifndef NDEBUG
#define DEBUG__(m, ...)						\
	(fprintf(stderr, "%s:%d: %s(): " m "%c",		\
		 __FILE__, __LINE__, __func__, __VA_ARGS__),	\
	 fflush(stderr),					\
	 (void)0)
/*
 * Save/restore errno around DEBUG__().
 * XXX somewhat undefined behavior, but works.
 */
#define DEBUG_(...)				\
	(errno = ((int []){			\
		*(volatile int *)&errno,	\
		(DEBUG__(__VA_ARGS__), 0)	\
	})[0])
#define DEBUG(...) DEBUG_(__VA_ARGS__, '\n')
#define claim_zero(...) assert((__VA_ARGS__) == 0)
#else /* NDEBUG */
/* No-ops. */
#define DEBUG(...) (void)0
#define claim_zero(...) (__VA_ARGS__)
#endif /* NDEBUG */

/* Runtime logging through RTE_LOG() is enabled when not in debugging mode.
 * Intermediate LOG_*() macros add the required end-of-line characters. */
#ifndef NDEBUG
#define INFO(...) DEBUG(__VA_ARGS__)
#define WARN(...) DEBUG(__VA_ARGS__)
#define ERROR(...) DEBUG(__VA_ARGS__)
#else
#define LOG__(level, m, ...) \
	RTE_LOG(level, PMD, MLX5_DRIVER_NAME ": " m "%c", __VA_ARGS__)
#define LOG_(level, ...) LOG__(level, __VA_ARGS__, '\n')
#define INFO(...) LOG_(INFO, __VA_ARGS__)
#define WARN(...) LOG_(WARNING, __VA_ARGS__)
#define ERROR(...) LOG_(ERR, __VA_ARGS__)
#endif

/* Convenience macros for accessing mbuf fields. */
#define NEXT(m) ((m)->next)
#define DATA_LEN(m) ((m)->data_len)
#define PKT_LEN(m) ((m)->pkt_len)
#define DATA_OFF(m) ((m)->data_off)
#define SET_DATA_OFF(m, o) ((m)->data_off = (o))
#define NB_SEGS(m) ((m)->nb_segs)
#define PORT(m) ((m)->port)

/* Transpose flags. Useful to convert IBV to DPDK flags. */
#define TRANSPOSE(val, from, to) \
	(((from) >= (to)) ? \
	 (((val) & (from)) / ((from) / (to))) : \
	 (((val) & (from)) * ((to) / (from))))

/* Allocate a buffer on the stack and fill it with a printf format string. */
#define MKSTR(name, ...) \
	char name[snprintf(NULL, 0, __VA_ARGS__) + 1]; \
	\
	snprintf(name, sizeof(name), __VA_ARGS__)

#endif /* RTE_PMD_MLX5_UTILS_H_ */
