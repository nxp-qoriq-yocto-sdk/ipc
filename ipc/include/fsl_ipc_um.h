/*
 *  @ fsl_psc913x_ipc_kern_mod.c
 *
 * Copyright (c) 2011
 *  Freescale Semiconductor Inc.  All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of Freescale Semiconductor Inc nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 *      Author: Manish Jaggi <manish.jaggi@freescale.com>
 */

#ifndef FSL_IPC_UM_H_
#define FSL_IPC_UM_H_

#include "fsl_types.h"
#include "fsl_user_dma.h"
#include "fsl_psc913x_ipc.h"

#define MAX_CHANNELS 64

typedef struct {
	void 		*msg_ring_vaddr;
	ipc_cbfunc_t	cbfunc;
	uint32_t 	signal;
	uint32_t	channel_id;
} ipc_channel_us_t;

typedef struct {
	int		init;
	/* MAX TXreq Linerized TB buffer size */
	uint32_t 	max_txreq_lbuff_size;
	uint32_t 	txreq_tb_lbuff_paddr;

	int 		num_channels;
	ipc_channel_us_t *channels[MAX_CHANNELS];

	ipc_p2v_t 	p2vcb;

	int 		dev_het_ipc;
	int 		dev_mem;
	int 		dev_het_mgr;

	uint32_t	txreq_inprocess;

	uint32_t	max_depth;
	uint32_t	max_channels;

	range_t		sh_ctrl_area;
	range_t		dsp_ccsr;
	range_t		pa_ccsr;

	fsl_udma_t	udma;

} ipc_userspace_t;

#endif /* FSL_IPC_UM_H_ */
