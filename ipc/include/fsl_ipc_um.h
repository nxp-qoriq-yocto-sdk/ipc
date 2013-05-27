/*
 *  @ fsl_bsc913x_ipc_kern_mod.c
 *
 * Copyright 2011-2013 Freescale Semiconductor, Inc.
 *
 * Author: Ashish Kumar <ashish.kumar@freescale.com>
 * Author: Manish Jaggi <manish.jaggi@freescale.com>
 */

#ifndef FSL_IPC_UM_H_
#define FSL_IPC_UM_H_

#include "fsl_ipc_types.h"
#include "fsl_user_dma.h"
#include "fsl_bsc913x_ipc.h"
#include "fsl_heterogeneous_ipc.h"
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
	unsigned long 	txreq_tb_lbuff_paddr;

	int 		num_channels;
	ipc_channel_us_t *channels[MAX_CHANNELS];

	ipc_p2v_t 	p2vcb;

	int 		dev_het_ipc;
	int 		dev_mem;
	int 		dev_het_mgr;

	uint32_t	txreq_inprocess;

	uint32_t	max_depth;
	uint32_t	max_channels;

	mem_range_t		sh_ctrl_area;
	mem_range_t		dsp_ccsr;
	mem_range_t		pa_ccsr;

	fsl_udma_t	udma;
	int 		rat_id;
	os_het_ipc_t	*ipc_inst;
	char uio_dev_buf[100];
} ipc_userspace_t;

#endif /* FSL_IPC_UM_H_ */
