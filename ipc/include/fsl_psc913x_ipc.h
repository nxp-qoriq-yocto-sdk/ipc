/*
 * @fsl_psc913x_ipc.h
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

#ifndef FSL_PSC913x_IPC_H_
#define FSL_PSC913x_IPC_H_

#include "fsl_types.h"

/* Defines */
/*****************************************************************************/
#define MAX_TX_REQ_MSG_SIZE 1020
#define MAX_SG_ENTRIES 8
/*****************************************************************************/
typedef void *fsl_ipc_t;
/*****************************************************************************
 * @ipc_ch_type_t
*****************************************************************************/
typedef enum {
	IPC_MSG_CH = 0,
	IPC_PTR_CH,
	IPC_TXREQ_CH
} ipc_ch_type_t;

/*****************************************************************************
 * Format of the tx.request message associated with Tx.Req Channel type
*****************************************************************************/
typedef struct {
	uint8_t		tx_request_buf[MAX_TX_REQ_MSG_SIZE];
	phys_addr_t	txreq_linearized_buf;	/* Pointer to the
						   linearized buffer*/
} txreq_ctrl_t;

/*****************************************************************************
 * @sg_entry_t
 * Scatter gather table entry
 *
 * src_addr 	-	dma_capable_physical_addr;
 *
 * is_tb_start 	-	if set, the src_address is a 32bit aligned. This should
 * 			 be set for the first tb part.
 *
 * len		- 	length of tb part
 *
 * is_valid 	- 	is this a valid entry.
 *
*****************************************************************************/
typedef struct {
	phys_addr_t	src_addr;
	uint32_t	len;
	uint8_t		is_tb_start;
	uint8_t		is_valid;
} sg_entry_t;

typedef struct {
	sg_entry_t  entry[MAX_SG_ENTRIES];
} sg_list_t;

/*****************************************************************************
 * @ipc_cbfunc_t
 *
 * Consumer callback function data type.
 *
 * channel_id	-	unique id of the channel
 *
 * context	-	This parameter has a different meaning based on the
 * 			channel type:
 *			On a IPC_MSG_CH - the context is the ipc buffer pointer
 *			from which the consumer should copy in local buffer
 *
 *			IPC_PTR_CH - the context may be a buffer pointer
 *			IPC_TXREQ_CH - not valid
 *
 * len		- 	usually contains the length of the context
 *
*****************************************************************************/
typedef void (*ipc_cbfunc_t)(uint32_t channel_id, void *context,
				uint32_t msg_len);

/*****************************************************************************
 * @ipc_p2v_t
 *
 * IPC callback function to get the virtual address from ipc user.
 *
 * phys_addr	-	physical address
 *
 * Return value
 * void*	- 	virtual address
 *
*****************************************************************************/
typedef void* (*ipc_p2v_t)(phys_addr_t phys_addr);

/*****************************************************************************
 * @fsl_ipc_init
 *
 * Init function to initialize the IPC subsystem.
 *
 * p2vcb 	- pointer to a function which does p2v
 * sh_ctrl_area - range_t for shared control area
 * dsp_ccsr 	- range_t for dsp_ccsr
 * pa_ccsr 	- range_t for pa_ccsr
 *
 * Return Value -
 *			fsl_ipc_t handle.
 *			This has to be provided in all subsequent calls to ipc
 *
*****************************************************************************/
fsl_ipc_t fsl_ipc_init(ipc_p2v_t p2vcb, range_t sh_ctrl_area, range_t dsp_ccsr,
			range_t pa_ccsr);
/*****************************************************************************
 * @ipc_configure_channel
 *
 * To be called one time per channel by the consumer. The channel pointer
 * ring is already created during ipc kernel driver initialization.
 * NOTE: The number of channels and the max depth of channels is taken as a
 * boot argument to linux kernel.
 *
 * channel_id	- 	unique id of the channel
 *
 * depth 	- 	user configurable number of entries in the ring.
 * 			depth <= max depth
 *
 * channel_type -	either of IPC_PTR_CH/IPC_MSG_CH only
 *
 * msg_ring_paddr - 	Physical address of the message ring.
 *
 * msg_size 	- 	max size of each message.
 * 			For PTR_CH, msg_ring_vaddr, msg_ring_paddr, msg_size
 *	 		are all NULL.
 *
 * cbfunc	- 	The callback function called on receiving a interrupt
 * 			from the producer. If cbfunc is NULL, channel does not
 * 			support	notifications.
 *
 * 			The channel supports the notification using interrupts
 * 			The ipc layer will find a free rt signal for process
 * 			and attach the signal with the interrupt.
 *
 * 			The kernel mode component of ipc will find a free irq
 * 			and attach to the channel structure in the shared ctrl
 * 			area which can be read by the producer.
 *
 * Return Value:
 * 	ERR_SUCCESS - no error
 * 	Non zero value - error (check fsl_ipc_errorcodes.h)
 *
*****************************************************************************/
int fsl_ipc_configure_channel(uint32_t channel_id, uint32_t depth,
			ipc_ch_type_t channel_type,
			phys_addr_t msg_ring_paddr, uint32_t msg_size,
			ipc_cbfunc_t cbfunc, fsl_ipc_t ipc);

/*****************************************************************************
 * @fsl_ipc_configure_txreq
 *
 ****************************************************************************/
int fsl_ipc_open_prod_ch(uint32_t channel_id, fsl_ipc_t ipc);

/*****************************************************************************
 * @fsl_ipc_configure_txreq
 *
 * For tx request the DSP side creates a msg channel of a particular depth
 * having each entry the size of tx_request fapi message, the producer side
 * IPC allocates a set of buffers for linearizing the tx request PDU's.
 *
 * The number of such buffers is same as depth of the channel + 1
 * NOTE: The extra buffer is used as a spare for IPC internal operations
 *
 * channel_id - unique id of the channel
 *
 * max_txreq_linearized_buf_size -
 * 		max size of a buffer which holds the linearized TB.
 * 		IPC reads the depth and allocates depth*max_txreq_lbuff_size
 * 		memory
 ****************************************************************************/
int fsl_ipc_configure_txreq(uint32_t channel_id, phys_addr_t lbuff_phys_addr,
			uint32_t max_txreq_linearized_buf_size, fsl_ipc_t ipc);

/*****************************************************************************
 *@fsl_ipc_send_ptr
 *
 *USE:
 *	Type 1 PRODUCER API, For sending a buffer pointer from producer
 *	to consumer.
 *
 * buffer_ptr	- 	The producer buffer pointer which is visible to both
 * 		    	producer and consumer
 *
 * len		- 	length of the producer buffer.
 ****************************************************************************/
int fsl_ipc_send_ptr(uint32_t channel_id, phys_addr_t buffer_ptr,
		uint32_t len, fsl_ipc_t ipc);
/*****************************************************************************
 *@fsl_ipc_send_msg
 *
 *	Type 2 PRODUCER API. For sending a buffer from producer to consumer.
 *	IPC copies the buffer into internal message ring.
 *
 * src_buf_addr	-	virtual address of the producer buffer
 *
 * len		-	length of the producer buffer.
 ****************************************************************************/
int fsl_ipc_send_msg(uint32_t channel_id, void *src_buf_addr, uint32_t len,
			fsl_ipc_t ipc);

/*****************************************************************************
 *@fsl_ipc_send_tx_req
 *
 * sgl		- 	A scatter gather list of tb parts
 *
 * tx_reg_addr	-	Virtual Address of the tx request buffer in producer's
 * 			memory this buffer is copied on to the message ring
 *
 * Return Value -
 * 	ERR_SUCCESS - no error
 * 	Non zero value - error (check fsl_ipc_errorcodes.h)
 ****************************************************************************/
int fsl_ipc_send_tx_req(uint32_t channel_id, sg_list_t *sgl,
		void *tx_req_vaddr, uint32_t tx_req_len, fsl_ipc_t ipc);

/*****************************************************************************
 * @fsl_ipc_get_last_tx_req_status
 *
 *	This api should be called by the producer to check the completion of
 *	last DMA transfer initiated by fsl_ipc_send_tx_req API.
 *
 * Return Value
 * int		-	status -TXREQ_DONE
 * 				TXREQ_IN_PROCESS
 * 				TXREQ_ERR
 *				(defined in fsl_ipc_errorcodes.h)
 ****************************************************************************/
int fsl_ipc_get_last_tx_req_status(fsl_ipc_t ipc);

/*****************************************************************************
 * @fsl_ipc_recv_ptr
 *
 *	Consumer API, called when the consumer is polling
 *
 * addr -
 * 	ipc copies this value from the ptr ring, and increments the
 * 	consumer index. (value is of phys_addr_t in most cases)
 *
 * len -
 * 	length provided by the producer
 *
 * Return Value -
 * 	ERR_SUCCESS - no error
 * 	Non zero value - error (check fsl_ipc_errorcodes.h)
 ****************************************************************************/
int fsl_ipc_recv_ptr(uint32_t channel_id, phys_addr_t *addr, uint32_t *len,
			fsl_ipc_t ipc);

/*****************************************************************************
 * @fsl_ipc_recv_ptr_hold
 *
 *	Consumer API, called when the consumer is polling
 *
 * addr -
 * 	ipc copies this value from the ptr ring, and does not increment the
 * 	consumer index. (value is of phys_addr_t in most cases).
 *	The consumer index is updated by calling fsl_ipc_set_consumed_status
 *
 * len -
 * 	length provided by the producer
 *
 * Return Value -
 * 	ERR_SUCCESS - no error
 * 	Non zero value - error (check fsl_ipc_errorcodes.h)
 ****************************************************************************/
int fsl_ipc_recv_ptr_hold(uint32_t channel_id, phys_addr_t *addr, uint32_t *len,
			fsl_ipc_t ipc);

/*****************************************************************************
 * @fsl_ipc_recv_msg
 *
 * 	Consumer API, called when the consumer is polling
 *
 * addr -
 * 	IPC copies from the message ring into the buffer pointer provided
 * 	by the consumer, and increments the consumer index.
 *
 * len -
 * 	length of the copied buffer
 *
 * Return Value -
 * 	ERR_SUCCESS - no error
 * 	Non zero value - error (check fsl_ipc_errorcodes.h)
 ****************************************************************************/
int fsl_ipc_recv_msg(uint32_t channel_id, void *dst_buffer, uint32_t *len,
		fsl_ipc_t ipc);

/*****************************************************************************
 * @fsl_ipc_recv_msg_ptr
 *
 * 	Consumer API, called when the consumer is polling, and when the
 * 	consumer is using the buffer in the message ring without copying
 * 	in the local buffer. (Zero Copy)
 * 	When consumed fully the API fsl_ipc_set_consumed_status should be
 * 	called, this would increment the consumer index.
 *
 * channel_id 	- unique id of the channel
 *
 * addr 	- IPC copies from the message ring into the buffer pointer
 * 		 provided by the consumer
 *
 * len 		- length of the copied buffer
 *
 * Return Value -
 * 	ERR_SUCCESS - no error
 * 	Non zero value - error (check fsl_ipc_errorcodes.h)
 ****************************************************************************/
int fsl_ipc_recv_msg_ptr(uint32_t channel_id, void *dst_buffer, uint32_t *len,
			fsl_ipc_t ipc);

/*****************************************************************************
 * @fsl_ipc_set_consumed_status
 *
 * channel_id	- unique id of the channel
 *
 * 	Called along with fsl_ipc_recv_msg_ptr to increment the consumer index
 * 	on that channel
 ****************************************************************************/
int fsl_ipc_set_consumed_status(uint32_t channel_id, fsl_ipc_t ipc);

/*****************************************************************************
 * @fsl_ipc_chk_recv_status
 *
 * The api checks all the consumer channels owned by the calling process to
 * find out which has a msg/ptr received.
 *
 * bmask 	- There can be a max of 32 channels. Each bit set represent
 * 		a channel has recieved a message/ptr. Bit 0 MSB - Bit 32 LSB
 * 		(Bit = Channel id)
 ****************************************************************************/
int fsl_ipc_chk_recv_status(uint64_t *bmask, fsl_ipc_t ipc);

#endif /* FSL_913x_IPC_H_ */
