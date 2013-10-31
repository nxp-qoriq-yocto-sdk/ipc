/*
 * @fsl_bsc913x_ipc.h
 *
 * Copyright 2011-2013 Freescale Semiconductor, Inc.
 *
 * Author: ashish kumar <ashish.kumar@freescale.com>
 * Author: Manish Jaggi <manish.jaggi@freescale.com>
 */

#ifndef FSL_PSC913x_IPC_H_
#define FSL_PSC913x_IPC_H_

/****************
Abbreviations
IN	- Input Parameter
OUT	- Output Paramater
M	- Mandatory
O	- Optional
****************/

#include "fsl_ipc_types.h"
#include "dsp_boot.h"

/* Defines */
/*****************************************************************************/
#define MAX_TX_REQ_MSG_SIZE 1020

#define MAX_SG_ENTRIES 128
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
	unsigned long	txreq_linearized_buf;	/* Pointer to the
						   linearized buffer*/
} txreq_ctrl_t;

/*****************************************************************************
 * @sg_entry_t
 * Scatter gather table entry
 *
 * src_addr 	-	dma_capable_physical_addr;
 *
 * is_tb_start	-	This field is currently ignored.
 *
 * len		- 	length of tb part
 *
 * is_valid 	- 	is this a valid entry.
 *
*****************************************************************************/
typedef struct {
	unsigned long	src_addr;
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
 * channel_id	-	[IN][M] unique id of the channel
 *
 * context	-	[IN][M] This parameter has different meaning based on
 * 			channel type:
 *			On a IPC_MSG_CH - the context is the ipc buffer pointer
 *			from which the consumer should copy in local buffer
 *
 *			IPC_PTR_CH - the context may be a buffer pointer
 *			IPC_TXREQ_CH - not valid
 *
 * len		- 	[IN][M] usually contains the length of the context
 *
*****************************************************************************/
typedef void (*ipc_cbfunc_t)(uint32_t channel_id, void *context,
				uint32_t msg_len);

/*****************************************************************************
 * @ipc_p2v_t
 *
 * IPC callback function to get the virtual address from ipc user.
 *
 * phys_addr	-	[IN][M] physical address
 *
 * Return value
 * void*	- 	virtual address
 *
*****************************************************************************/
typedef void* (*ipc_p2v_t)(unsigned long phys_addr);

/*****************************************************************************
 * @fsl_ipc_init
 *
 * Init function to initialize the IPC subsystem.
 *
 * p2vcb 	- [IN][M]pointer to a function which does p2v
 * sh_ctrl_area - [IN][M]mem_range_t for shared control area
 * dsp_ccsr 	- [IN][M]mem_range_t for dsp_ccsr
 * pa_ccsr 	- [IN][M]mem_range_t for pa_ccsr
 * uiodevbuf 	- [IN][M]UIO INTERFACE to used
 *
 * Return Value -
 *			fsl_ipc_t handle.
 *			This has to be provided in all subsequent calls to ipc
 *
*****************************************************************************/
fsl_ipc_t fsl_ipc_init(ipc_p2v_t p2vcb, mem_range_t sh_ctrl_area,
		mem_range_t dsp_ccsr, mem_range_t pa_ccsr, char uiodevbuf[]);

/*****************************************************************************
 * @fsl_ipc_init_rat
 *
 * Init function to initialize the IPC subsystem.
 *
 * rat_id	- [IN][M]id of the rat for which ipc is instantiated.
 *			This is used on in multiRAT scenerio.
 * p2vcb 	- [IN][M]pointer to a function which does p2v
 * sh_ctrl_area - [IN][M]mem_range_t for shared control area
 * dsp_ccsr 	- [IN][M]mem_range_t for dsp_ccsr
 * pa_ccsr 	- [IN][M]mem_range_t for pa_ccsr
 * uiodevbuf 	- [IN][M]UIO INTERFACE to used
 *
 * Return Value -
 *			fsl_ipc_t handle.
 *			This has to be provided in all subsequent calls to ipc
 *
*****************************************************************************/
fsl_ipc_t fsl_ipc_init_rat(uint32_t rat_id, ipc_p2v_t p2vcb,
		mem_range_t sh_ctrl_area, mem_range_t dsp_ccsr,
		mem_range_t pa_ccsr, char uiodevbuf[]);

/*****************************************************************************
 * @ipc_configure_channel
 *
 * To be called one time per channel by the consumer. The channel pointer
 * ring is already created during ipc kernel driver initialization.
 * NOTE: The number of channels and the max depth of channels is taken as a
 * boot argument to linux kernel.
 *
 * channel_id	- 	[IN][M]unique id of the channel
 *
 * depth 	- 	[IN][M]user configurable number of entries in the ring.
 * 			depth <= max depth
 *
 * channel_type -	[IN][M]either of IPC_PTR_CH/IPC_MSG_CH
 *
 * msg_ring_paddr - 	[IN]Physical address of the message ring. Required
 *			only for IPC_MSG_CH
 *
 * msg_size 	- 	[IN]max size of each message.
 * 			For PTR_CH, msg_ring_vaddr, msg_ring_paddr, msg_size
 *	 		are all NULL. Required only for IPC_MSG_CH
 *
 * cbfunc	- 	[IN]The callback function called on receiving interrupt
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
			unsigned long msg_ring_paddr, uint32_t msg_size,
			ipc_cbfunc_t cbfunc, fsl_ipc_t ipc);

/*****************************************************************************
 * @fsl_ipc_open_prod_ch
 *
 * All params [IN][M]
 * Sets the Producer Initialized value in the channel structure
 * Return Value:
 * 	ERR_SUCCESS - no error
 * 	Non zero value - error (check fsl_ipc_errorcodes.h)
 ****************************************************************************/
int fsl_ipc_open_prod_ch(uint32_t channel_id, fsl_ipc_t ipc);

/*****************************************************************************
 * @fsl_ipc_configure_txreq
 *
 * For tx request the DSP side creates a msg channel of a particular depth
 * having each entry the size of tx_request fapi message. The size of a
 * tx_request message in IPC is 1024bytes at max. The fapi message
 * can be of max 1020 bytes. Last 4 bytes points to the linearized buffer
 * corresponsing to the tx_request fapi message.
 * While DSP allocates memory for tx_request FAPI message, the producer
 * PA allocates memory for lineraized buffers.
 * PA allocate memory = (Max size of 1 lineaized buffer)*(channel depth +2)
 *
 * NOTE: The extra buffer is used as a spare for IPC internal operations
 *
 * channel_id - unique id of the channel
 *
 * max_txreq_linearized_buf_size -[IN][M]
 * 		max size of a buffer which holds the linearized TB.
 / 		PA allocates (depth+2)*max_txreq_lbuff_size
 *
 * lbuff_phys_addr -[IN][M]
 *		Start address of the allocated buffer
 *
 * Note: PA should not use this buffer for other operation.
 * Return Value -
 * 	ERR_SUCCESS - no error
 ****************************************************************************/
int fsl_ipc_configure_txreq(uint32_t channel_id, unsigned long lbuff_phys_addr,
			uint32_t max_txreq_linearized_buf_size, fsl_ipc_t ipc);

/*****************************************************************************
 *@fsl_ipc_send_ptr
 *
 *USE:
 *	Type 1 PRODUCER API, For sending a buffer pointer from producer
 *	to consumer.
 *
 * buffer_ptr	- 	[IN][M]The producer buffer pointer which is visible
 * 		    	to producer and consumer
 *
 * len		- 	[IN][M]length of the producer buffer.
 * Return Value -
 * 	ERR_SUCCESS - no error
 * 	Non zero value - error (check fsl_ipc_errorcodes.h)
 ****************************************************************************/
int fsl_ipc_send_ptr(uint32_t channel_id, unsigned long buffer_ptr,
		uint32_t len, fsl_ipc_t ipc);
/*****************************************************************************
 *@fsl_ipc_send_msg
 *
 *	Type 2 PRODUCER API. For sending a buffer from producer to consumer.
 *	IPC copies the buffer into internal message ring.
 *
 * src_buf_addr	- [IN][M]virtual address of the producer buffer
 *
 * len			- [IN][M]length of the producer buffer.
 * Return Value -
 * 	ERR_SUCCESS - no error
 ****************************************************************************/
int fsl_ipc_send_msg(uint32_t channel_id, void *src_buf_addr, uint32_t len,
			fsl_ipc_t ipc);

/*****************************************************************************
 *@fsl_ipc_send_tx_req
 *
 * sgl			- [IN][M] A scatter gather list of tb parts
 *
 * tx_reg_addr	- [IN][M] Virtual Address of tx request buffer in producer's
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
 * addr - [IN][M]
 * 	ipc copies this value from the ptr ring, and increments the
 * 	consumer index. (value is of unsigned long in most cases)
 *
 * len - [IN][M]
 * 	length provided by the producer
 *
 * Return Value -
 * 	ERR_SUCCESS - no error
 * 	Non zero value - error (check fsl_ipc_errorcodes.h)
 ****************************************************************************/
int fsl_ipc_recv_ptr(uint32_t channel_id, unsigned long *addr, uint32_t *len,
			fsl_ipc_t ipc);

/*****************************************************************************
 * @fsl_ipc_recv_ptr_hold
 *
 *	Consumer API, called when the consumer is polling
 *
 * addr - [IN][M]
 * 	ipc copies this value from the ptr ring, and does not increment the
 * 	consumer index. (value is of unsigned long in most cases).
 *	The consumer index is updated by calling fsl_ipc_set_consumed_status
 *
 * len - [IN][M]
 * 	length provided by the producer
 *
 * Return Value -
 * 	ERR_SUCCESS - no error
 * 	Non zero value - error (check fsl_ipc_errorcodes.h)
 ****************************************************************************/
int fsl_ipc_recv_ptr_hold(uint32_t channel_id, unsigned long *addr,
		uint32_t *len, fsl_ipc_t ipc);

/*****************************************************************************
 * @fsl_ipc_recv_msg
 *
 * 	Consumer API, called when the consumer is polling
 *
 * addr - [IN][M]
 * 	IPC copies from the message ring into the buffer pointer provided
 * 	by the consumer, and increments the consumer index.
 *
 * len - [IN][M]
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
 * channel_id 	- [IN][M] unique id of the channel
 *
 * dst_buffer 	- [OUT][M] IPC copies the virtual address of message buffer
 *
 * len 		- [IN][M] length of the copied buffer
 *
 * Return Value -
 * 	ERR_SUCCESS - no error
 * 	Non zero value - error (check fsl_ipc_errorcodes.h)
 ****************************************************************************/
int fsl_ipc_recv_msg_ptr(uint32_t channel_id, void **dst_buffer, uint32_t *len,
			fsl_ipc_t ipc);

/*****************************************************************************
 * @fsl_ipc_set_consumed_status
 *
 * channel_id	- [IN][M] unique id of the channel
 *
 * 	Called along with fsl_ipc_recv_msg_ptr to increment the consumer index
 * 	on that channel
 * Return Value -
 * 	ERR_SUCCESS - no error
 * 	Non zero value - error (check fsl_ipc_errorcodes.h)
 ****************************************************************************/
int fsl_ipc_set_consumed_status(uint32_t channel_id, fsl_ipc_t ipc);

/*****************************************************************************
 * @fsl_ipc_chk_recv_status
 *
 * The api checks all the consumer channels owned by the _calling process_ to
 * find out which has a msg/ptr received.
 *
 * bmask 	- [OUT][M] There can be a max of 64 channels. Each bit set
 *		represents a channel has recieved a message/ptr.
 *		Bit 0 MSB - Bit 64 LSB
 * 		(Bit = Channel id)
 * Return Value -
 * 	ERR_SUCCESS - no error
 * 	Non zero value - error (check fsl_ipc_errorcodes.h)
 ****************************************************************************/
int fsl_ipc_chk_recv_status(uint64_t *bmask, fsl_ipc_t ipc);

/**Re-Initialize IPC Channels**/
/***************************************************************************
 ** @fsl_ipc_reinit
 **
 ** Re initialize all IPC channels
 **
 ** ipc[in] . ipc handle returned by fsl_ipc_init
 **
 ** Return: On Success, zero is returned
 ** On Failure, non-zero value is returned.
 ***************************************************************************/
int fsl_ipc_reinit(fsl_ipc_t ipc);

/**Reloading DSP Image**/
/*************************************************************************
 ** @fsl_load_dsp_image
 **
 ** Loads DSP image
 **
 ** fname[in] . absolute path of DSP image to be loaded
 **
 ** Return: On Success, zero is returned
 ** On Failure, non-zero value is returned.
 **************************************************************************/
int fsl_load_dsp_image(char *fname);

/**Triggering VNMI**/
/*************************************************************************
 ** @fsl_send_vnmi
 * *
 * * Generate Virtual NMI towards DSP core
 * *
 * * Return: On Success, zero is returned
 * *On Failure, non-zero value is returned.
 * ***********************************************************************/
int fsl_send_vnmi(void);

/*************************************************************************
 * * @fsl_restart_L1
 * *
 * *  Triggers the DSP Recovery and re-loads/start DSP image
 * *
 * * ipc[in] . IPC handle returned earlier by fsl_ipc_init
 * * fname[in] . absolute path of DSP/L1 image to be re-loaded/started
 * *
 * * Return: On Success, zero is returned
 * *                 On Failure, non-zero value is returned.
 * *
 * * Description
 * * It re-initializes all IPC channels and then generates a VNMI towards DSP
 * * so that DSP can perform recovery. It furthers re-loads and starts the
 * * DSP/L1 image upon receiving `DSP Ready. indication from DSP.
 * ************************************************************************/
int fsl_restart_L1(fsl_ipc_t ipc, char *fname);

/**************************************************************************
 * * Function pointer returning void
 * * core_mask[in] core mask returned by IOCTL thread
 * *
 * * typedef void (*fsl_defense_cb)(uint32_t core_mask)
 * * cb: pointer to function of type .fsl_defence_cb.
 * *
 * ***********************************************************************/
typedef void (*fsl_defense_cb)(uint32_t core_mask);

/**************************************************************************
 * * @fsl_L1_defense_register_cb
 * * typedef void (*fsl_defense_cb)(uint32_t core_mask)
 * * cbi[in]: pointer to function of type fsl_defence_cb.
 * *
 * * Return: void
 * *
 * * Description
 * * Create a thread for IOCTL of WSRSR bitmask
 * * If bitmask is non zero calls recovery function
 * * else IOCTL sleeps on waitqueqe
 * **********************************************************************/
void fsl_L1_defense_register_cb(fsl_defense_cb cb);

#endif /* FSL_913x_IPC_H_ */
