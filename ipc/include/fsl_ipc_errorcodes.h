/*
 * Copyright 2011-2012 Freescale Semiconductor, Inc.
 *
 * Author: Manish Jaggi <manish.jaggi@freescale.com>
 */

#ifndef FSL_PSC913X_IPC_ERRORCODES_H_
#define FSL_PSC913X_IPC_ERRORCODES_H_

#define ERR_SUCCESS	 			0
#define ERR_CHANNEL_NOT_FOUND			1
#define ERR_CHANNEL_BUSY			2
#define ERR_CHANNEL_FULL			3
#define ERR_INVALID_SIZE			4
#define ERR_CHANNEL_EMPTY			5
/*TX_REQ_STATUS ERROR MESSAGES */
#define TXREQ_DONE				6
#define TXREQ_IN_PROCESS			7
#define TXREQ_ERR				8

#define ERR_INVALID_DEPTH			9
#define ERR_NULL_MEMORY				10
#define ERR_NO_SIGNAL_FOUND			11
#define ERR_IOCTL_FAIL				12
#define	ERR_CALLOC				13

#define ERR_DEV_HETIPC_FAIL			14
#define ERR_P2V_NULL				15
#define ERR_DEV_MEM_MMAP_FAIL			16
#define ERR_DEV_MEM_FAIL			17
#define ERR_DEV_HETMGR_FAIL			18
#define ERR_DMA_MMAP_FAILED			19
#define ERR_DMA_LIST_FULL			20
#define ERR_DMA_BUSY				21
#define	ERR_DEV_HW_SEM_FAIL			22
#define ERR_FAIL				23
#define ERR_NULL_VALUE				24
#define ERR_PRODUCER_NOT_INIT			25
#define ERR_CONSUMER_NOT_INIT			26
#define ERR_INCORRECT_RAT_MODE			27
#define ERR_MAX_MSG_SIZE_EXCD			28
#define ERR_MAX_MSG_SIZE_EXCD_TXREQ		29
/*
 * ERR_L1_DEFENSE_API_FAIL			30
 * used in dsp_boot.h
 */
#endif /* FSL_PSC913X_IPC_ERRORCODES_H_ */
