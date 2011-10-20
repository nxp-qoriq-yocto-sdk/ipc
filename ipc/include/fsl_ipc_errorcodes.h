/*
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
#endif /* FSL_PSC913X_IPC_ERRORCODES_H_ */
