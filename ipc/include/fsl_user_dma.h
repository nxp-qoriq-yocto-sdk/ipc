/*
 * @fsl_user_dma.h
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
 *	Author: Manish Jaggi <manish.jaggi@freescale.com>
 *	Author: Pankaj Chauhan <pankaj.chauhan@freescale.com>
 */
#include <stdint.h>
#include "fsl_types.h"
#define MAX_DMA_ENTRIES	16
/*****************************************************************************
 * @dma_regs_t
 *
 * DMA controller register space
 *
*****************************************************************************/
typedef struct {
	volatile uint32_t  mr;
	volatile uint32_t  sr;
	uint32_t  eclndar;
	uint32_t  clndar;
	uint32_t  satr;
	uint32_t  sar;
	uint32_t  datr;
	uint32_t  dar;
	uint32_t  bcr;
	uint32_t  reserved2;
	uint32_t  clabdar;
	uint32_t  reserved3;
	uint32_t  nlsdar;
	uint32_t  ssr;
	uint32_t  dsr;
} dma_regs_t;

/*****************************************************************************
 * @dma_list_t
 *
 * DMA descriptor data structure
 *
*****************************************************************************/

 typedef struct {
	uint32_t  sattr;
	phys_addr_t src;
	uint32_t  dattr;
	phys_addr_t dest;
	uint32_t  enlndar;
	uint32_t  nlndar;
	uint32_t  length;
	uint32_t  reserved;
 } dma_list_t;
/*****************************************************************************
 * @uspace_dma_t
 *
 *
*****************************************************************************/

typedef struct {
	volatile dma_regs_t	*dma;
	int 		list_index;
	range_t 	dma_list_mem;
	dma_list_t	*dma_list;
} uspace_dma_t;

/*****************************************************************************
 * @fsl_uspace_dma_init
 *
 * Initialize dma controller.
 *
 * dma_list_mem	-	The caller should reserve memory for the dma lib to
 * 			create desriptors. The physical and virtual address
 *			of the reserved memory is provided with dma_list_mem
 *
*****************************************************************************/
int fsl_uspace_dma_init(range_t dma_list_mem);
/*****************************************************************************
 * @fsl_uspace_dma_add_entry
 *
 * Initialize dma controller.
 *
 * src		-	physical address of src buffer
 *
 * dest		-	physical address of destination buffer
 *
 * length	- 	length of the src buffer
 *
*****************************************************************************/
int fsl_uspace_dma_add_entry(phys_addr_t src, phys_addr_t dest,
				uint32_t length);
/*****************************************************************************
 * @fsl_uspace_dma_start
 *
 * Start the DMA
 *
*****************************************************************************/
int fsl_uspace_dma_start(void);
/*****************************************************************************
 * @fsl_uspace_dma_busy
 *
 * Check if the DMA transfer is in process
 *
*****************************************************************************/
int fsl_uspace_dma_busy(void);
/*****************************************************************************
 * @fsl_uspace_dma_list_clear
 *
 * Clear the DMA descriptor list
 *
*****************************************************************************/
void fsl_uspace_dma_list_clear(void);
