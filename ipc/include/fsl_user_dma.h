/*
 * @fsl_user_dma.h
 *
 * Copyright (c) 2011-2013 Freescale Semiconductor Inc.
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
 *	Author: Ashish Kumar <ashish.kumar@freescale.com>
 */
#ifndef _FSL_USER_DMA_H
#define _FSL_USER_DMA_H

#include <stdint.h>
#include "fsl_ipc_types.h"
typedef void *fsl_udma_t;
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
fsl_udma_t fsl_uspace_dma_init(mem_range_t dma_list_mem, mem_range_t pa_ccsr,
		uint32_t dma_ch_id, char uio_dev_buf[]);
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
int fsl_uspace_dma_add_entry(unsigned long src, uint64_t dest,
				uint32_t length, fsl_udma_t udma);
/*****************************************************************************
 * @fsl_uspace_dma_start
 *
 * Start the DMA
 *
*****************************************************************************/
int fsl_uspace_dma_start(fsl_udma_t udma);
/*****************************************************************************
 * @fsl_uspace_dma_busy
 *
 * Check if the DMA transfer is in process
 *
*****************************************************************************/
int fsl_uspace_dma_busy(fsl_udma_t udma);
/*****************************************************************************
 * @fsl_uspace_dma_list_clear
 *
 * Clear the DMA descriptor list
 *
*****************************************************************************/
void fsl_uspace_dma_list_clear(fsl_udma_t udma);
#endif
