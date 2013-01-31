/*
 * @fsl_user_dma.c
 *
 * Copyright (c) 2011-2013
 *  Freescale Semiconductor Inc.
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
 *      	Author: Manish Jaggi <manish.jaggi@freescale.com>
 *		Author: Pankaj Chauhan <pankaj.chauhan@freescale.com>
 *		Author: Ashish Kumar <ashish.kumar@freescale.com>
 */
#include <stdio.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include "fsl_user_dma.h"
#include "logdefs.h"
#include "fsl_ipc_errorcodes.h"

#define DMA_ADDR        0x21000
#define DMA_REG_OFFSET  0x100
#define DMA_CH_OFFSET	0x80
#define DMA_ATTR        0x00050000
#define DMA_DEST_ATTR	0x00040000
#define DMA_BUSY        4
/* #define DMA_START       0x08200003 */
#define DMA_START       0x08200001
#define DMA_STOP        0x08200000
#define DMA_BLOCK       0x80
#define DMA_BASIC_SINGLE_WRITE_MODE	(0x08200404)
#define DMA_CHAINED_SINGLE_WRITE_MODE	(0x08200030)
#define DMA_TRANSFER_ERR	(0x80)
#define DMA_PROGRAMMING_ERR	(0x10)
#define DMA_ERRORS	(DMA_TRANSFER_ERR | DMA_PROGRAMMING_ERR)
#define PAGE_SIZE       0x1000

#define VTOP(A)		((A) - &dma_priv->dma_list[i+1] + \
					dma_priv->dma_list_mem.phys_addr)
#define MAX_DMA_ENTRIES	144
/*****************************************************************************
 * @dma_regs_t
 *
 * DMA controller register space
 *
*****************************************************************************/
typedef struct {
	volatile uint32_t mr;
	volatile uint32_t sr;
	uint32_t eclndar;
	uint32_t clndar;
	uint32_t satr;
	uint32_t sar;
	uint32_t datr;
	uint32_t dar;
	uint32_t bcr;
	uint32_t reserved2;
	uint32_t clabdar;
	uint32_t reserved3;
	uint32_t nlsdar;
	uint32_t ssr;
	uint32_t dsr;
} dma_regs_t;

/*****************************************************************************
 * @dma_list_t
 *
 * DMA descriptor data structure
 *
*****************************************************************************/
typedef struct {
	uint32_t sattr;
	phys_addr_t src;
	uint32_t dattr;
	phys_addr_t dest;
	uint32_t enlndar;
	uint32_t nlndar;
	uint32_t length;
	uint32_t reserved;
} dma_list_t;
/*****************************************************************************
 * @uspace_dma_t
 *
 *
*****************************************************************************/
typedef struct {
	volatile dma_regs_t *dma;
	int list_index;
	range_t dma_list_mem;
	dma_list_t *dma_list;
} uspace_dma_t;

fsl_udma_t fsl_uspace_dma_init(range_t dma_list_mem, range_t pa_ccsr,
			uint32_t dma_ch_id)
{
	void *dma;
	ENTER();

	uspace_dma_t *dma_priv = malloc(sizeof(uspace_dma_t));
	if (!dma_priv)
		goto end;

	dma = pa_ccsr.vaddr + DMA_ADDR;
	debug_print("dma addr = %x\n", (uint32_t) dma);
	dma_priv->dma =
	    (volatile dma_regs_t *)((unsigned long)dma + DMA_REG_OFFSET
			+dma_ch_id*DMA_CH_OFFSET);
	dma_priv->dma_list = (dma_list_t *) dma_list_mem.vaddr;
	fsl_uspace_dma_list_clear(dma_priv);

	dma_priv->dma->clndar = dma_list_mem.phys_addr;
	debug_print("Setting dma_priv->dma->clndar = %x\n",
		    dma_list_mem.phys_addr);
	memcpy(&dma_priv->dma_list_mem, &dma_list_mem, sizeof(range_t));
	debug_print("Setting dma_priv->dma->clndar = %x\n",
		    dma_priv->dma_list_mem.phys_addr);
end:
	EXIT(0);
	return dma_priv;
}

void fsl_uspace_dma_exit(fsl_udma_t udma)
{
	uspace_dma_t *dma_priv = (uspace_dma_t *)udma;
	if (dma_priv)
		free(dma_priv);
}

int fsl_uspace_dma_add_entry(phys_addr_t src, phys_addr_t dest, uint32_t length,
			     fsl_udma_t udma)
{
	void *dma_curr;
	ENTER();
	uspace_dma_t *dma_priv = (uspace_dma_t *)udma;

	debug_print("S=%x D=%x L=%x\n", src, dest, length);
	int i = dma_priv->list_index;
	if (i == MAX_DMA_ENTRIES) {
		debug_print("Exceeded Mac number of DMA entries !!\n");
		EXIT(-ERR_DMA_LIST_FULL);
		return -ERR_DMA_LIST_FULL;
	}

	dma_priv->dma_list[i].sattr = DMA_ATTR;
	dma_priv->dma_list[i].dattr = DMA_ATTR;
	dma_priv->dma_list[i].enlndar = 0;
	dma_priv->dma_list[i].reserved = 0;
	dma_priv->dma_list[i].src = src;
	dma_priv->dma_list[i].dest = dest;
	dma_priv->dma_list[i].length = length;
	dma_priv->dma_list[i].nlndar = (uint32_t) (
							 dma_priv->dma_list_mem.
							 phys_addr + (i +
								      1) *
							 sizeof(dma_list_t));
	dma_curr = &dma_priv->dma_list[dma_priv->list_index];
	debug_print("DMA ENTRY Addr=%x S=%x D=%x L=%x NL=%x \n",
		    dma_priv->dma_list_mem.phys_addr + (i) * sizeof(dma_list_t),
		    dma_priv->dma_list[i].src, dma_priv->dma_list[i].dest,
		    dma_priv->dma_list[i].length, dma_priv->dma_list[i].nlndar);
	asm volatile ("dcbf 0,%0" : : "r" (dma_curr));

	dma_priv->list_index++;
	EXIT(0);
	return 0;
}

int fsl_uspace_dma_busy(fsl_udma_t udma)
{
	int ret;
	ENTER();
	uspace_dma_t *dma_priv = (uspace_dma_t *)udma;
	if ((dma_priv->dma->sr & DMA_BUSY) == DMA_BUSY)
		ret = 1;
	else
		ret = 0;

	EXIT(ret);
	return ret;
}

int fsl_uspace_dma_start(fsl_udma_t udma)
{
	unsigned int dma_err;
	void *dma_curr;

	ENTER();
	uspace_dma_t *dma_priv = (uspace_dma_t *)udma;

	if (fsl_uspace_dma_busy(udma))
		return -ERR_DMA_BUSY;

	dma_err = dma_priv->dma->sr & DMA_ERRORS;
	if (dma_err) {
		dma_priv->dma->sr |= dma_err;
	}

	dma_priv->dma->mr = DMA_STOP;
	dma_priv->list_index -= 1;

	dma_priv->dma_list[dma_priv->list_index].nlndar = 0x1;
	dma_curr = &dma_priv->dma_list[dma_priv->list_index];

	asm volatile ("dcbf 0,%0" : : "r" (dma_curr));
	asm("isync");
	asm("msync");
	asm("sync");

	dma_priv->dma->eclndar = 0;
	debug_print("Setting dma_priv->dma->clndar = %x\n",
		    dma_priv->dma_list_mem.phys_addr);
	dma_priv->dma->clndar = dma_priv->dma_list_mem.phys_addr;
	asm("isync");
	asm("isync");
	dma_priv->dma->mr = DMA_START;
	asm("isync");
	asm("msync");
	asm("sync");
	/*while(dma_busy()); */
	asm("isync");

	EXIT(0);
	return 0;
}

void fsl_uspace_dma_list_clear(fsl_udma_t udma)
{
	ENTER();

	uspace_dma_t *dma_priv = (uspace_dma_t *)udma;
	dma_priv->list_index = 0;

	EXIT(0);
}
