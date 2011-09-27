/*
 * @fsl_user_dma.c
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
 *		Author: Pankaj Chauhan <pankaj.chauhan@freescale.com>
 */
#include <stdio.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <stdlib.h>
#include <unistd.h>
#include "fsl_user_dma.h"
#include "logdefs.h"
#include "fsl_ipc_errorcodes.h"

#define DMA_ADDR        0x21000
#define DMA_REG_OFFSET  0x100
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

uspace_dma_t dma_priv;
#define VTOP(A)		((A) - &dma_priv.dma_list[i+1] + \
					dma_priv.dma_list_mem.phys_addr)

int fsl_uspace_dma_init(range_t dma_list_mem)
{
	int i;
	void *dma;
	range_t r;
	int ret = ERR_SUCCESS;
	ENTER();
	DUMPR(&dma_list_mem);

	ret = get_pa_ccsr_area(&r);
	if (ret) {
		EXIT(ret);
		return ret;
	}
	dma = r.vaddr + DMA_ADDR;
	printf("dma addr = %x\n", (uint32_t)dma);
	dma_priv.dma = (volatile dma_regs_t *)((unsigned long)dma + DMA_REG_OFFSET);
	dma_priv.dma_list = (dma_list_t *)dma_list_mem.vaddr;
	fsl_uspace_dma_list_clear();

	dma_priv.dma->clndar = dma_list_mem.phys_addr;
	printf("Setting dma_priv.dma->clndar = %x\n", dma_list_mem.phys_addr);
	memcpy(&dma_priv.dma_list_mem, &dma_list_mem, sizeof(range_t));
	printf("Setting dma_priv.dma->clndar = %x\n", dma_priv.dma_list_mem.phys_addr);
end:
	EXIT(ret);
	return ret;
}



int fsl_uspace_dma_add_entry(phys_addr_t src, phys_addr_t dest, uint32_t length)
{
	void *dma_curr;
	ENTER();
	printf("S=%x D=%x L=%x\n", src, dest, length);
	int i = dma_priv.list_index;
	if (i == MAX_DMA_ENTRIES) {
		printf("Exceeded Mac number of DMA entries !!\n");
		EXIT(-ERR_DMA_LIST_FULL);
		return -ERR_DMA_LIST_FULL;
	}

	dma_priv.dma_list[i].src = src;
	dma_priv.dma_list[i].dest = dest;
	dma_priv.dma_list[i].length = length;
	dma_priv.dma_list[i].nlndar = (uint32_t)(
			/* (uint32_t) VTOP(&dma_priv.dma_list[i+1]) | 0x10; */
	dma_priv.dma_list_mem.phys_addr + (i+1)*sizeof(dma_list_t));
	dma_curr = &dma_priv.dma_list[dma_priv.list_index];
	printf("DMA ENTRY Addr=%x S=%x D=%x L=%x NL=%x \n",
		dma_priv.dma_list_mem.phys_addr + (i)*sizeof(dma_list_t) ,
		dma_priv.dma_list[i].src, dma_priv.dma_list[i].dest,
		dma_priv.dma_list[i].length, dma_priv.dma_list[i].nlndar);
	asm volatile ("dcbf 0,%0" : : "r"(dma_curr));

	dma_priv.list_index++;
	EXIT(0);
	return 0;
}

int fsl_uspace_dma_busy()
{
	int ret;
	ENTER();
	if ((dma_priv.dma->sr & DMA_BUSY) == DMA_BUSY)
		ret = 1;
	else
		ret = 0;

	EXIT(ret);
	return ret;
}

int fsl_uspace_dma_start(void)
{
	unsigned int dma_err;
	void *dma_curr;

	ENTER();
	if (fsl_uspace_dma_busy())
		return ERR_DMA_BUSY;

    dma_err = dma_priv.dma->sr & DMA_ERRORS;
    if (dma_err) {
	dma_priv.dma->sr |= dma_err;
    }

    dma_priv.dma->mr = DMA_STOP;
	dma_priv.list_index -= 1;

    dma_priv.dma_list[dma_priv.list_index].nlndar = 0x1;
    dma_curr = &dma_priv.dma_list[dma_priv.list_index];

    asm volatile ("dcbf 0,%0" : : "r"(dma_curr));
    asm("isync");
	asm("msync");
	asm("sync");

    dma_priv.dma->eclndar = 0;
	printf("Setting dma_priv.dma->clndar = %x\n",
	dma_priv.dma_list_mem.phys_addr);
    dma_priv.dma->clndar = dma_priv.dma_list_mem.phys_addr;
    asm("isync");
    asm("isync");
    dma_priv.dma->mr = DMA_START;
    asm("isync");
	asm("msync");
	asm("sync");
	/*while(dma_busy());*/
    asm("isync");

	EXIT(0);
	return 0;
}

void fsl_uspace_dma_list_clear(void)
{
	int i = 0;
	ENTER();
	dma_priv.list_index = 0;
	memset(&dma_priv.dma_list[0], 0,
		(sizeof(dma_list_t) * MAX_DMA_ENTRIES));

	for (i = 0; i < MAX_DMA_ENTRIES; i++) {
		dma_priv.dma_list[i].sattr = DMA_ATTR;
		dma_priv.dma_list[i].dattr = DMA_ATTR;
		dma_priv.dma_list[i].enlndar = 0;
	}
	EXIT(0);
}
