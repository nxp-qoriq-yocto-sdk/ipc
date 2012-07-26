/*
 * @fsl_usmmgr.h
 *
 * Copyright (c) 2011-2012 Freescale Semiconductor Inc.
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
 */
#ifndef IPC_HELPER_H
#define IPC_HELPER_H
#include "fsl_types.h"
#include "fsl_ipc_shm.h"

typedef void *fsl_usmmgr_t;
/*****************************************************************************
 * @fsl_usmmgr_alloc
 *
 * Perform Memory allocation from shared area.
 *
 * r:
 *    size[in]- Memory size that needs to be allocated.
 *    vaddr[out] - virtual address of allocated memory.
 *    phys_addr[out] - physical address of allocated memory.
 *
 * usmmgr - handle returned by fsl_usmmgr_init.
 *
 * Return: On Sucess, zero is returned
 *          On Failure, less than zero is returned.
*****************************************************************************/
int fsl_usmmgr_alloc(range_t *r, fsl_usmmgr_t usmmgr);

/*****************************************************************************
 * @fsl_usmmgr_memalign
 *
 * Perform Aligned Memory Allocation from pa_shared_area
 *
 * r:
 *    size[in] - Memory size that needs to be allocated.
 *    vaddr[out] - virtual address of allocated memory.
 *    phys_addr[out] - physical address of allocated memory.
 *
 * usmmgr - handle returned by fsl_usmmgr_init.
 *
 * Return: On Sucess, zero is returned
 *          On Failure, less than zero is returned.
*****************************************************************************/
int fsl_usmmgr_memalign(range_t *r, unsigned long align, fsl_usmmgr_t usmmgr);

/*****************************************************************************
 * @fsl_usmmgr_free
 *
 * Frees the memory allocated using fsl_usmmgr_alloc, fsl_usmmgr_memalign.
 *
 * r:
 *    vaddr[in] - virtual address of allocated memory.
 *
 * usmmgr - handle returned by fsl_usmmgr_init.
 *
*****************************************************************************/
void fsl_usmmgr_free(range_t *r, fsl_usmmgr_t usmmgr);

/*****************************************************************************
 * @fsl_usmmgr_init
 *
 * Initialize the ipc helper memory management subsystem.
 * Any application which calls this API, will map the complete TLB1 in its
 * virtual address space
 *
 * Return Value:
 *	ERR_SUCCESS as pass, non zero value as failure
*****************************************************************************/
fsl_usmmgr_t fsl_usmmgr_init();

/*****************************************************************************
 * @fsl_usmmgr_exit
 *
 * Deinit the ipc helper memory management subsystem.
 *
 * usmmgr - handle returned by fsl_usmmgr_init.
 *
*****************************************************************************/
int fsl_usmmgr_exit(fsl_usmmgr_t usmmgr);

/*****************************************************************************
 * @fsl_usmmgr_p2v
 *
 * Returns the virtual address of a physical address passed as argument.
 * The p2v mapping suported is for
 *			- dsp_m2
 *			- pa_shared_area
 *			- dsp_shared_area
 *
 * usmmgr - handle returned by fsl_usmmgr_init.
 *
*****************************************************************************/
void *fsl_usmmgr_p2v(phys_addr_t, fsl_usmmgr_t usmmgr);

/*****************************************************************************
 * @fsl_usmmgr_v2p
 *
 * Perform virtual to physical address translation.
 *
 * vaddr[in] - virtual address to be traslated
 * usmmgr[in] - handle returned by fsl_usmmgr_init.
 *
 * Return: On Sucess, physical address is returned
 *         On Failure, zero value is returned.
 *
******************************************************************************/
phys_addr_t fsl_usmmgr_v2p(void *vaddr, fsl_usmmgr_t usmmgr);

/*****************************************************************************
 * @get_pa_shared_area
 *
 * r	[out] parameter in which the range is returned
 *
 * usmmgr - handle returned by fsl_usmmgr_init.
 *
 * Return Value:
 *	ERR_SUCCESS as pass, non zero value as failure
*****************************************************************************/
int get_pa_shared_area(range_t *r, fsl_usmmgr_t usmmgr);

/*****************************************************************************
 * @get_pa_ccsr_area
 *
 * Returns the range_t for pa_ccsr area
 *
 * r	[out] parameter in which the range is returned
 *
 * usmmgr - handle returned by fsl_usmmgr_init.
 *
 * Return Value:
 *	ERR_SUCCESS as pass, non zero value as failure
*****************************************************************************/
int get_pa_ccsr_area(range_t *r, fsl_usmmgr_t usmmgr);

/*****************************************************************************
 * @get_dsp_ccsr_area
 *
 * Returns the range_t for dsp_ccsr area
 *
 * r	[out] parameter in which the range is returned
 *
 * usmmgr - handle returned by fsl_usmmgr_init.
 *
 * Return Value:
 *	ERR_SUCCESS as pass, non zero value as failure
*****************************************************************************/
int get_dsp_ccsr_area(range_t *r, fsl_usmmgr_t usmmgr);

/*****************************************************************************
 * @get_shared_ctrl_area
 *
 * Returns the range_t for the shared control area
 *
 * r	[out] parameter in which the range is returned
 *
 * usmmgr - handle returned by fsl_usmmgr_init.
 *
 * Return Value:
 *	ERR_SUCCESS as pass, non zero value as failure
*****************************************************************************/
int get_shared_ctrl_area(range_t *r, fsl_usmmgr_t usmmgr);

/***************************************************************************
 * @fsl_usmmgr_dump_memory
 *
 * Dump memory contents at specified location.
 *
 * mem_dump_buf[in] - virtual address of buffer where memory contents
 * Needs to be dumped.
 *
 * size[in] - Maximum size of the memory contents that needs to be dumped
 *
 * Return - On success, number of bytes dumped into mem_dump_buf
 *          On Failure, less than zero is returned.
 * ************************************************************************/
int fsl_usmmgr_dump_memory(void *, size_t);


#endif
