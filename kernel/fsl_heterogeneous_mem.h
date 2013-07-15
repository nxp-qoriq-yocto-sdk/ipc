/*
 * Copyright (c) 2013 Freescale Semiconductor Inc.
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
 */


/******************************************************************************

 @File          fsl_heterogeneous_mem.h

 @Description   Memory manager between SC and PA cores.

 @Cautions      None.

*//***************************************************************************/

#ifndef __FSL_HETEROGENEOUS_MEM_H
#define __FSL_HETEROGENEOUS_MEM_H

#include "fsl_heterogeneous_common.h"

/**************************************************************************//**
 @Description   Memory Descriptor structure.

		This structure will be used to describe a shared memory slab.

*//***************************************************************************/
#ifdef B913x
typedef struct {
    /* Indicates the start address of a memory region */
    uintptr_t   start_addr;
    /* Indicates the size of a memory region */
    uint32_t    size;
} os_het_mem_t;
#else /*B4860 */
typedef struct {
    /* Indicates the start address of a memory region */
    uint64_t   start_addr;
    /* Indicates the size of a memory region */
    uint64_t    size;
} os_het_mem_t;
#endif
#endif /*__FSL_HETEROGENEOUS_MEM_H*/
