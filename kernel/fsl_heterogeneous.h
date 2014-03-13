/*
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
 */


/******************************************************************************

 @File          fsl_heterogeneous.h

 @Description   Main shared header file between SC and PA cores.

 @Cautions      None.

*//***************************************************************************/

#ifndef __FSL_HETEROGENEOUS_H
#define __FSL_HETEROGENEOUS_H


#include "fsl_heterogeneous_common.h"
#include "fsl_heterogeneous_ipc.h"
#include "fsl_heterogeneous_mem.h"
#include "fsl_heterogeneous_debug.h"
#include "fsl_heterogeneous_debug_print.h"
#include "fsl_heterogeneous_l1_defense.h"

#define OS_HET_IPC_HW_SEMAPHORE_NUM     0
/* Hardware semaphore to use in case of need for mutual exclusion
 * in the IPC module */
#define OS_HET_BOOT_HW_SEMAPHORE_NUM    1
/* Hardware semapohore for synchronizing the boot processes between
 * SC and PA */


/**************************************************************************
 @Description   Heterogeneous OS global control structure

**************************************************************************/
#ifdef B913x
typedef struct {
	os_het_init_t           initialized;
	/* Initialization indication strcuture */
	uint32_t				shared_ctrl_size;
	/* Size of the shared memory for control information in bytes -
	   starts at the base of the PA managed shared memory.
	   Mimumum (and default) size is 4 KB */
	os_het_mem_t            pa_shared_mem;
	/* PA shared memory region */
	os_het_mem_t            sc_shared_mem;
	/* SC shared memory region */

	os_het_ipc_t            (*ipc)[];

	void                    *aic;
	/* Pointer to shared AIC configuration control structure;
	 * as an offset from the base of the shared address space */
	os_het_smartdsp_log_t   (*smartdsp_debug)[];
	/* Pointer to where SmartDSP logs system events; PA initializes
	 * an array with the number of entries as there is SC cores*/
	os_het_debug_print_t    *het_debug_print;

#ifdef CONFIG_MULTI_RAT
	uint32_t                num_ipc_regions;
#endif
} os_het_control_t;
#else /* B4860 */

typedef struct {
    uint32_t		    start_validation_value;
    /* Initialization indication strcuture */
    os_het_init_t           initialized;
    /* SET BY PA: PA shared memory region; */
    os_het_mem_t            pa_shared_mem;
    /* SET BY DSP: SC shared memory region; */
    os_het_mem_t            sc_shared_mem;
    /* Pointer to array of os_het_ipc_t
     * Pointer IPC heterogeneous structure */
    uint64_t		    ipc;
    /* Pointer l1_defence pointer */
    uint64_t		    l1d;
    /* SET BY PA: Pointer to where SmartDSP logs system events
     * PA initializes an array with the number of entries as
     * there is SC cores
     * os_het_smartdsp_log_t */
    uint64_t                smartdsp_debug;
    os_het_debug_print_t    het_debug_print;
    /* SET BY DSP: Size of the shared memory for control information in
     * bytes - Mimumum size is 4 KB set by PA*/
    uint32_t                shared_ctrl_size;
    /* Number of IPC regions - only for multimode usages */
    uint32_t                num_ipc_regions;
    uint32_t		    end_validation_value;
} os_het_control_t;

#endif
extern os_het_control_t  *g_os_het_control;
/* Pointer to the base address of the heterogeneous OS control strcuture */

#endif /* __FSL_HETEROGENEOUS_H */
