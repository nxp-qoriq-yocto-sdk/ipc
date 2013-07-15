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

 @File          fsl_heterogeneous_debug.h

 @Description   Debug shared header file between SC and PA cores.

*//***************************************************************************/

#ifndef __FSL_HETEROGENEOUS_DEBUG_H
#define __FSL_HETEROGENEOUS_DEBUG_H


#include "fsl_heterogeneous_common.h"

/**************************************************************************/
/*@Description   SmartDSP Event log*/

/***************************************************************************/
#ifdef B913x
typedef struct {
    /* Pointer to the core's event log(physical address) */
    uintptr_t   base_address;
    /* Size of the event log (in bytes) */
    uint32_t    size;
    /* Pointer to the last error in SmartDSP. A value of 1 (OS_SUCCESS)
	* means no error */
    uint32_t	*last_error;
} os_het_smartdsp_log_t;
#else /* B4860 */
typedef struct {
    /* Pointer to the core's event log(physical address) */
    uint64_t   base_address;
    /* Size of the event log (in bytes) */
    uint32_t    size;
    /* Pointer to the last error in SmartDSP. A value of 1 (OS_SUCCESS)
	* means no error */
    uint64_t	last_error;
} os_het_smartdsp_log_t;
#endif
#endif /*__FSL_HETEROGENEOUS_DEBUG_H*/
