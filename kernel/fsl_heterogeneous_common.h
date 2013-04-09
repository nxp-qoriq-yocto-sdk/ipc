/*
 * Copyright (c) 2011-2013
 * Freescale Semiconductor Inc.
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

 @File          fsl_heterogeneous_common.h

 @Description   Shared macros and includes between SC and PA cores.

 @Cautions      None.

*//***************************************************************************/

#ifndef __FSL_HETEROGENEOUS_COMMON_H
#define __FSL_HETEROGENEOUS_COMMON_H

/*************************************************************//**
 @Collection    Initialization indication values

 @{
*//**************************************************************/
/* The resource is initializaed */
#define OS_HET_INITIALIZED              0xFEDCBA98UL
/* The resource is initializaed for multimode work*/
#define OS_HET_INITIALIZED_MULTIMODE    0xEDCBA987UL
/* The resource is uninitialized */
#define OS_HET_UNINITIALIZED            0x00000000UL
/* @} */


/*************************************************************//**
 @Collection    Hardware/Software semaphore values

 @{
*//**************************************************************/
/* Sempahore is taken by the PA domain */
#define OS_HET_PA_SEMAPHORE_VAL         0xFF
/* Sempahore is taken by the SC domain */
#define OS_HET_SC_SEMAPHORE_VAL         0xFE
/* Semaphore is free */
#define OS_HET_FREE_SEMAPHORE_VAL       0x00
/* @} */

/* PASS / FAIL macro */
typedef enum {
    OS_HETERO_FAIL      = 0,
    OS_HETERO_SUCCESS   = 1
} os_het_status_t;


/**************************************************************************//**
 @Description   Initialization Control structure

	This structure will be used to indicatye that both OS
	domains	have initialized a specific structure

*//***************************************************************************/
typedef struct {
    /* Indicates whether the overall control structure is initialized
	(PA side) */
    uint32_t                pa_initialized;
    /* Indicates whether the overall control structure is initialized
	(SC side) */
    uint32_t                sc_initialized;
} os_het_init_t;

/**************************************************************************//**
 @Description   Producer/Consumer tracker

	The producer and consumer each perform counter++ to
	their counter. It is assumed the size of what the
	tracker is tracking is less than < MAX_UINT_32

 @Cautions      The counters must only be incremented, never decremented

*//***************************************************************************/
typedef struct {
    /* Number of items the producer produced */
    uint32_t      producer_num;
    /* Number of items the consumer consumed */
    uint32_t      consumer_num;
} os_het_tracker_t;


#define OS_HET_CALCULATE_ADDR(BASE, OFFSET) (void *)	\
				((uint8_t *)(BASE) + (uint32_t)(OFFSET))
/* Used by the various OS to calculate an address in it's
	own virtual address space  */

#endif /*__FSL_HETEROGENEOUS_COMMON_H*/
