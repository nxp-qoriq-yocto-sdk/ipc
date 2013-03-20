/*
 * @fsl_ipc_helper.h
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
 */
#ifndef IPC_HELPER_H
#define IPC_HELPER_H
#include "fsl_ipc_types.h"

/*****************************************************************************
 * @fsl_ipc_helper_init
 *
 * Initialize the ipc helper memory management subsystem.
 *
 * Return Value:
 *	ERR_SUCCESS as pass, non zero value as failure
*****************************************************************************/
int fsl_ipc_helper_init();

/*****************************************************************************
 * @fsl_ipc_helper_p2v
 *
 * Returns the virtual address of a physical address passed as argument.
 * The p2v mapping suported is for
 *			- dsp_m2
 *			- pa_shared_area
 *			- dsp_shared_area
 *
*****************************************************************************/
void *fsl_ipc_helper_p2v(phys_addr_t);

/*****************************************************************************
 * @get_free_pool
 *
 * Returns a pool from pa_shared area.
 *
 * r	[out] parameter in which the range is returned
 *
 * Return Value:
 *	ERR_SUCCESS as pass, non zero value as failure
*****************************************************************************/
int get_free_pool(range_t *r);

/*****************************************************************************
 * @get_pa_ccsr_area
 *
 * Returns the range_t for pa_ccsr area
 *
 * r	[out] parameter in which the range is returned
 *
 * Return Value:
 *	ERR_SUCCESS as pass, non zero value as failure
*****************************************************************************/
int get_pa_ccsr_area(range_t *r);

/*****************************************************************************
 * @get_dsp_ccsr_area
 *
 * Returns the range_t for dsp_ccsr area
 *
 * r	[out] parameter in which the range is returned
 *
 * Return Value:
 *	ERR_SUCCESS as pass, non zero value as failure
*****************************************************************************/
int get_dsp_ccsr_area(range_t *r);

/*****************************************************************************
 * @get_shared_ctrl_area
 *
 * Returns the range_t for the shared control area
 *
 * r	[out] parameter in which the range is returned
 *
 * Return Value:
 *	ERR_SUCCESS as pass, non zero value as failure
*****************************************************************************/
int get_shared_ctrl_area(range_t *r);


#endif
