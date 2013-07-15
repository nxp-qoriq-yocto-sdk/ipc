/*
* Copyright (c) 2011-2013 Freescale Semiconductor, Inc.
*
* Freescale IPC and HetMgr Data structures
*
* Author: Manish Jaggi <manish.jaggi@freescale.com>
*
* This software is available to you under a choice of one of two
* licenses.  You may choose to be licensed under the terms of the GNU
* General Public License (GPL) Version 2, available from the file
* COPYING in the main directory of this source tree, or the BSD-type
* license below:
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
* 2. Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the
*    documentation and/or other materials provided with the distribution.
* 3. Neither the names of the copyright holders nor the names of its
*    contributors may be used to endorse or promote products derived from
*    this software without specific prior written permission.
*
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef _FSL_IPC_TYPES_H
#define _FSL_IPC_TYPES_H
#ifdef __KERNEL__
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/major.h>
#include <linux/fcntl.h>
#include <linux/types.h>
#else
#include <linux/types.h>
#endif

typedef struct {
	__u64 phys_addr;
	__u32	size;
} mem_strt_addr_t;

typedef struct {
	__u64 phys_addr;
	void    *vaddr;
	__u32 size;
} mem_range_t;

typedef struct {
	__u32	pa_shared_size;
	__u32	dsp_shared_size;
	mem_strt_addr_t		pa_ccsrbar;
	mem_strt_addr_t		dsp_ccsrbar;
	mem_strt_addr_t		linux_priv_area;
	mem_strt_addr_t		smart_dsp_os_priv_area;
	mem_strt_addr_t		sh_ctrl_area;
	mem_strt_addr_t		dsp_core0_m2;
	mem_strt_addr_t		dsp_core1_m2;
	mem_strt_addr_t		dsp_m3;
	mem_strt_addr_t		dbg_area;
} sys_map_t;

typedef struct {
	mem_strt_addr_t 	pa_ipc_shared;
	mem_strt_addr_t	dsp_ipc_shared;
	mem_strt_addr_t	pa_dbgprint_shared;
	mem_strt_addr_t	dsp_dbgprint_shared;
} shared_area_t;

typedef struct {
	__u32 	sem_no;
	__u32	value;
} hw_sem_t;

typedef struct {
	__u32	pa_uniq_val;
	__u32	dsp_uniq_val;
} hw_sem_info_t;

#endif
