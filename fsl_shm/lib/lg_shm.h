/*
 * Copyright (C) 2011 Freescale Semiconductor, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN
 * NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
 * TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *	Author: Naveen Burmi <naveenburmi@freescale.com>
 */

#ifndef __LG_SHM_H
#define __LG_SHM_H

#define DEV_FILE	"/dev/fsl_shm"
#define HUGE_PAGE_256M	0x10000000
#define LG_SHM_KEY	2

enum bool {
	TRUE, FALSE,
};

typedef struct shm_seg {
	__u32 vaddr;
	__u32 paddr;
	__u32 size;
} shm_seg_t;

typedef struct alloc_req {
	__u32 paddr;
	__u32 size;
} alloc_req_t;

typedef struct memalign_req {
	__u32 paddr;
	__u32 align;
	__u32 size;
} memalign_req_t;

#define FSL_MM_MAGIC	'F'
#define IOCTL_FSL_SHM_INIT	_IOWR(FSL_MM_MAGIC, 1, __u64)
#define IOCTL_FSL_SHM_ALLOC	_IOWR(FSL_MM_MAGIC, 2, __u64)
#define IOCTL_FSL_SHM_MEMALIGN	_IOWR(FSL_MM_MAGIC, 3, __u64)
#define IOCTL_FSL_SHM_FREE	_IOR(FSL_MM_MAGIC, 4, __u64)

#endif
