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

#include <stdio.h>
#include <fcntl.h>
#include <sys/shm.h>
#include <sys/ioctl.h>
#include<linux/fs.h>
#include <string.h>
#include <unistd.h>

#include "lg_shm.h"

/* Shared Memory Segment */
shm_seg_t shm;

/* file descriptor */
int fd;

static enum bool shm_init_done = FALSE;

/*
 * shm_vtop - Perform virtual to physical address translation.
 * ptr[in] - virtual address, to be translated
 * Return - physical address for valid physical address, otherwise NULL.
 */
void *shm_vtop(void *ptr)
{
	if (shm_init_done == FALSE)
		return NULL;

	if (!ptr || (ptr < shm.vaddr) || (ptr > (shm.vaddr + shm.size)))
		return NULL;

	return shm.paddr + (ptr - shm.vaddr);
}

/*
 * shm_ptov - Perform physical to virtual address translation.
 * ptr[in] - physical address, to be translated
 * Return - virtual address for valid physical address, otherwise NULL.
 */
void *shm_ptov(void *ptr)
{
	if (shm_init_done == FALSE)
		return NULL;

	if (!ptr || (ptr < shm.paddr) || (ptr > (shm.paddr + shm.size)))
		return NULL;

	return shm.vaddr + (ptr - shm.paddr);
}

/*
 * shm-memalign - Performs address aligned shared memory allocation.
 * size [in] - size of memory that needs to be allocated.
 * align[in] - Alignment of starting address.
 * Return - On sucess, Aligned virtual address of allocated shared memory.
 *          On Failure, NULL.
 */
void *shm_memalign(size_t size, unsigned long align)
{
	memalign_req_t req;

	if (shm_init_done == FALSE)
		return NULL;

	memset(&req, 0, sizeof(memalign_req_t));
	req.size = size;
	req.align = align;
	ioctl(fd, IOCTL_FSL_SHM_MEMALIGN, &req);

	return shm_ptov(req.paddr);
}

/*
 * shm_alloc - Perform shared memory allocation.
 * size[in] - size of memory that needs to be allocated.
 * Retruns - On sucess, virtual address of allocated shared memory.
 *           On Failure, NULL.
 */
void *shm_alloc(size_t size)
{
	alloc_req_t req;

	if (shm_init_done == FALSE)
		return NULL;

	memset(&req, 0, sizeof(alloc_req_t));
	req.size = size;
	ioctl(fd, IOCTL_FSL_SHM_ALLOC, &req);

	return shm_ptov(req.paddr);
}

/*
 * shm_free - Frees shared memory allocated using shm_alloc, shm_memalign.
 * ptr[in] - address of allocated shared memory area.
 */
void shm_free(void *ptr)
{
	unsigned long addr;
	addr = (unsigned long)shm_vtop(ptr);
	if (!addr)
		return;

	ioctl(fd, IOCTL_FSL_SHM_FREE, &addr);
}

/*
 * fsl_shm_init - Initializses Application for using Shared Memory Allocator.
 * dsp_shared_size[in] - dsp shared area size.
 * Returns - On sucess, virtual address of shared memory.
 *           On failure, NULL.
 */
void *fsl_shm_init(size_t dsp_shared_size)
{
	int shmid;

	fd = open(DEV_FILE, O_RDONLY);
	if (fd < 0) {
		perror(DEV_FILE);
		return NULL;
	}

	shmid = shmget(LG_SHM_KEY, HUGE_PAGE_256M,
		       SHM_HUGETLB | IPC_CREAT | SHM_R | SHM_W);
	if (shmid < 0) {
		perror("shmget");
		close(fd);
		return NULL;
	}

	memset(&shm, 0, sizeof(shm_seg_t));
	shm.vaddr = shmat(shmid, 0, 0);
	if (shm.vaddr == (char *)-1) {
		perror("Shared memory attach failure");
		shmctl(shmid, IPC_RMID, NULL);
		close(fd);
		return NULL;
	}

	shm.size = HUGE_PAGE_256M - dsp_shared_size;
	ioctl(fd, IOCTL_FSL_SHM_INIT, &shm);

	shm_init_done = TRUE;

	return shm.paddr;
}
