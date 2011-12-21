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
#include<linux/fs.h>

/* For using Share Memory Allocator, Application MUST include fsl_ipc_shm.h */
#include "fsl_ipc_shm.h"

int
main(int argc, char *argv[], char *envp[])
{
	void *ret;
	unsigned int *ptr, *ptr1;
	unsigned int *paddr, *vaddr;

	/*
	 * Before using any allocation(shm_alloc), de-allocation(shm_free) API
	 * application MUST call fsl_shm_init API.
	 *
	 * fsl_shm_init needs to be called, ONLY ONCE, during the initialization
	 * of application.
	 */
	ret = fsl_shm_init(0);
	if (!ret) {
		printf("Intialization of Shared Memory Allocator Failed\n");
		return -1;
	}

	/*
	 * shm_alloc - can be used for allocation of shared memory.
	 * On Sucess, it will return the virtual address of allocated memory.
	 * On Failure, It returns NULL.
	 */
	ptr = (unsigned int *)shm_alloc(0x100000);
	if (!ptr) {
		printf("Memory Allocation Failed through shm_alloc\n");
		return -1;
	}

	*ptr = 0xdeadbeaf;

	/*
	 * shm_vtop - Used to translate virtual address to its corresponding
	 * physical address of allocated shared memory.
	 */
	paddr = shm_vtop(ptr);

	/*
	 * shm_ptov - Used to translate physical address to its corresponding
	 * virtual address of allocated shared memory.
	 */
	vaddr = shm_ptov(paddr);

	printf("shm_alloc: vaddr %p paddr %p\n", vaddr, paddr);

	/*
	 * shm_memalign - Used for allocation of aligned shared memory area.
	 * On Sucess, Return virtual address of aligned allocated memory.
	 * On Failure, It returns NULL.
	 */
	ptr1 = (unsigned int *)shm_memalign(0x100000, 0x1000000);
	if (!ptr1) {
		printf("Aligned Memory Allocation Failed using shm_memalign\n");
		return -1;
	}

	printf("shm_memalign: vaddr %p paddr %p\n", ptr1, shm_vtop(ptr1));

	/*
	 * shm_free - It free/de-allocates the memory allocated using shm_alloc
	 * shm_memalign in order to further allocate it again.
	 */
	shm_free(ptr);
	shm_free(ptr1);

	printf("Apps finished\n");

	return 0;
}
