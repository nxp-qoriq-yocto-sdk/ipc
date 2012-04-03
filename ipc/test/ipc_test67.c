/*
 * Copyright 2011-2012 Freescale Semiconductor, Inc.
 *
 * Author: Manish Jaggi <manish.jaggi@freescale.com>
 */
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include "fsl_usmmgr.h"
#include "fsl_bsc913x_ipc.h"
#include "fsl_ipc_errorcodes.h"

#define ENTER()	printf(">> %s %d %s\n", __FILE__, __LINE__, __func__)
#define EXIT(A)	printf("<< (%d) %s %d %s\n", A, __FILE__, __LINE__, __func__)

fsl_usmmgr_t usmmgr;
fsl_ipc_t ipc;
int ch7init;
void test_init();

/* Logging Function */
void dump_msg(char *msg, int len)
{
	int i = 0;
	for (i = 0; i < len; i++)
		printf("%x", msg[i]);

	printf("\n");
}

int main()
{
	test_init();

	return 0;
}

int isbitset(uint64_t v, int bit)
{
	if ((v >> (63 - bit)) & 0x1)
		return 1;

	return 0;
}

void channel67_thread(void *ptr)
{
	phys_addr_t p;
	uint32_t len;
	int ret;
	int depth = 16;
	int ctr;
	range_t r;
	sg_list_t lst;
	void *vaddr;
	int first = 1;
	char fapi_msg[1020];
	ENTER();
	ret = fsl_ipc_configure_channel(7, depth, IPC_PTR_CH, 0, 0, NULL, ipc);
	if (ret) {
		printf("\n ret %d \n", ret);
		EXIT(ret);
		pthread_exit(0);
	}
	ch7init = 1;

	r.size = 0x200000;
	ret = fsl_usmmgr_alloc(&r, usmmgr);
	if (ret) {
		printf("\n Unable to allocate memory from shm_alloc \n");
		EXIT(-1);
		pthread_exit(0);
	}

	printf("range of free pool P=%lx V=%p S=%x\n", r.phys_addr, r.vaddr,
	       r.size);
	ret = fsl_ipc_configure_txreq(6, r.phys_addr + 0x100000, 1024*2, ipc);
	if (ret) {
		printf("\n Error in fsl_ipc_configure_txreq ret %d \n", ret);
		goto end;
	}
	ctr = 1;

	while (1) {
		lst.entry[0].src_addr = r.phys_addr;
		lst.entry[0].len = 1024;
		lst.entry[0].is_valid = 1;
		vaddr = fsl_usmmgr_p2v(lst.entry[0].src_addr, usmmgr);
		if (!vaddr) {
			ret = -ERR_NULL_VALUE;
			printf("\nError in translating physical address %lx "
			       "to virtual address\n", lst.entry[0].src_addr);
			goto end;
		}

		memset(vaddr, 0x11 + ctr, 1024);

		lst.entry[1].src_addr = r.phys_addr + 1024 * 16;
		lst.entry[1].len = 1024;
		lst.entry[1].is_valid = 1;
		vaddr = fsl_usmmgr_p2v(lst.entry[1].src_addr, usmmgr);
		if (!vaddr) {
			ret = -ERR_NULL_VALUE;
			printf("\nError in translating physical address %lx "
			       "to virtual address\n", lst.entry[1].src_addr);
			goto end;
		}

		memset(vaddr, 0x22 + ctr, 1024);

		lst.entry[2].is_valid = 0;
		/* vaddr = fsl_usmmgr_p2v(lst.entry[1].src_addr, usmmgr); */
		memset(fapi_msg, ctr, 1020);
		do {
			if (!first)
				while (fsl_ipc_get_last_tx_req_status(ipc)
				       != TXREQ_DONE) {
					printf("*");
				}
			ret =
			    fsl_ipc_send_tx_req(6, &lst, &fapi_msg, 1020, ipc);
			first = 0;
		} while (ret == -ERR_CHANNEL_FULL);
		if (ret) {
			printf("\n ERROR ret %x \n", ret);
			goto end;
		}
		printf("\nS:C6:TXREQ\n");

		do {
			ret = fsl_ipc_recv_ptr(7, &p, &len, ipc);
			usleep(1000);
		} while (ret == -ERR_CHANNEL_EMPTY);
		if (ret) {
			printf("\n ERROR ret %x \n", ret);
			goto end;
		}
		printf("\nR:C7:P:[%lx]L:%x\n", p, len);
		ctr++;
	}
end:
	printf("Exit from Thread for ch6/7\n");
	EXIT(ret);
	pthread_exit(0);
}

void *test_p2v(phys_addr_t phys_addr)
{
	return fsl_usmmgr_p2v(phys_addr, usmmgr);
}

void test_init()
{
	int ret = 0;
	int ret3;
	pthread_t thread3;
	range_t sh_ctrl;
	range_t dsp_ccsr;
	range_t pa_ccsr;

	ENTER();
	printf("\n=========$IPC TEST Channel 67$====%s %s====\n", __DATE__,
	       __TIME__);

	usmmgr = fsl_usmmgr_init();
	if (!usmmgr) {
		printf("Error in Initializing User Space Memory Manager\n");
		return;
	}

	/* get values from mmgr */
	ret = get_pa_ccsr_area(&pa_ccsr, usmmgr);
	if (ret) {
		printf("Error in obtaining PA CCSR Area information\n");
		return;
	}

	ret = get_dsp_ccsr_area(&dsp_ccsr, usmmgr);
	if (ret) {
		printf("Error in obtaining DSP CCSR Area information\n");
		return;
	}

	ret = get_shared_ctrl_area(&sh_ctrl, usmmgr);
	if (ret) {
		printf("Error in obtaining Shared Control Area information\n");
		return;
	}

	ipc = fsl_ipc_init(test_p2v, sh_ctrl, dsp_ccsr, pa_ccsr);

	if (!ipc) {
		printf("Issue with fsl_ipc_init %d\n", ret);
		return;
	}
	fsl_ipc_open_prod_ch(6, ipc);

	printf("Trying to start a thread\n");
	ret3 = pthread_create(&thread3, NULL, (void *)&channel67_thread, NULL);
	if (ret3) {
		printf("pthread_create returns with error: %d", ret3);
		return;
	}

	printf("ptherad_create %d\n", ret3);

	while (!ch7init) {
		printf(".");
		usleep(1000);
	}

	pthread_join(thread3, NULL);
	EXIT(0);
}
