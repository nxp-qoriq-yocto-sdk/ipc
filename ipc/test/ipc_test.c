/*
 * Copyright 2011-2013 Freescale Semiconductor, Inc.
 *
 * Author: Ashish Kumar <ashish.kumar@freescale.com>
 * Author: Manish Jaggi <manish.jaggi@freescale.com>
 */
#include <stdio.h>
#include <stdlib.h>
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
int ch3init;
int ch4init;
void test_init(int rat_id);
int rat_id;

/* Logging Function */
void dump_msg(char *msg, int len)
{
	int i = 0;
	for (i = 0; i < len; i++)
		printf("%x", msg[i]);

	printf("\n");
}

int main(int argc, char **argv)
{
	if (argc > 2) {
		printf("Usage:\n %s <rat_id>\n", argv[0]);
		printf("OR \nUsage:\n %s\n", argv[0]);
		goto end;
	}
	if (argc == 1)
		rat_id = 0;
	else if (argc == 2)
		rat_id = atoi(argv[1]);

	test_init(rat_id);
end:
	return 0;
}

///////////////// Create Channels ////////////////////
int isbitset(uint64_t v, int bit)
{
	if ((v >> (63 - bit)) & 0x1)
		return 1;

	return 0;
}

void channel3_thread(void *ptr)
{
	phys_addr_t p;
	uint32_t len;
	int ret;
	int ctr = 0;
	int depth = 16;
	char bs[20] = { 0 };
	ENTER();
	ret = fsl_ipc_configure_channel(3, depth, IPC_PTR_CH, 0, 0, NULL, ipc);
	if (ret) {
		printf("\n ret %x \n", ret);
		EXIT(ret);
		pthread_exit(0);
	}
	ch3init = 1;
	while (1) {
		do {
			ret = fsl_ipc_recv_ptr(3, &p, &len, ipc);
			usleep(1000);
		} while (ret == -ERR_CHANNEL_EMPTY);
		if (ret) {
			ENTER();
			printf("\n ERROR ret %x \n", ret);
			goto end;
		}
		printf("\n[IPC_PA%d] R:C3:P:[%lx]L:%x\n", rat_id, p, len);

		sprintf(bs, "%x ", ctr++);
		do {
			ret = fsl_ipc_send_msg(2, bs, 9, ipc);
		} while (ret == -ERR_CHANNEL_FULL);
		if (ret) {
			printf("\n ret %x \n", ret);
			goto end;
		}
		printf("\n[IPC_PA%d] S:C2:M:L:%x\n", rat_id, 9);
	}
end:
	printf("Exiting thread for ch2/3\n");
	EXIT(ret);
	pthread_exit(0);
}
void channel4_thread(void *ptr)
{
	phys_addr_t p;
	uint32_t len;
	int ret;
	int depth = 16;
	char retbuf[1024];
	void *vaddr;
	ENTER();
	ret = fsl_ipc_configure_channel(4, depth, IPC_PTR_CH, 0, 0, NULL, ipc);
	if (ret) {
		printf("\n ret %x \n", ret);
		EXIT(ret);
		pthread_exit(0);
	}
	ch4init = 1;
	while (1) {
		do {
			ret = fsl_ipc_recv_ptr(4, &p, &len, ipc);
			usleep(1000);
		} while (ret == -ERR_CHANNEL_EMPTY);
		if (ret) {
			printf("\n ERROR ret %x \n", ret);
			goto end;
		}
		printf("\nR:C4:P:[%lx]L:%x\n", p, len);

		vaddr = fsl_usmmgr_p2v(p, usmmgr);
		if (!vaddr) {
			ret = -ERR_NULL_VALUE;
			printf("\n Error in translating physical address %lx"
			       " to virtual address\n", p);
			goto end;
		}

		memcpy(retbuf, vaddr, len);
		do {
			ret = fsl_ipc_send_msg(5, retbuf, len, ipc);
			usleep(1000);
		} while (ret == -ERR_CHANNEL_FULL);
		if (ret) {
			printf("Error code = %x\n", ret);
			goto end;
		}
		printf("\nS:C5:M:L:%x\n", len);
	}
end:
	printf("Exiting Thread for ch4/5\n");
	EXIT(ret);
	pthread_exit(0);
}

void *test_p2v(phys_addr_t phys_addr)
{
	return fsl_usmmgr_p2v(phys_addr, usmmgr);
}

void test_init(int rat_id)
{
	uint64_t bmask;
	int ret = 0;
	int ret1, ret2;
	char *buf = "Hello DSP.";
	pthread_t thread1, thread2;
	range_t sh_ctrl;
	range_t dsp_ccsr;
	range_t pa_ccsr;

	ENTER();
	printf("\n=========$IPC TEST$====%s %s====\n", __DATE__, __TIME__);

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

	if (rat_id == 0) {
		/* use of fsl_ipc_init is deprecated
		* Instead use fsl_ipc_init_rat with rat_id 0,
		* for non-MULTI RAT scenarios*/
		ipc = fsl_ipc_init(
			test_p2v, sh_ctrl, dsp_ccsr, pa_ccsr);
	} else {
		ipc = fsl_ipc_init_rat(
			rat_id,
			test_p2v, sh_ctrl, dsp_ccsr, pa_ccsr);
	}

	if (!ipc) {
		printf("Issue with fsl_ipc_init %d\n", ret);
		return;
	}
	do {
		fsl_ipc_chk_recv_status(&bmask, ipc);
		printf("\n main loop #ret %llx \n", bmask);
		usleep(10000);
	} while (!(isbitset(bmask, 0)));

	fsl_ipc_open_prod_ch(2, ipc);
	fsl_ipc_open_prod_ch(5, ipc);

	printf("Trying to start a thread\n");
	ret1 = pthread_create(&thread1, NULL, (void *)&channel3_thread, NULL);
	if (ret1) {
		printf("pthread_create returns with error: %d", ret1);
		return;
	}

	ret2 = pthread_create(&thread2, NULL, (void *)&channel4_thread, NULL);
	if (ret2) {
		printf("pthread_create returns with error: %d", ret2);
		return;
	}

	printf("ptherad_create %d %d\n", ret1, ret2);

	while (!(ch3init && ch4init)) {
		printf(".");
		usleep(1000);
	}
	printf("Trying to send message on ch#2\n");
	ret = fsl_ipc_send_msg(2, buf, 10, ipc);
	if (ret)
		printf("Issue with fsl_ipc_send_msg %d\n", ret);

	printf("\n[IPC_PA] S:C2:M:L:%x\n", 10);
	pthread_join(thread1, NULL);
	pthread_join(thread2, NULL);
	EXIT(0);
}
