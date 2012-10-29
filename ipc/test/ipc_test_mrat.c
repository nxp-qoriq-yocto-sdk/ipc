/*
 * Copyright 2011-2012 Freescale Semiconductor, Inc.
 *
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
	if (argc < 2) {
		printf("Usage:\n %s <rat_id>\n", argv[0]);
		goto end;
	}
	rat_id = atoi(argv[1]);

	test_init(rat_id);
end:
	return 0;

}

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

void *test_p2v(phys_addr_t phys_addr)
{
	return fsl_usmmgr_p2v(phys_addr, usmmgr);
}

void test_init(int rat_id)
{
	int ret = 0;
	int ret1;
	char *buf = "Hello DSP.";
	pthread_t thread1;
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

	ipc = fsl_ipc_init(
			rat_id,
			test_p2v, sh_ctrl, dsp_ccsr, pa_ccsr);

	if (!ipc) {
		printf("Issue with fsl_ipc_init %d\n", ret);
		return;
	}

	fsl_ipc_open_prod_ch(2, ipc);
	fsl_ipc_open_prod_ch(5, ipc);

	printf("Trying to start a thread\n");
	ret1 = pthread_create(&thread1, NULL, (void *)&channel3_thread, NULL);
	if (ret1) {
		printf("pthread_create returns with error: %d", ret1);
		return;
	}

	while (ch3init) {
		printf(".");
		usleep(1000);
	}
	printf("Trying to send message on ch#2\n");
	ret = fsl_ipc_send_msg(2, buf, 10, ipc);
	if (ret)
		printf("Issue with fsl_ipc_send_msg %d\n", ret);

	printf("\n[IPC_PA] S:C2:M:L:%x\n", 10);
	pthread_join(thread1, NULL);
	EXIT(0);
}
