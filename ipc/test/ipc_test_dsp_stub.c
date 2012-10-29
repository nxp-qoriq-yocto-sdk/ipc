/*
 * Copyright 2011-2012 Freescale Semiconductor, Inc.
 *
 * Author: Manish Jaggi <manish.jaggi@freescale.com>
 */
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
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
int ch2init;
int ch6init;
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

int converthtoi(char *String)
{
	unsigned long Value = 0, Digit;
	char c;

	while ((c = (char)*String++) != ' ') {

		if (c >= '0' && c <= '9')
			Digit = (unsigned long) (c - '0');
		else if (c >= 'a' && c <= 'f')
			Digit = (unsigned long) (c - 'a') + 10;
		else if (c >= 'A' && c <= 'F')
			Digit = (unsigned long) (c - 'A') + 10;
		else
			break;

		Value = (Value << 4) + Digit;
	}

	return Value;
}

int isbitset(uint64_t v, int bit)
{
	if ((v >> (63 - bit)) & 0x1)
		return 1;

	return 0;
}

void channel2_thread(void *ptr)
{
	phys_addr_t p;
	uint32_t len;
	int ret;
	int depth = 16;
	int msg_size = 20;
	char recv_buf[20] = { 0 };

	ENTER();

	range_t r;
	r.size = depth * msg_size;

	ret = fsl_usmmgr_alloc(&r, usmmgr);
	if (ret) {
		printf("\n Unable to allocate memory from shm_alloc \n");
		EXIT(-1);
		pthread_exit(0);
	}


	ret = fsl_ipc_configure_channel(2, depth, IPC_MSG_CH, r.phys_addr,
					msg_size, NULL, ipc);
	if (ret) {
		printf("\n ret %x \n", ret);
		EXIT(ret);
		pthread_exit(0);
	}
	ch2init = 1;
	while (1) {
		do {
			memset(recv_buf, 0, msg_size);
			ret = fsl_ipc_recv_msg(2, &recv_buf, &len, ipc);
			usleep(1000);
		} while (ret == -ERR_CHANNEL_EMPTY);
		if (ret) {
			printf("\n ERROR ret %x \n", ret);
			goto end;
		}
		printf("\n[IPC_DSP %d] R:C2:P:[%s]L:%x\n", rat_id, recv_buf,
			len);
		/*Recv a message number*/
		unsigned long num = converthtoi(recv_buf);
		p = num;
		/*Send that message number as a ptr */
		do {
			ret = fsl_ipc_send_ptr(3, p, 0, ipc);
		} while (ret == -ERR_CHANNEL_FULL);
		if (ret) {
			printf("\n ret %x \n", ret);
			goto end;
		}
		printf("\n[IPC_DSP %d] S:C3:P%x:L:%x\n", rat_id,
			(uint32_t)p, 0);
	}
end:
	printf("Exiting thread for ch2/3\n");
	EXIT(ret);
	pthread_exit(0);
}

void channel67_thread(void *ptr)
{
	uint32_t len;
	int ret;
	int depth = 16;
	int msg_size = 1024;
	char recv_buf[1024] = { 0 };

	ENTER();

	range_t r;
	r.size = depth * msg_size;

	ret = fsl_usmmgr_alloc(&r, usmmgr);
	if (ret) {
		printf("\n Unable to allocate memory from shm_alloc \n");
		EXIT(-1);
		pthread_exit(0);
	}


	ret = fsl_ipc_configure_channel(6, depth, IPC_MSG_CH, r.phys_addr,
					msg_size, NULL, ipc);
	if (ret) {
		printf("\n ret %x \n", ret);
		EXIT(ret);
		pthread_exit(0);
	}
	ch6init = 1;
	while (1) {
		do {
			memset(recv_buf, 0, msg_size);
			ret = fsl_ipc_recv_msg(6, &recv_buf, &len, ipc);
			usleep(1000);
		} while (ret == -ERR_CHANNEL_EMPTY);
		if (ret) {
			printf("\n ERROR ret %x \n", ret);
			goto end;
		}

		/*Print the 4 bytes from the message */


		/*Recv a message number*/
		unsigned long num = converthtoi(recv_buf);
		printf("\n[IPC_DSP TXREQ %d] R[%x]:C6:L:%x\n=>",
				rat_id, num, len);

#if 0
		/*Send that message number as a ptr */
		do {
			ret = fsl_ipc_send_ptr(7, p, 0, ipc);
		} while (ret == -ERR_CHANNEL_FULL);
		if (ret) {
			printf("\n ret %x \n", ret);
			goto end;
		}
		printf("\n[IPC_DSP %d] S:C3:P%x:L:%x\n", rat_id,
				(uint32_t)p, 0);
#endif
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

	ipc = fsl_ipc_init(
			rat_id,
			test_p2v, sh_ctrl, dsp_ccsr, pa_ccsr);

	if (!ipc) {
		printf("Issue with fsl_ipc_init %d\n", ret);
		return;
	}

	fsl_ipc_open_prod_ch(3, ipc);
	fsl_ipc_open_prod_ch(4, ipc);

	printf("Trying to start a thread\n");
	ret1 = pthread_create(&thread1, NULL, (void *)&channel2_thread, NULL);
	if (ret1) {
		printf("pthread_create returns with error: %d", ret1);
		return;
	}

	ret1 = pthread_create(&thread2, NULL, (void *)&channel67_thread, NULL);
	if (ret1) {
		printf("pthread_create returns with error: %d", ret1);
		return;
	}

	while (!ch2init || !ch6init) {
		printf(".");
		usleep(1000);
	}
	pthread_join(thread1, NULL);
	pthread_join(thread2, NULL);
	EXIT(0);
}
