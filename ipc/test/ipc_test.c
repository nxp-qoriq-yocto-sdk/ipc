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
#include <ctype.h>
#include <pthread.h>
#include "fsl_usmmgr.h"
#include "fsl_bsc913x_ipc.h"
#include "fsl_ipc_errorcodes.h"
#define UIO_INTERFACE   "/dev/uio0"
#define CMD_LINE_OPT "r:i:"

#define ENTER()	printf(">> %s %d %s\n", __FILE__, __LINE__, __func__)
#define EXIT(A)	printf("<< (%d) %s %d %s\n", A, __FILE__, __LINE__, __func__)
#define mute_print(...)

fsl_usmmgr_t usmmgr;
fsl_ipc_t ipc;
int ch3init;
int ch4init;
void test_init(int rat_id);
int rat_id, loop, mute_flag, loop2;

/* Logging Function */
void dump_msg(char *msg, int len)
{
	int i = 0;
	for (i = 0; i < len; i++)
		printf("%x", msg[i]);

	printf("\n");
}
void usage(char **argv)
{
	printf("Usage: %s -r <rat_id> -i <nr_msg>\n",
		argv[0]);
	printf("whereas,\n <rat_id> : 0 for SingleRAT\n"
		"          : 1 for MultiRAT\n"
		" <nr_msg> : Number of Messages to be exchanged on an"
		" IPC channel \n");
	exit(EXIT_FAILURE);
}

int main(int argc, char **argv)
{
	int opt;
	while ((opt = getopt(argc, argv, CMD_LINE_OPT)) != -1) {
		switch (opt) {
		case 'r':
			if (isdigit(optarg[0])) {
				rat_id = atoi(optarg);
				break;
			} else
				usage(argv);
		case 'i':
			loop = atoi(optarg);
			loop2 = loop;
			mute_flag = 1;
			break;
		default:
			usage(argv);
		}

	}
	if (optind == 1)
		usage(argv);

	test_init(rat_id);
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
	unsigned long p;
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
	unsigned long p;
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

void channel3_thread_m(void *ptr)
{
	unsigned long p;
	uint32_t len;
	int ret, ch2send_ctr = 0;
	int tr = 0, ts = 0;
	int ctr = 0, ch3recv_ctr = 0;
	int depth = 16;
	char bs[20] = { 0 };

	ret = fsl_ipc_configure_channel(3, depth, IPC_PTR_CH, 0, 0, NULL, ipc);
	if (ret) {
		printf("\n ret %x \n", ret);
		EXIT(ret);
		pthread_exit(0);
	}
	ch3init = 1;
	while (loop-- > 0) {
		do {
			ret = fsl_ipc_recv_ptr(3, &p, &len, ipc);
			usleep(1000);
			tr++;
			if (ret == ERR_SUCCESS)
				tr = 0;
			else if (tr == 500) {
				/* wait for 0.5 sec*/
				printf("Timed out. Message not received"
						" on ch#3\n");
				pthread_exit(0);
			}
		} while (ret == -ERR_CHANNEL_EMPTY);
		if (ret) {
			ENTER();
			printf("\n ERROR ret %x \n", ret);
			printf("Exiting thread for ch3\n");
			goto end;
		}
		ch3recv_ctr++;

		sprintf(bs, "%x ", ctr++);
		do {
			ret = fsl_ipc_send_msg(2, bs, 9, ipc);
			usleep(1000);
			ts++;
			if (ret == ERR_SUCCESS)
				ts = 0;
			else if (ts == 500) {
				/* wait for 0.5 sec*/
				printf("Timed out. Not able to send"
						" message on ch#2\n");
				pthread_exit(0);
			}
		} while (ret == -ERR_CHANNEL_FULL);
		if (ret) {
			printf("\n ret %x \n", ret);
			printf("Exiting thread for ch2\n");
			goto end;
		}
		ch2send_ctr++;
	}

	if (ch3recv_ctr == ch2send_ctr && ch3recv_ctr != 0) {
		printf("(%d) Msg Sent on ch#2\n", ch2send_ctr);
		printf("(%d) Msg Recieved on ch#3\n", ch3recv_ctr);
		printf("Success on ch#2-ch#3 pair\n");
		pthread_exit(0);
	} else
		printf("failure on ch#2-ch#3 pair\n");
end:
	pthread_exit(0);
}

void channel4_thread_m(void *ptr)
{
	unsigned long p;
	uint32_t len;
	int ret, ch4recv_ctr = 0;
	int tr = 0, ts = 0;
	int depth = 16, ch5send_ctr = 0;
	char retbuf[1024];
	void *vaddr;

	ret = fsl_ipc_configure_channel(4, depth, IPC_PTR_CH, 0, 0, NULL, ipc);
	if (ret) {
		printf("\n ret %x \n", ret);
		EXIT(ret);
		pthread_exit(0);
	}
	ch4init = 1;
	while (loop2-- > 0) {
		do {
			ret = fsl_ipc_recv_ptr(4, &p, &len, ipc);
			usleep(1000);
			tr++;
			if (ret == ERR_SUCCESS)
				tr = 0;
			else if (tr == 500) {
				/* wait for 0.5 sec*/
				printf("Timed out. Message not received"
						" on ch#4\n");
				pthread_exit(0);
			}
		} while (ret == -ERR_CHANNEL_EMPTY);
		if (ret) {
			printf("\n ERROR ret %x \n", ret);
			printf("Exiting Thread for ch4\n");
			goto end;
		}
		ch4recv_ctr++;

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
			ts++;
			if (ret == ERR_SUCCESS)
				ts = 0;
			else if (ts == 500) {
				/* wait for 0.5 sec*/
				printf("Timed out. Not able to send"
						" message on ch#5\n");
				pthread_exit(0);
			}
		} while (ret == -ERR_CHANNEL_FULL);
		if (ret) {
			printf("Error code = %x\n", ret);
			printf("Exiting Thread for ch5\n");
			goto end;
		}
		ch5send_ctr++;
	}

	if (ch4recv_ctr == ch5send_ctr && ch4recv_ctr != 0) {
		printf("(%d) Msg Sent on ch#5\n", ch5send_ctr);
		printf("(%d) Msg Recieved on ch#4\n", ch4recv_ctr);
		printf("Success on ch#4-ch#5 pair\n");
		pthread_exit(0);
	} else
		printf("failure on ch#4-ch#5 pair\n");
end:
	pthread_exit(0);
}

void *test_p2v(unsigned long phys_addr)
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
	mem_range_t sh_ctrl;
	mem_range_t dsp_ccsr;
	mem_range_t pa_ccsr;

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
			test_p2v, sh_ctrl, dsp_ccsr, pa_ccsr, UIO_INTERFACE);
	} else {
		ipc = fsl_ipc_init_rat(
			rat_id,
			test_p2v, sh_ctrl, dsp_ccsr, pa_ccsr, UIO_INTERFACE);
	}

	if (!ipc) {
		printf("Issue with fsl_ipc_init %d\n", ret);
		return;
	}
	do {
		fsl_ipc_chk_recv_status(&bmask, ipc);
		usleep(10000);
		if (!bmask)
			printf("\n main loop #ret %llx \n",
				(long long unsigned int)bmask);
	} while (!(isbitset(bmask, 0)));

	fsl_ipc_open_prod_ch(2, ipc);
	fsl_ipc_open_prod_ch(5, ipc);

	if (mute_flag == 1) {
		ret1 = pthread_create(&thread1, NULL,
				(void *)&channel3_thread_m, NULL);
		if (ret1) {
			printf("pthread_create returns with error: %d", ret1);
			printf("from channel3_thread_m\n");
			return;
		}

		ret2 = pthread_create(&thread2, NULL,
				(void *)&channel4_thread_m, NULL);
		if (ret2) {
			printf("pthread_create returns with error: %d", ret2);
			printf("from channel4_thread_m\n");
			return;
		}

		mute_print("ptherad_create %d %d\n", ret1, ret2);
	} else {
		printf("Trying to start a thread\n");
		ret1 = pthread_create(&thread1, NULL,
				(void *)&channel3_thread, NULL);
		if (ret1) {
			printf("pthread_create returns with error: %d", ret1);
			printf("from channel3_thread\n");
			return;
		}

		ret2 = pthread_create(&thread2, NULL,
				(void *)&channel4_thread, NULL);
		if (ret2) {
			printf("pthread_create returns with error: %d", ret2);
			printf("from channel4_thread\n");
			return;
		}

		printf("ptherad_create %d %d\n", ret1, ret2);
	}

	while (!(ch3init && ch4init)) {
		mute_print(".");
		usleep(1000);
	}

	if (mute_flag == 0)
		printf("Trying to send message on ch#2\n");
	ret = fsl_ipc_send_msg(2, buf, 10, ipc);
	if (ret)
		printf("Issue with fsl_ipc_send_msg %d\n", ret);

	if (mute_flag == 0)
		printf("\n[IPC_PA] S:C2:M:L:%x\n", 10);
	pthread_join(thread1, NULL);
	pthread_join(thread2, NULL);
	exit(0);
}
