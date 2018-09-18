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
#include <ctype.h>
#include "fsl_usmmgr.h"
#include "fsl_bsc913x_ipc.h"
#include "fsl_ipc_errorcodes.h"
#define CMD_LINE_OPT "r:i:"

#define ENTER()	printf(">> %s %d %s\n", __FILE__, __LINE__, __func__)
#define EXIT(A)	printf("<< (%d) %s %d %s\n", A, __FILE__, __LINE__, __func__)
#define mute_print(...)

#define MAX_NUM_RATS_USED	6
#define UIO_NAME_LENGTH		12

fsl_usmmgr_t usmmgr;
fsl_ipc_t ipc[MAX_NUM_RATS_USED];
int ch7init[MAX_NUM_RATS_USED];
void test_init(void *id);
int loop, mute_flag;

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
	printf("Usage: %s -r <rat_id>  [-r <rat_id>]  -i <nr_msg>\n",
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
	int rat_id[MAX_NUM_RATS_USED];
	pthread_t rat_thread[MAX_NUM_RATS_USED];
	int num_rats = 0;
	int i;

	while ((opt = getopt(argc, argv, CMD_LINE_OPT)) != -1) {
		switch (opt) {
		case 'r':
			if (isdigit(optarg[0])) {
				rat_id[num_rats++] = atoi(optarg);
				printf("rat_id[num_rats] =%d\n", rat_id[num_rats-1]);
				break;
			} else
				usage(argv);
		case 'i':
			loop = atoi(optarg);
			mute_flag = 0;
			break;
		default:
			usage(argv);
		}

	}
	if (optind == 1)
		usage(argv);

	usmmgr = fsl_usmmgr_init();
	if (!usmmgr) {
		printf("Error in Initializing User Space Memory Manager\n");
		exit(EXIT_FAILURE);
	}

	for (i = 0; i < num_rats; i++) {
		printf("rat_id[i] %d\n", rat_id[i]);
		int error_code;

		error_code = pthread_create(&rat_thread[i], NULL, (void *)&test_init, &rat_id[i]);
		if (error_code) {
			printf("Error - RAT thread create return code: %d \n", error_code);
			exit(EXIT_FAILURE);
		}
		sleep(2);
	}
	for (i = 0; i < num_rats; i++) {
		pthread_join(rat_thread[i], NULL);

	}
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
	unsigned long p;
	uint32_t len;
	int ret;
	int depth = 16;
	int ctr;
	mem_range_t r;
	sg_list_t lst;
	void *vaddr;
	int first = 1;
	char fapi_msg[1020];
	int rat_id;
	rat_id = *((int *)ptr);

	ENTER();
	ret = fsl_ipc_configure_channel(7, depth, IPC_PTR_CH, 0, 0, NULL, ipc[rat_id]);
	if (ret) {
		printf("\n ret %d \n", ret);
		EXIT(ret);
		pthread_exit(0);
	}

	ch7init[rat_id] = 1;
	r.size = 0x200000;
	ret = fsl_usmmgr_alloc(&r, usmmgr);
	if (ret) {
		printf("\n Unable to allocate memory from shm_alloc \n");
		EXIT(-1);
		pthread_exit(0);
	}
	printf("range of free pool P=%llx V=%p S=%x\n", r.phys_addr, r.vaddr,
	       r.size);
	ret = fsl_ipc_configure_txreq(6, r.phys_addr + 0x100000, 1024*2, ipc[rat_id]);
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

		memset(vaddr, 0x11 + rat_id + ctr, 1024);

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

		memset(vaddr, 0x22 + rat_id + ctr, 1024);

		lst.entry[2].is_valid = 0;
		/* vaddr = fsl_usmmgr_p2v(lst.entry[1].src_addr, usmmgr); */
		memset(fapi_msg, rat_id + ctr, 1020);
		do {
			if (!first)
				while (fsl_ipc_get_last_tx_req_status(ipc[rat_id])
				       != TXREQ_DONE) {
						usleep(10000);
						printf(",");
				}
			ret =
			    fsl_ipc_send_tx_req(6, &lst, &fapi_msg, 1020, ipc[rat_id]);
			    usleep(10000);
			    first = 0;
		} while (ret == -ERR_CHANNEL_FULL);
		if (ret) {
			printf("\n ERROR ret %x \n", ret);
			goto end;
		}
		printf("\n[IPC_PA %d]S:C6:TXREQ ctr=%d\n", rat_id, ctr);

		do {
			ret = fsl_ipc_recv_ptr(7, &p, &len, ipc[rat_id]);
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

void channel67_thread_m(void *ptr)
{
	unsigned long p;
	uint32_t len;
	int ret, ch6send_ctr = 0;
	int depth = 16;
	int ctr, ch7recv_ctr = 0;
	int tr = 0, ts = 0, ts_dma = 0;
	mem_range_t r;
	sg_list_t lst;
	void *vaddr;
	int first = 1;
	char fapi_msg[1020];
	int rat_id = *((int *)ptr);

	ret = fsl_ipc_configure_channel(7, depth, IPC_PTR_CH, 0, 0,
						NULL, ipc[rat_id]);
	if (ret) {
		printf("\n ret %d \n", ret);
		EXIT(ret);
		pthread_exit(0);
	}

	ch7init[rat_id] = 1;
	r.size = 0x200000;
	ret = fsl_usmmgr_alloc(&r, usmmgr);
	if (ret) {
		printf("\n Unable to allocate memory from shm_alloc \n");
		EXIT(-1);
		pthread_exit(0);
	}
	mute_print("range of free pool P=%llx V=%p S=%x\n", r.phys_addr,
			r.vaddr, r.size);
	ret = fsl_ipc_configure_txreq(6, r.phys_addr + 0x100000,
						1024*2, ipc[rat_id]);
	if (ret) {
		printf("\n Error in fsl_ipc_configure_txreq ret %d \n", ret);
		goto end;
	}
	ctr = 1;
	while (loop-- > 0) {
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

		memset(vaddr, 0x11 + rat_id + ctr, 1024);

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
		memset(vaddr, 0x22 + rat_id + ctr, 1024);
		lst.entry[2].is_valid = 0;
		/* vaddr = fsl_usmmgr_p2v(lst.entry[1].src_addr, usmmgr); */
		memset(fapi_msg, rat_id + ctr, 1020);
		do {
			if (!first) {
				ret = fsl_ipc_get_last_tx_req_status(
								ipc[rat_id]);
				while (ret != TXREQ_DONE) {
						usleep(10000);
						ts_dma++;
						if (ret == TXREQ_DONE ||
						    ret == TXREQ_IN_PROCESS)
							ts_dma = 0;
						else if (ts_dma == 100) {
							/* wait for 1 sec*/
							printf("Timed out DMA"
							". errno on ch#6"
							"is (%i)\n", ret);
							pthread_exit(0);
						}
				}
			}
			ret =
			    fsl_ipc_send_tx_req(6, &lst, &fapi_msg, 1020,
							ipc[rat_id]);
			    usleep(10000);
			    first = 0;
			    ts++;
			    if (ret == ERR_SUCCESS)
				ts = 0;
			    else if (ts == 100) {
				/* wait for 1 sec*/
				printf("Timed out. Not able to send"
						" message on ch#6\n");
				pthread_exit(0);
			    }
		} while (ret == -ERR_CHANNEL_FULL);
		if (ret) {
			printf("\n ERROR ret %x \n", ret);
			goto end;
		}
		ch6send_ctr++;

		do {
			ret = fsl_ipc_recv_ptr(7, &p, &len, ipc[rat_id]);
			usleep(1000);
			tr++;
			if (ret == ERR_SUCCESS)
				tr = 0;
			else if (tr == 500) {
				/* wait for 0.5 sec*/
				printf("Timed out. Message not received"
						" on ch#7\n");
				pthread_exit(0);
			}
		} while (ret == -ERR_CHANNEL_EMPTY);
		if (ret) {
			printf("\n ERROR ret %x \n", ret);
			goto end;
		}
		ch7recv_ctr++;
		ctr++;
	}
	if (ch7recv_ctr == ch6send_ctr && ch7recv_ctr != 0) {
		printf("(%d) Msg Sent on ch#6\n", ch6send_ctr);
		printf("(%d) Msg Recieved on ch#7\n", ch7recv_ctr);
		printf("Success on ch#6-ch#7 pair\n");
		pthread_exit(0);
	} else
		printf("failure on ch#6-ch#7 pair\n");

end:
	pthread_exit(0);
}

void *test_p2v(unsigned long phys_addr)
{
	return fsl_usmmgr_p2v(phys_addr, usmmgr);
}

void test_init(void *id)
{
	int err = 0;
	int ret = 0;
	int ret3;
	pthread_t thread3;
	mem_range_t sh_ctrl;
	mem_range_t dsp_ccsr;
	mem_range_t pa_ccsr;
	int rat_id = *((int *)id);
	char uio_interface[UIO_NAME_LENGTH];

	printf("\n=========$IPC TEST Channel 67$====%s %s====\n", __DATE__,
	       __TIME__);

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

	sprintf(uio_interface, "/dev/uio%d", rat_id);
	ipc[rat_id] = fsl_ipc_init_rat(
		rat_id,
		test_p2v, sh_ctrl, dsp_ccsr, pa_ccsr, uio_interface);

	if (NULL == ipc[rat_id]) {
		printf("Issue with fsl_ipc_init %d\n", ret);
		return;
	}
	err = fsl_ipc_open_prod_ch(6, ipc[rat_id]);
	if (err) {
		printf("Issue opening producer channel 6\n");
		return;
	}

	if (mute_flag == 1) {
		mute_print("Trying to start a thread\n");
		ret3 = pthread_create(&thread3, NULL,
				(void *)&channel67_thread_m, &rat_id);

		if (ret3) {
			printf("pthread_create returns with error: %d", ret3);
			return;
		}

		mute_print("ptherad_create %d\n", ret3);
	} else {
		printf("Trying to start a thread\n");
		ret3 = pthread_create(&thread3, NULL,
				(void *)&channel67_thread, &rat_id);

		if (ret3) {
			printf("pthread_create returns with error: %d", ret3);
			return;
		}
		printf("ptherad_create %d\n", ret3);


	}

	while (!ch7init[rat_id]) {
		mute_print(".");
		usleep(1000);
	}
	pthread_join(thread3, NULL);
	exit(0);
}
