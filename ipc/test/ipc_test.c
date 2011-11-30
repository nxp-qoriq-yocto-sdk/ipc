 /*
  * Copyright (c) 2011
  *  Freescale Semiconductor Inc.  All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without
  * modification, are permitted provided that the following conditions
  * are met:
  * 1. Redistributions of source code must retain the above copyright
  *    notice, this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright
  *    notice, this list of conditions and the following disclaimer in the
  *    documentation and/or other materials provided with the distribution.
  * 3. Neither the name of Freescale Semiconductor Inc nor the names of its
  *    contributors may be used to endorse or promote products derived from
  *    this software without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND
  * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  * ARE DISCLAIMED.  IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
  * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
  * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
  * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
  * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
  * SUCH DAMAGE.
  *
  *      Author: Manish Jaggi <manish.jaggi@freescale.com>
  */
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <pthread.h>
#include "fsl_usmmgr.h"
#include "fsl_psc913x_ipc.h"
#include "fsl_ipc_errorcodes.h"

#define ENTER()	printf(">> %s %d %s\n", __FILE__, __LINE__, __func__)
#define EXIT(A)	printf("<< (%d) %s %d %s\n", A, __FILE__, __LINE__, __func__)

fsl_usmmgr_t usmmgr;
fsl_ipc_t ipc;
int ch3init;
int ch4init;
void test_init();

/* Logging Function */
void dump_msg(char *msg, int len)
{
	int i = 0;
	for (i = 0; i < len; i++)
		printf("%x", msg[i]);

	printf("\n");
}

void main()
{
	test_init();
}

///////////////// Create Channels ////////////////////
int isbitset(uint64_t v, int bit)
{
	if ((v >> (63 - bit)) & 0x1)
		return 1;
}
void channel3_thread(void *ptr)
{
	uint64_t bmask;
	phys_addr_t p;
	uint32_t len;
	int num_sent = 0;
	int num_recv = 0;
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
		printf("\nR:C3:P:[%x]L:%x\n", p, len);

		sprintf(bs, "MSG #%x", ctr++);
		do {
			ret = fsl_ipc_send_msg(2, bs, 15, ipc);
		} while (ret == -ERR_CHANNEL_FULL);
		if (ret) {
			printf("\n ret %x \n", ret);
			goto end;
		}
		printf("\nS:C2:M:L:%x\n", 15);
	}
end:
	printf("Exiting thread for ch2/3\n");
	EXIT(ret);
	pthread_exit(0);
}
void channel4_thread(void *ptr)
{
	uint64_t bmask;
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
		printf("\nR:C4:P:[%x]L:%x\n", p, len);

		vaddr = fsl_usmmgr_p2v(p, usmmgr);
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

void test_init()
{
	range_t r;
	uint64_t bmask;
	int ret = 0;
	int ret1, ret2, ret3;
	char *buf = "Hello DSP.";
	pthread_t thread1, thread2, thread3;
	int depth = 8;
	range_t sh_ctrl;
	range_t dsp_ccsr;
	range_t pa_ccsr;

	ENTER();
	printf("\n=========$IPC TEST$====%s %s====\n", __DATE__, __TIME__);

	usmmgr = fsl_usmmgr_init();

	/* get values from mmgr */
	ret = get_pa_ccsr_area(&pa_ccsr, usmmgr);
	ret = get_dsp_ccsr_area(&dsp_ccsr, usmmgr);
	ret = get_shared_ctrl_area(&sh_ctrl, usmmgr);

	/* TBD : check return values */

	ipc = fsl_ipc_init(test_p2v, sh_ctrl, dsp_ccsr, pa_ccsr);

	if (!ipc) {
		printf("Issue with fsl_ipc_init %d\n", ret);
		return;
	}
	do {
		fsl_ipc_chk_recv_status(&bmask, ipc);
		printf("\n main loop #ret %x \n", bmask);
		usleep(10000);
	} while (!(isbitset(bmask, 0)));

	fsl_ipc_open_prod_ch(2, ipc);
	fsl_ipc_open_prod_ch(5, ipc);

	printf("Trying to start a thread\n");
	ret1 = pthread_create(&thread1, NULL, (void *)&channel3_thread, NULL);
	ret2 = pthread_create(&thread2, NULL, (void *)&channel4_thread, NULL);

	printf("ptherad_create%d %d %d\n", ret1, ret2, ret3);

	while (!(ch3init && ch4init)) {
		printf(".");
		usleep(1000);
	}
	printf("Trying to send message on ch#2\n");
	ret = fsl_ipc_send_msg(2, buf, 10, ipc);
	if (ret)
		printf("Issue with fsl_ipc_send_msg %d\n", ret);

	printf("\nS:C2:M:L:%x\n", 10);
	pthread_join(thread1, NULL);
	pthread_join(thread2, NULL);
	EXIT(0);
}