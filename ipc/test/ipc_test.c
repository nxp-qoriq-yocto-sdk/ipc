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
#include <pthread.h>
#include "fsl_ipc_helper.h"
#include "fsl_psc913x_ipc.h"
#include "fsl_ipc_errorcodes.h"

#define ENTER()	printf(">> %s %d %s\n", __FILE__, __LINE__, __func__)
#define EXIT(A)	printf("<< (%d) %s %d %s\n", A, __FILE__, __LINE__, __func__)

volatile int ch3init=0;
volatile int ch4init=0;
volatile int ch7init=0;
void test_init();

/* Logging Function */
void dump_msg(char *msg, int len)
{
    int i=0;
    for(i=0;i<len;i++)
        printf("%x",msg[i]);

    printf("\n");
}

void main()
{
    test_init();
}

///////////////// Create Channels ////////////////////
int isbitset(uint32_t v, int bit)
{
	if((v>>(31-bit)) & 0x1)
		return 1;
}
void channel3_thread(void *ptr)
{
	uint32_t bmask;
	phys_addr_t p;
	uint32_t len;
	int num_sent=0;
	int num_recv=0;
	int ret;
	int ctr=0;
	int depth = 16;
	char bs[20] ={0};
	ENTER();
	ret = fsl_ipc_configure_channel(3, depth, IPC_PTR_CH, 0, 0, NULL);
	if(ret)
	{
		printf("\n ret %x \n", ret); 
		EXIT(ret);
		pthread_exit(0);
	}
	ch3init=1;
	while(1)
	{
		do{
			ret = fsl_ipc_recv_ptr(3, &p, &len);
			usleep(1000);
		}while(ret == ERR_CHANNEL_EMPTY);
		if(ret)
		{
			ret = printf("\n ERROR ret %x \n", ret); 
			while(1);
		}
		{
			
			printf("\n Recv %x %x \n", p, len);
			sprintf(bs,"MSG #%x", ctr++);
			do{
				ret = fsl_ipc_send_msg(2, bs, 15);
			}while(ret == ERR_CHANNEL_FULL);
			if(ret)
			{
				ret = printf("\n ret %x \n", ret); 
			}
			printf("$$$ sent=%d recv=%d\n", num_sent++, num_recv++);			
		}
		usleep(10000);
	}
	EXIT(0);
	pthread_exit(0);
}
void channel4_thread(void *ptr)
{
	uint32_t bmask;
	phys_addr_t p;
	uint32_t len;
	int ret;
	int depth = 16;
	char retbuf[1024];
	ENTER();
	ret = fsl_ipc_configure_channel(4, depth, IPC_PTR_CH, 0, 0, NULL);
	if(ret)
	{
		printf("\n ret %x \n", ret); 
		EXIT(ret);
		pthread_exit(0);
	}
	ch4init=1;
	while(1)
	{
		do{
			ret = fsl_ipc_recv_ptr(4, &p, &len);
			printf("_");
			usleep(1000);
		}while(ret == ERR_CHANNEL_EMPTY);
		if(ret)
		{
			ret = printf("\n ERROR ret %x \n", ret); 
			while(1);
		}
		
		{
			printf("\n Th4 Recv %x %x \n", p, len);
			void *vaddr = fsl_ipc_helper_p2v(p);
			printf("Vaddr = %x\n", vaddr);
			memcpy(retbuf, vaddr, len);
			do{
				ret = fsl_ipc_send_msg(5, retbuf, len);	
				printf("^");
				usleep(1000);
			}while(ret == ERR_CHANNEL_FULL);
			if (ret)
			{
				printf("Error code = %x\n", ret);
				while(1);
				pthread_exit(0);
			}
		}		
		usleep(100000);
	}
	EXIT(0);
	pthread_exit(0);
}
void channel67_thread(void *ptr)
{
	uint32_t bmask;
	phys_addr_t p;
	uint32_t len;
	int ret;
	int depth = 8;
	int ctr;
	range_t r;
	sg_entry_t *e;
	sg_list_t  lst;
	void *vaddr;
	int first=1;
	char fapi_msg[1020];
	ENTER();
	ret = fsl_ipc_configure_channel(7, depth, IPC_PTR_CH, 0, 0, NULL);
	if(ret)
	{
		printf("\n ret %d \n", ret); 
		EXIT(ret);
		pthread_exit(0);
	}
	ch7init=1;
	//configure tx request
	get_free_pool(&r);
	printf("range of free pool P=%x V=%x S=%x\n", r.phys_addr, r.vaddr, r.size); 
	ret = fsl_ipc_configure_txreq(6, r.phys_addr + 0x100000, 1024);
	if(ret) {
		printf("\n ***********************ret %d \n", ret); 
		EXIT(ret);
		pthread_exit(0);
	}
	ctr=1;	
	
	while(1)
	{
		lst.entry[0].src_addr = r.phys_addr;
		lst.entry[0].len = 1024;
		lst.entry[0].is_valid = 1;
		vaddr = fsl_ipc_helper_p2v(lst.entry[0].src_addr);
		memset(vaddr,0x11 + ctr, 1024);
	
		lst.entry[1].src_addr = r.phys_addr + 1024*16; 
		lst.entry[1].len = 1024;
		lst.entry[1].is_valid = 1;
		vaddr = fsl_ipc_helper_p2v(lst.entry[1].src_addr);
		memset(vaddr,0x22 + ctr, 1024);

		lst.entry[2].is_valid = 0;
		vaddr = fsl_ipc_helper_p2v(lst.entry[1].src_addr);
		memset(fapi_msg, ctr, 1020);
		do{
			if(!first)
				while(fsl_ipc_get_last_tx_req_status()
								!=TXREQ_DONE)
				{		
					printf("*");
				}
			ret = fsl_ipc_send_tx_req(6, &lst, &fapi_msg, 1020);
			first =0;
		}while(ret == ERR_CHANNEL_FULL);
		if(ret)
		{
			ret = printf("\n ERROR ret %x \n", ret); 
			EXIT(ret);
			while(1);
		}

		do{
			ret = fsl_ipc_recv_ptr(7, &p, &len);
			usleep(1000);
		}while(ret == ERR_CHANNEL_EMPTY);
		if(ret)
		{
			ret = printf("\n ERROR ret %x \n", ret); 
			EXIT(ret);
			while(1);
		}
	
		printf("\n Recvd %x %x \n", p, len);
		ctr++;
	}
	EXIT(0);
	pthread_exit(0);
}

void test_init()
{
	range_t r;
	uint32_t bmask;
	int ret=0;
	int ret1,ret2,ret3;
	char *buf = "Hello DSP.";
	pthread_t thread1, thread2, thread3;
	int depth=8; //ping pong
	ENTER();	
	printf("\n=========$IPC TEST$====%s %s====\n", __DATE__, __TIME__);
	fsl_ipc_helper_init();
	ret  = fsl_ipc_init(fsl_ipc_helper_p2v);
	if(ret)
		printf("Issue with fsl_ipc_init %d\n", ret);

	do{	
		fsl_ipc_chk_recv_status(&bmask);
		printf("\n main loop #ret %x \n", bmask); 
		usleep(10000);
	}while(!(isbitset(bmask, 0)));
	
	fsl_ipc_open_prod_ch(2);
	fsl_ipc_open_prod_ch(5);
	fsl_ipc_open_prod_ch(6);

	printf("Trying to start a thread\n");
	ret1  = pthread_create(&thread1, NULL, (void*)&channel3_thread, NULL);
	ret2  = pthread_create(&thread2, NULL, (void*)&channel4_thread, NULL);
	ret3  = pthread_create(&thread3, NULL, (void*)&channel67_thread, NULL);
	
	printf("ptherad_create%d %d %d\n", ret1, ret2, ret3);
	
	while(!(ch3init && ch4init && ch7init))
	{
		printf(".");
		usleep(1000);
	}
	printf("Trying to send message on ch#2\n");
	ret = fsl_ipc_send_msg(2, buf, 10); 
	if(ret)
		printf("Issue with fsl_ipc_send_msg %d\n", ret);

	pthread_join(thread1, NULL);
	pthread_join(thread2, NULL);
	pthread_join(thread3, NULL);
	EXIT(0);
}


