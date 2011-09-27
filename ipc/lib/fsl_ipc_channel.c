/*
 *  @ fsl_ipc_channel.c
 *
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
#include <fcntl.h>
#include <sys/mman.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <stdint.h>
#include "logdefs.h"
#include "fsl_het_mgr.h"
#include "fsl_user_dma.h"
#include "fsl_ipc_um.h"
#include "fsl_ipc_helper.h"
#include "fsl_ipc_kmod.h"
#include "fsl_psc913x_ipc.h"
#include "fsl_ipc_errorcodes.h"
#include "psc913x_heterogeneous.h"
#include "psc913x_heterogeneous_ipc.h"

#define LOCAL_PRODUCER_NUM pa_reserved[0]
#define LOCAL_CONSUMER_NUM pa_reserved[1]

/* Defines */
#define MAX_MSG_SIZE 1020
#define PAGE_SIZE 4096
#define GCR_OFFSET 0x17000
#define SH_CTRL_VADDR(A) \
		(void *)((phys_addr_t)(A) \
		- (ipc_priv.sh_ctrl_area.phys_addr) \
		+  ipc_priv.sh_ctrl_area.vaddr)

phys_addr_t get_channel_paddr(uint32_t channel_id);
void *get_channel_vaddr(uint32_t channel_id);
void signal_handler(int signo, siginfo_t *siginfo, void *data);
uint32_t ipc_get_free_rt_signal(void);
int init_het_ipc();
void get_channels_info();
void generate_indication(volatile os_het_ipc_channel_t *ipc_ch);

/*
 * Global Data structure
 *
 */
ipc_userspace_t ipc_priv;
/*
TBD: if shared library this function
can be extended to return a cookie passed as IN param
which will contain the pointer to ipc_priv
*/
int fsl_ipc_init(ipc_p2v_t p2vcb)
{
	int ret = ERR_SUCCESS;
	ipc_bootargs_info_t ba;
	struct sigaction sig_action;

	ENTER();

	memset(&ipc_priv, 0, sizeof(ipc_userspace_t));
	ipc_priv.p2vcb = p2vcb;

	if (!p2vcb) {
		ret = -ERR_P2V_NULL;
		goto end;
	}

	ret = get_shared_ctrl_area(&ipc_priv.sh_ctrl_area);
	if (ret)
		goto end;

	ret = get_dsp_ccsr_area(&ipc_priv.dsp_ccsr);
	if (ret)
		goto end;

	ret = get_pa_ccsr_area(&ipc_priv.pa_ccsr);
	if (ret)
		goto end;

	ret = init_het_ipc();
	if (ret)
		goto end;
	get_channels_info();

end:
	EXIT(ret);
	return ret;
}

void fsl_ipc_exit(void)
{
	int i;

	close(ipc_priv.dev_het_ipc);
	/* free memory */
	for (i = 0; i < ipc_priv.num_channels; i++)
		free(ipc_priv.channels[i]);
}

int fsl_ipc_configure_channel(uint32_t channel_id, uint32_t depth,
			ipc_ch_type_t channel_type,
			phys_addr_t msg_ring_paddr, uint32_t msg_size,
			ipc_cbfunc_t cbfunc)
{
	int 			ret = ERR_SUCCESS;
	uint32_t		signal = 0;
	volatile os_het_ipc_channel_t 	*ch;
	ipc_channel_us_t	*uch;
	ipc_rc_t		rc;
	struct sigaction sig_action;
	ENTER();
	/*check if the channel id is valid */

	/*
	* Get the ptr to channel structure
	* in shared control area.
	*/
	ch = (volatile os_het_ipc_channel_t *)get_channel_vaddr(channel_id);

	printf("/* set the depth of the channel */\n");
	if (ch->bd_ring_size <= ipc_priv.max_depth)
		ch->bd_ring_size = depth;
	else {
		ret = -ERR_INVALID_DEPTH;
		EXIT(ret);
		goto end;
	}

	ch->ch_type = channel_type;
	if (channel_type == OS_HET_IPC_MESSAGE_CH)
		ipc_channel_attach_msg_ring(ch, msg_ring_paddr, msg_size);

	if (cbfunc) {
		/* find a free rt signal */
		signal = ipc_get_free_rt_signal();
		if (!signal) {
			printf("No Free signal found!!!\n");
			ret = -ERR_NO_SIGNAL_FOUND;
			EXIT(ret);
			goto end;
		}
		/*
		 * call the kernel mode handler to register the irq
		 * the ipc kernel driver finds a way for DSP to interrupt
		 * the PA, either using MSG or MSI interrupt.
		 */
		rc.channel_id = channel_id;
		rc.signal = signal;

		sig_action.sa_sigaction = signal_handler;
		sig_action.sa_flags = SA_SIGINFO;
		ret = sigaction(signal, &sig_action, NULL);

		ret = ioctl(ipc_priv.dev_het_ipc, IOCTL_IPC_REGISTER_SIGNAL,
				&rc);
		if (!ret) {
			ret = -ERR_IOCTL_FAIL;
			EXIT(ret);
			goto end;
		}
	} else
		ch->ipc_ind = OS_HET_NO_INT;
	printf("/* allocate ipc_channel_us_t */\n");
	uch = calloc(1, sizeof(ipc_channel_us_t));
	if (!uch) {
		ret = -ERR_CALLOC;
		EXIT(ret);
		goto end;
	}

	printf("/* attach the channel to the list */\n");
	ipc_priv.channels[ipc_priv.num_channels++] = uch;

	printf("/* fill the channel structure */");
	uch->cbfunc = cbfunc;
	uch->signal = signal;
	uch->channel_id = channel_id;
	ch->consumer_initialized = OS_HET_INITIALIZED;

end:
	EXIT(ret);
	return ret;
}

int fsl_ipc_configure_txreq(uint32_t channel_id, phys_addr_t phys_addr,
			uint32_t max_txreq_lbuff_size)
{
	int ret;
	phys_addr_t phys_addr_s;
	range_t dma_list_mem;
	volatile os_het_ipc_channel_t *ipc_ch;

	ENTER();
	printf("Params %x %x %x \n", channel_id, phys_addr,
			max_txreq_lbuff_size);
	ret = ERR_SUCCESS;
	ipc_ch =  (volatile os_het_ipc_channel_t *)
					get_channel_vaddr(channel_id);

	ipc_priv.txreq_tb_lbuff_paddr = phys_addr;
	ipc_priv.max_txreq_lbuff_size = max_txreq_lbuff_size;

/*	dump_ipc_channel(ipc_ch); */
	/* Get spare area vaddr */
/*FIXME*/
	phys_addr_s = ipc_priv.txreq_tb_lbuff_paddr +
		(ipc_ch->bd_ring_size + 2) * ipc_priv.max_txreq_lbuff_size;

	printf("Phys_addr_s =%x\n", phys_addr_s);

	phys_addr_s += 2*sizeof(phys_addr_t);

	phys_addr_s += (32  - phys_addr_s % 32);

	/* TBD dma_list_mem.phys_addr = align(phys_addr_s, 32); */
	dma_list_mem.phys_addr = phys_addr_s;

	dma_list_mem.vaddr = (*ipc_priv.p2vcb)(phys_addr_s);

	ret = fsl_uspace_dma_init(dma_list_mem);

	EXIT(ret);
	return ret;
}

int fsl_ipc_send_tx_req(uint32_t channel_id, sg_list_t *sgl,
		void *tx_req_vaddr, uint32_t tx_req_len)
{
	int ret;
	uint32_t	ctr;
	uint32_t	incr1, incr2;
	void		*vaddr;
	sg_entry_t	*sge;
	phys_addr_t	phys_addr;
	phys_addr_t	phys_addr2;
	volatile os_het_ipc_bd_t         *bd_base;
	volatile os_het_ipc_bd_t	*bd;
	volatile os_het_ipc_channel_t *ipc_ch;

	ENTER();
	ret = ERR_SUCCESS;
	ipc_ch =  get_channel_vaddr(channel_id);;

	/* check if the channel is full */
	if (OS_HET_CH_FULL(ipc_ch)) {
		EXIT(-ERR_CHANNEL_FULL);
		return -ERR_CHANNEL_FULL;
	}

	fsl_uspace_dma_list_clear();

	/* copy txreq */
	bd_base = SH_CTRL_VADDR(ipc_ch->bd_base);
/*OLD	bd = &bd_base[ipc_ch->tracker.producer_num]; */
	bd = &bd_base[ipc_ch->LOCAL_PRODUCER_NUM];

	phys_addr = bd->msg_ptr;
	bd->msg_len = tx_req_len;
	vaddr = (*ipc_priv.p2vcb)(phys_addr);
	if (!vaddr) {
		EXIT(-1);
		return -1;
	}
	printf("copying %x to %x length %x\n", vaddr, tx_req_vaddr, tx_req_len);
	memcpy(vaddr, tx_req_vaddr, tx_req_len);

	/*write the lbuff address at the end of the message */
	printf("copying %x to %x length %x\n", ((uint32_t)vaddr +
		MAX_TX_REQ_MSG_SIZE), ipc_priv.txreq_tb_lbuff_paddr,
			sizeof(phys_addr_t));
	memcpy((void *)((uint32_t)vaddr + MAX_TX_REQ_MSG_SIZE),
		&ipc_priv.txreq_tb_lbuff_paddr, sizeof(phys_addr_t));

/*OLD	phys_addr = ipc_priv.txreq_tb_lbuff_paddr + ipc_ch->tracker.producer_num*ipc_priv.max_txreq_lbuff_size;*/
	phys_addr = ipc_priv.txreq_tb_lbuff_paddr +
		ipc_ch->LOCAL_PRODUCER_NUM * ipc_priv.max_txreq_lbuff_size;

	ctr = 0;
	while (sgl->entry[ctr].is_valid) {
		printf("%x %x %x\n", sgl->entry[ctr].is_valid,
			sgl->entry[ctr].src_addr, sgl->entry[ctr].len);
		if (sgl->entry[ctr].is_tb_start)	{
			/*check for alignment*/
		}
		fsl_uspace_dma_add_entry(sgl->entry[ctr].src_addr, phys_addr,
			sgl->entry[ctr].len);
		phys_addr += sgl->entry[ctr].len;
		ctr++;
		/* Alignment */
	}
	/* Get spare area vaddr */
	phys_addr = ipc_priv.txreq_tb_lbuff_paddr + (ipc_ch->bd_ring_size + 2) *
			ipc_priv.max_txreq_lbuff_size;

	vaddr = (*ipc_priv.p2vcb)(phys_addr);
	incr1 = ipc_ch->tracker.producer_num + 1;
	incr2 = (ipc_ch->LOCAL_PRODUCER_NUM + 1) % ipc_ch->bd_ring_size;

	printf("## Writing P=%x V=%x #=%x\n", phys_addr, vaddr, incr1);
	 /* Add producer increment */
	memcpy(vaddr, &incr1, sizeof(uint32_t));
	/* Get physical address of producer_num */
	phys_addr2 = get_channel_paddr(channel_id);
	printf("TXREQ 0: %x %x\n", phys_addr2, channel_id);
	phys_addr2 += (uint32_t)&ipc_ch->tracker.producer_num -
			(uint32_t)ipc_ch;
	printf("TXREQ: PIaddr=%x val=%x\n", phys_addr2, phys_addr);
	fsl_uspace_dma_add_entry(phys_addr, phys_addr2, sizeof(phys_addr_t));

	/* Get physical address of LOCAL_PRODUCER_NUM */
	memcpy((void *)((uint32_t)vaddr + 4), &incr2, sizeof(uint32_t));
	printf("## Writing V=%x P=%x #=%x\n", phys_addr + 4, vaddr + 4, incr2);
	phys_addr2 = get_channel_paddr(channel_id);
	printf("TXREQ 0: %x %x\n", phys_addr2, channel_id);
	phys_addr2 += (uint32_t)&ipc_ch->LOCAL_PRODUCER_NUM - (uint32_t)ipc_ch;
	printf("TXREQ: PILaddr=%x val=%x\n", phys_addr2, phys_addr);
	fsl_uspace_dma_add_entry(phys_addr + 4, phys_addr2,
			sizeof(phys_addr_t));

	/* VIRQ geneartion */
	if (ipc_ch->ipc_ind == OS_HET_VIRTUAL_INT) {
		phys_addr2 = ipc_priv.dsp_ccsr.phys_addr + GCR_OFFSET +
				ipc_ch->ind_offset;
		memcpy(((void *)(uint32_t)vaddr + 8), &ipc_ch->ind_value, 4);
		printf("TXREQ: INDaddr=%x val=%x\n", phys_addr2, phys_addr);
		fsl_uspace_dma_add_entry(phys_addr + 8, phys_addr2,
				sizeof(phys_addr_t));
	}
	fsl_uspace_dma_start();
	ipc_priv.txreq_inprocess = 1;

	EXIT(ret);
	return ret;
}

int fsl_ipc_get_last_tx_req_status()
{
	if (ipc_priv.txreq_inprocess) {
		if (fsl_uspace_dma_busy()) {
			printf(",");
			return TXREQ_IN_PROCESS;
		} else {
			ipc_priv.txreq_inprocess = 0;
			return TXREQ_DONE;
		}
	}

	return TXREQ_ERR;
}


void fsl_ipc_set_consumed_status(uint32_t channel_id)
{
	volatile os_het_ipc_channel_t *ipc_ch;

	ipc_ch =  get_channel_vaddr(channel_id);

	if (!OS_HET_CH_EMPTY(ipc_ch)) {
		OS_HET_INCREMENT_CONSUMER(ipc_ch);
		ipc_ch->LOCAL_CONSUMER_NUM = (ipc_ch->LOCAL_CONSUMER_NUM +
						1) % ipc_ch->bd_ring_size;
	}
}

void fsl_ipc_chk_recv_status(uint32_t *bmask)
{
	int i = 0;
	ipc_channel_us_t 	*ch;
	volatile os_het_ipc_channel_t 	*ipc_ch;
	ENTER();
	*bmask = 0;
	memset(bmask, 0, sizeof(uint32_t));
#ifdef	TEST_CH_ZERO
	{
		ipc_ch =  get_channel_vaddr(0);

		if (!OS_HET_CH_EMPTY(ipc_ch))
			*bmask |= 1 << 31;
	}
#endif
	/* Loop for all channels
	 * check for the availability */
	for (i = 0; i < ipc_priv.num_channels; i++) {
		ch = ipc_priv.channels[i];
		ipc_ch =  get_channel_vaddr(ch->channel_id);;
		printf("%d.", ch->channel_id);

		if (ipc_ch->tracker.producer_num >
			ipc_ch->tracker.consumer_num)
		*bmask |= 1<<(31 - ch->channel_id); /*FIXME*/
	}
	EXIT(*bmask);
}

/* Blocking calls*/
int fsl_ipc_send_ptr(uint32_t channel_id, phys_addr_t addr, uint32_t len)
{
	if (channel_id >= ipc_priv.num_channels)
		return ERR_CHANNEL_NOT_FOUND;

	volatile os_het_ipc_bd_t	*bd_base;
	volatile os_het_ipc_bd_t	*bd;
	volatile os_het_ipc_channel_t	*ipc_ch =  get_channel_vaddr(channel_id);

	/* check if the channel is init by the consumer */
	if (!ipc_ch->consumer_initialized) {
		printf("Error: consumer not initialized\n");
		return -1;
	}

	/* check if the channel is full */
	if (OS_HET_CH_FULL(ipc_ch))
		return ERR_CHANNEL_FULL;

	/* virtual address of the bd_ring pointed to by bd_base(phys_addr) */
	bd_base = SH_CTRL_VADDR(ipc_ch->bd_base);

	/*OLD bd = &bd_base[ipc_ch->tracker.producer_num]; */
	bd = &bd_base[ipc_ch->LOCAL_PRODUCER_NUM];
	bd->msg_ptr = addr;
	bd->msg_len = len;

	OS_HET_INCREMENT_PRODUCER(ipc_ch);
	ipc_ch->LOCAL_PRODUCER_NUM = (ipc_ch->LOCAL_PRODUCER_NUM + 1) % ipc_ch->bd_ring_size;

	if (ipc_ch->ipc_ind != OS_HET_NO_INT) {
		generate_indication(ipc_ch);
	}

	return 0;
}
void dump_ipc_channel(volatile os_het_ipc_channel_t *ipc_ch)
{
	printf("ipc_ch%x PI=%x CI=%x ID=%x PN=%x CN=%x LPI=%x LCI=%x BS=%x MX=%x CH=%x BD=%x II=%x IO=%x IV=%x",
	(uint32_t)ipc_ch,
	ipc_ch->producer_initialized,
	ipc_ch->consumer_initialized,
	ipc_ch->id,
	ipc_ch->tracker.producer_num,
	ipc_ch->tracker.consumer_num,
	ipc_ch->LOCAL_PRODUCER_NUM,
	ipc_ch->LOCAL_CONSUMER_NUM,
	ipc_ch->bd_ring_size,
	ipc_ch->max_msg_size,
	ipc_ch->ch_type,
	(uint32_t)ipc_ch->bd_base,
	ipc_ch->ipc_ind,
	ipc_ch->ind_offset,
	ipc_ch->ind_value);

}

void fsl_ipc_open_prod_ch(uint32_t channel_id)
{
	volatile os_het_ipc_channel_t *ipc_ch;
	ipc_ch =   get_channel_vaddr(channel_id);
	ipc_ch->producer_initialized = OS_HET_INITIALIZED;
}

int fsl_ipc_send_msg(uint32_t channel_id, void *src_buf_addr, uint32_t len)
{
	void *vaddr;
	volatile os_het_ipc_bd_t         *bd_base;
	volatile os_het_ipc_bd_t	*bd;
	volatile os_het_ipc_channel_t *ipc_ch;

	ENTER();

	ipc_ch =   get_channel_vaddr(channel_id);
	ipc_ch->producer_initialized = OS_HET_INITIALIZED;
	/* check if the channel is init by the consumer */
	if (!ipc_ch->consumer_initialized) {
		printf("Error: consumer not initialized\n");
		EXIT(-1);
		return -1;
	}
	printf("\n num free bds = %d \n", OS_HET_CH_FREE_BDS(ipc_ch));

	/* check if the channel is full */
	if (OS_HET_CH_FULL(ipc_ch)) {
		EXIT(-ERR_CHANNEL_FULL);
		return -ERR_CHANNEL_FULL;
	}

	/* virtual address of the bd_ring pointed to by bd_base(phys_addr) */
	bd_base = SH_CTRL_VADDR(ipc_ch->bd_base);
	printf("V= %x P=%x \n", ipc_ch->bd_base, bd_base);

	/*OLD bd = &bd_base[ipc_ch->tracker.producer_num];*/
	bd = &bd_base[ipc_ch->LOCAL_PRODUCER_NUM];

	printf("Vaddr = %x n\n", (uint32_t)bd);
	printf("MSG PTR = %x n\n", (uint32_t)bd->msg_ptr);
	/* get the virtual address of the msg ring */
	vaddr =  (*ipc_priv.p2vcb)(bd->msg_ptr);
	if (!vaddr) {
		EXIT(-1);
		return -1;
	}
	printf("Address of msg P=%x V= %x\n", bd->msg_ptr, (uint32_t)vaddr);
	memcpy(vaddr, src_buf_addr, len);
	bd->msg_len = len;

	OS_HET_INCREMENT_PRODUCER(ipc_ch);
	ipc_ch->LOCAL_PRODUCER_NUM = (ipc_ch->LOCAL_PRODUCER_NUM + 1) % ipc_ch->bd_ring_size;
	if (ipc_ch->ipc_ind != OS_HET_NO_INT) {
		generate_indication(ipc_ch);
	}

	EXIT(0);
	return 0;
}

int fsl_ipc_recv_ptr(uint32_t channel_id, phys_addr_t *addr, uint32_t *len)
{

	volatile os_het_ipc_bd_t	*bd_base;
	volatile os_het_ipc_bd_t	*bd;
	volatile os_het_ipc_channel_t	*ipc_ch;

	ENTER();

	ipc_ch =  get_channel_vaddr(channel_id);;

	/* check if the channel is full */
	if (OS_HET_CH_EMPTY(ipc_ch))
		return ERR_CHANNEL_EMPTY;

	bd_base = SH_CTRL_VADDR(ipc_ch->bd_base);
	/*bd = &bd_base[ipc_ch->tracker.consumer_num]; */
	bd = &bd_base[ipc_ch->LOCAL_CONSUMER_NUM];

	*addr = (phys_addr_t)bd->msg_ptr;
	*len = bd->msg_len;

	OS_HET_INCREMENT_CONSUMER(ipc_ch);
	ipc_ch->LOCAL_CONSUMER_NUM = (ipc_ch->LOCAL_CONSUMER_NUM + 1) % ipc_ch->bd_ring_size;
	EXIT(0);
	return 0;
}

int fsl_ipc_recv_ptr_hold(uint32_t channel_id, phys_addr_t *addr, uint32_t *len)
{

	volatile os_het_ipc_bd_t	*bd_base;
	volatile os_het_ipc_bd_t	*bd;
	volatile os_het_ipc_channel_t	*ipc_ch;

	ENTER();

	ipc_ch = get_channel_vaddr(channel_id);;

	/* check if the channel is full */
	if (OS_HET_CH_EMPTY(ipc_ch))
		return ERR_CHANNEL_EMPTY;

	bd_base = SH_CTRL_VADDR(ipc_ch->bd_base);
	/*bd = &bd_base[ipc_ch->tracker.consumer_num];*/
	bd = &bd_base[ipc_ch->LOCAL_CONSUMER_NUM];

	*addr = (phys_addr_t)bd->msg_ptr;
	*len = bd->msg_len;

	EXIT(0);
	return 0;
}

int fsl_ipc_recv_msg(uint32_t channel_id, void *addr, uint32_t *len)
{
	volatile os_het_ipc_bd_t	*bd_base;
	volatile os_het_ipc_bd_t	*bd;
	volatile void			*vaddr;
	volatile os_het_ipc_channel_t *ipc_ch =  get_channel_vaddr(channel_id);;

	/* check if the channel is full */
	if (OS_HET_CH_EMPTY(ipc_ch))
		return ERR_CHANNEL_EMPTY;

	bd_base = SH_CTRL_VADDR(ipc_ch->bd_base);
	/*bd = &bd_base[ipc_ch->tracker.consumer_num];*/
	bd = &bd_base[ipc_ch->LOCAL_CONSUMER_NUM];

	vaddr = (*ipc_priv.p2vcb)(bd->msg_ptr);
	*len = bd->msg_len;

	memcpy((void *)addr, (void *)vaddr, *len);

	OS_HET_INCREMENT_CONSUMER(ipc_ch);
	ipc_ch->LOCAL_CONSUMER_NUM = (ipc_ch->LOCAL_CONSUMER_NUM + 1) % ipc_ch->bd_ring_size;

	return 0;
}

int fsl_ipc_recv_msg_ptr(uint32_t channel_id, void *dst_buffer, uint32_t *len)
{
	volatile os_het_ipc_bd_t		*bd_base;
	volatile os_het_ipc_bd_t		*bd;
	volatile os_het_ipc_channel_t *ipc_ch =  get_channel_vaddr(channel_id);;

	if (!ipc_ch->producer_initialized)
		return -1;

	/* check if the channel is full */
	if (OS_HET_CH_EMPTY(ipc_ch))
		return ERR_CHANNEL_EMPTY;

	bd_base = SH_CTRL_VADDR(ipc_ch->bd_base);
	/* bd = &bd_base[ipc_ch->tracker.consumer_num]; */
	bd = &bd_base[ipc_ch->LOCAL_CONSUMER_NUM];

	dst_buffer = (*ipc_priv.p2vcb)(bd->msg_ptr);
	*len = bd->msg_len;

	return ERR_SUCCESS;
}

/*Internal API */
void generate_indication(volatile os_het_ipc_channel_t *ipc_ch)
{
	void *addr;
	uint32_t value = ipc_ch->ind_value;
	ENTER();
	if (ipc_ch->ipc_ind == OS_HET_VIRTUAL_INT) {
		addr = ipc_priv.dsp_ccsr.vaddr + GCR_OFFSET + ipc_ch->ind_offset;
		printf("Writing %x on %x\n", (uint32_t)addr, value);
		memcpy(addr, &value, sizeof(value));
	}
	EXIT(0);
}

uint32_t ipc_get_free_rt_signal(void)
{
	int free_sig = 0, i, rc;
	struct sigaction old;

	for (i = (SIGRTMIN + 4); i < SIGRTMAX ; i++) {
		rc = sigaction(i, NULL, &old);
		if (rc < 0)
			continue;
		if (old.sa_handler == SIG_IGN || old.sa_handler == SIG_DFL) {
			free_sig = i;
			break;
		}
	}
	return free_sig;
}

void signal_handler(int signo, siginfo_t *siginfo, void *data)
{
	int i;
	ipc_channel_us_t *ch = NULL;
	volatile os_het_ipc_channel_t 	*ipc_ch;
	volatile os_het_ipc_bd_t	*bd_base;
	volatile os_het_ipc_bd_t	*bd;
	void			*context;

	for (i = 0; i < ipc_priv.num_channels; i++) {
		ch = ipc_priv.channels[i];
		if (ch->signal == signo)
			break;
	}
	if (ch) {
		ipc_ch = get_channel_vaddr(ch->channel_id);
		bd_base = SH_CTRL_VADDR(ipc_ch->bd_base);
		bd	= &bd_base[ipc_ch->LOCAL_CONSUMER_NUM];

		if (ipc_ch->ch_type == OS_HET_IPC_MESSAGE_CH)
			context	= (*ipc_priv.p2vcb)(bd->msg_ptr);
		else
			context = bd->msg_ptr;
		if (ch->cbfunc)
			(*ch->cbfunc)(ch->channel_id, context, bd->msg_len);
	}
}

/*
 *@ channel_attach_msg_ring
 *
 *This function attaches the ptr's to the msg buffers to the pointer ring.
 *
 *Type: Internal
 */
int ipc_channel_attach_msg_ring(uint32_t channel_id, phys_addr_t msg_phys_addr,
				uint32_t msg_size)
{
	int depth;
	int err = ERR_SUCCESS;
	int i = 0;

	volatile os_het_ipc_channel_t 	*ch;
	volatile os_het_ipc_bd_t	*bd_base;
	volatile os_het_ipc_bd_t	*bd;

	ch =  get_channel_vaddr(channel_id);
	depth = ch->bd_ring_size;
	bd_base = SH_CTRL_VADDR(ch->bd_base);

	for (i = 0; i < depth; i++) {
		bd = &bd_base[i];
		bd->msg_ptr = msg_phys_addr;
		msg_phys_addr += msg_size;
	}

	return err;
}

void get_channels_info()
{
	ENTER();
	os_het_control_t *sh_ctrl =  ipc_priv.sh_ctrl_area.vaddr;
	os_het_ipc_t *ipc = SH_CTRL_VADDR(sh_ctrl->ipc);
	ipc_priv.max_channels = ipc->num_ipc_channels;
	ipc_priv.max_depth = ipc->ipc_max_bd_size;
	EXIT(0);
}
/*
 * @get_channel_paddr
 *
 * Returns the phyical address of the channel data structure in the
 * share control area.
 *
 * Type: Internal function
 */
phys_addr_t get_channel_paddr(uint32_t channel_id)
{
	os_het_ipc_channel_t 	*ch;
	phys_addr_t		phys_addr;

	ENTER();
	os_het_control_t *sh_ctrl =  ipc_priv.sh_ctrl_area.vaddr;
	os_het_ipc_t *ipc = SH_CTRL_VADDR(sh_ctrl->ipc);
	phys_addr = (phys_addr_t)ipc->ipc_channels + sizeof(os_het_ipc_channel_t)*channel_id;

	EXIT(phys_addr);
	return phys_addr;
}
/*
 * @get_channel_vaddr
 *
 * Returns the virtual address of the channel data structure in the
 * share control area.
 *
 * Type: Internal function
 */
void *get_channel_vaddr(uint32_t channel_id)
{
	void *vaddr;
	ENTER();
	vaddr = SH_CTRL_VADDR(get_channel_paddr(channel_id));
	EXIT(vaddr);
	return vaddr;
}
/*
 * @init_het_ipc
 *
 */
int init_het_ipc()
{
	int ret = ERR_SUCCESS;
	ENTER();

	ipc_priv.dev_het_ipc = open("/dev/het_ipc", O_RDWR);
	if (ipc_priv.dev_het_ipc == -1) {
		printf("Error: Cannot open /dev/het_ipc. %d\n");
		ret = -ERR_DEV_HETIPC_FAIL;
	}
	EXIT(ret);
	return ret;
}
