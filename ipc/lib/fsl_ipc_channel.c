/*
 *  @ fsl_ipc_channel.c
 *
 * Copyright 2011-2013 Freescale Semiconductor, Inc.
 *
 * Author: Manish Jaggi <manish.jaggi@freescale.com>
 * Author: Ashish Kumar <ashish.kumar@freescale.com>
 *
 */
#include <stdio.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <string.h>
#include <stdint.h>
#include "logdefs.h"
#include "fsl_het_mgr.h"
#include "fsl_user_dma.h"
#include "fsl_ipc_um.h"
#include "fsl_usmmgr.h"
#include "fsl_ipc_lock.h"
#include "fsl_ipc_kmod.h"
#include "fsl_bsc913x_ipc.h"
#include "fsl_ipc_errorcodes.h"
#include "bsc913x_heterogeneous.h"
#include "bsc913x_heterogeneous_ipc.h"

#define LOCAL_PRODUCER_NUM pa_reserved[0]
#define LOCAL_CONSUMER_NUM pa_reserved[1]

range_t chvpaddr_arr[TOTAL_IPC_CHANNELS];
int ch_semid[TOTAL_IPC_CHANNELS];

/*********** Defines ******************/
#define MAX_MSG_SIZE 1020
#define PAGE_SIZE 4096
#define GCR_OFFSET 0x17000

#define SH_CTRL_VADDR(A) \
	(void *)((phys_addr_t)(A) \
			- (ipc_priv->sh_ctrl_area.phys_addr) \
			+  ipc_priv->sh_ctrl_area.vaddr)

int init_het_ipc(ipc_userspace_t *ipc);
uint32_t ipc_get_free_rt_signal(void);
static void *get_channel_vaddr(uint32_t channel_id,
		ipc_userspace_t *ipc_priv);
static void *__get_channel_vaddr(uint32_t channel_id,
		ipc_userspace_t *ipc_priv);
static phys_addr_t get_channel_paddr(uint32_t channel_id,
		ipc_userspace_t *ipc_priv);
static phys_addr_t __get_channel_paddr(uint32_t channel_id,
		ipc_userspace_t *ipc_priv);
int get_ipc_vaddr(ipc_userspace_t *ipc_priv, int inst_id);
int get_ipc_inst(ipc_userspace_t *ipc_priv, uint32_t inst_id);
int get_channels_info(ipc_userspace_t *ipc, uint32_t rat_id);
void generate_indication(os_het_ipc_channel_t *ipc_ch,
		ipc_userspace_t *ipc_priv);
void signal_handler(int signo, siginfo_t *siginfo, void *data);
void dump_ipc_channel(os_het_ipc_channel_t *);

/***** Implementation ******************/
fsl_ipc_t fsl_ipc_init(ipc_p2v_t p2vcb, range_t sh_ctrl_area,
		range_t dsp_ccsr, range_t pa_ccsr)
{
	uint32_t rat_id = 0;
	return fsl_ipc_init_rat(rat_id, p2vcb, sh_ctrl_area,
		dsp_ccsr, pa_ccsr);
}


fsl_ipc_t fsl_ipc_init_rat(uint32_t rat_id, ipc_p2v_t p2vcb,
	range_t sh_ctrl_area, range_t dsp_ccsr, range_t pa_ccsr)
{
	int ret = ERR_SUCCESS;
	ipc_userspace_t *ipc_priv = NULL;
	int i;
	ENTER();

	if (!p2vcb) {
		ret = -ERR_P2V_NULL;
		goto end;
	}

	/* Allocate memory for ipc instance */
	ipc_priv = malloc(sizeof(ipc_userspace_t));
	if (!ipc_priv)
		goto end;

	memset(ipc_priv, 0, sizeof(ipc_userspace_t));
	ipc_priv->p2vcb = p2vcb;

	memcpy(&ipc_priv->sh_ctrl_area, &sh_ctrl_area, sizeof(range_t));
	memcpy(&ipc_priv->dsp_ccsr, &dsp_ccsr, sizeof(range_t));
	memcpy(&ipc_priv->pa_ccsr, &pa_ccsr, sizeof(range_t));

	/* Init /dev/het_ipc */
	ret = init_het_ipc(ipc_priv);
	if (ret)
		goto end;

	/* Read number of channels and
	   max msg size from sh_ctrl_area */
	ret = get_channels_info(ipc_priv, rat_id);

	if (ret)
		goto end;

	for (i = 0; i < TOTAL_IPC_CHANNELS; i++) {
		chvpaddr_arr[i].phys_addr = __get_channel_paddr(i, ipc_priv);
		chvpaddr_arr[i].vaddr = __get_channel_vaddr(i, ipc_priv);
	}
	ipc_priv->rat_id = rat_id;

#ifdef CONFIG_LOCK
	for (i = 0; i < TOTAL_IPC_CHANNELS; i++)
		ch_semid[i] = fsl_ipc_init_lock(chvpaddr_arr[i].phys_addr);
#endif
end:
	EXIT(ret);
	if (ret) /* if ret non zero free ipc_priv */
		if (ipc_priv) {
			if (!(ipc_priv->dev_het_ipc == -1))
				close(ipc_priv->dev_het_ipc);
			free(ipc_priv);
			ipc_priv = NULL;
		}

	return ipc_priv;
}

void fsl_ipc_exit(fsl_ipc_t ipc)
{
	int i;
	ipc_userspace_t *ipc_priv = (ipc_userspace_t *)ipc;

	/* close het_ipc */
	close(ipc_priv->dev_het_ipc);

#ifdef CONFIG_LOCK
	for (i = 0; i < TOTAL_IPC_CHANNELS; i++)
		fsl_ipc_destroy_lock(ch_semid[i]);
#endif

	/* free memory */
	for (i = 0; i < ipc_priv->num_channels; i++)
		free(ipc_priv->channels[i]);

	/* free ipc */
	free(ipc_priv);
}

/*
 * @channel_attach_msg_ring
 *
 * This function attaches the ptr's to the msg buffers to the pointer ring.
 *
 * Type: Internal
 * It is assumed that the lock is taken by the caller.
 */
int ipc_channel_attach_msg_ring(uint32_t channel_id, phys_addr_t msg_phys_addr,
		uint32_t msg_size, ipc_userspace_t *ipc_priv)
{
	int depth;
	int ret = ERR_SUCCESS;
	int i = 0;

	os_het_ipc_channel_t 	*ch;
	os_het_ipc_bd_t	*bd_base;
	os_het_ipc_bd_t	*bd;

	ch =  get_channel_vaddr(channel_id, ipc_priv);
	depth = ch->bd_ring_size;
	bd_base = SH_CTRL_VADDR(ch->bd_base);

	for (i = 0; i < depth; i++) {
		bd = &bd_base[i];
		bd->msg_ptr = msg_phys_addr;
		msg_phys_addr += msg_size;
	}

	return ret;
}

int fsl_ipc_configure_channel(uint32_t channel_id, uint32_t depth,
		ipc_ch_type_t channel_type,
		phys_addr_t msg_ring_paddr, uint32_t msg_size,
		ipc_cbfunc_t cbfunc, fsl_ipc_t ipc)
{
	int 				ret = ERR_SUCCESS;
	ipc_channel_us_t		*uch;
	os_het_ipc_channel_t 		*ch;
	ipc_userspace_t			*ipc_priv;
	int locked = 0;

	ENTER();
	ipc_priv = (ipc_userspace_t *) ipc;

	ch = get_channel_vaddr(channel_id, ipc_priv);

	ret = fsl_ipc_lock(ch_semid[channel_id]);
	if (ret < 0) {
		EXIT(ret);
		goto end;
	}
	locked = 1;

	if (ch->consumer_initialized != OS_HET_INITIALIZED) {
		if (ch->bd_ring_size <= ipc_priv->max_depth)
			ch->bd_ring_size = depth;
		else {
			ret = -ERR_INVALID_DEPTH;
			EXIT(ret);
			goto end;
		}
		ch->ch_type = channel_type;
		if (channel_type == IPC_MSG_CH)
			ipc_channel_attach_msg_ring(channel_id, msg_ring_paddr,
					msg_size, ipc_priv);

		ch->ipc_ind = OS_HET_NO_INT;
	}

	debug_print("/* allocate ipc_channel_us_t */\n");
	uch = calloc(1, sizeof(ipc_channel_us_t));
	if (!uch) {
		debug_print("Memory Allocation Failure with error %d\n",
				errno);
		ret = -ERR_CALLOC;
		EXIT(ret);
		goto end;
	}

	debug_print("/* attach the channel to the list */\n");
	ipc_priv->channels[ipc_priv->num_channels++] = uch;

	debug_print("/* fill the channel structure */");
	uch->channel_id = channel_id;
	ch->consumer_initialized = OS_HET_INITIALIZED;
end:
	if (locked)
		fsl_ipc_unlock(ch_semid[channel_id]);

#if DEBUG_RELOAD
	printf("dump_ipc_channel_ in %s\n", __func__);
	dump_ipc_channel(ch);
#endif

	EXIT(ret);
	return ret;
}

int fsl_ipc_configure_txreq(uint32_t channel_id, phys_addr_t phys_addr,
		uint32_t max_txreq_lbuff_size, fsl_ipc_t ipc)
{
	int ret;
	int locked = 0;
	phys_addr_t phys_addr_s;
	range_t dma_list_mem;
	os_het_ipc_channel_t 	*ipc_ch;
	ipc_userspace_t		*ipc_priv;

	ENTER();

	ipc_priv = (ipc_userspace_t *) ipc;
	if (channel_id >= ipc_priv->max_channels) {
		ret = -ERR_CHANNEL_NOT_FOUND;
		EXIT(ret);
		goto end;
	}

	ipc_ch = get_channel_vaddr(channel_id, ipc_priv);

	ret = fsl_ipc_lock(ch_semid[channel_id]);
	if (ret < 0) {
		EXIT(ret);
		goto end;
	}
	locked = 1;

	if (ipc_ch->producer_initialized != OS_HET_INITIALIZED) {
		printf("Producer must open TxReq channel %d before doing "
				"its configuration\n", channel_id);
		ret =  -ERR_PRODUCER_NOT_INIT;
		EXIT(ret);
		goto end;
	}

	debug_print("Params %x %x %x \n", channel_id, phys_addr,
			max_txreq_lbuff_size);

	ipc_priv->txreq_tb_lbuff_paddr = phys_addr;
	ipc_priv->max_txreq_lbuff_size = max_txreq_lbuff_size;

	/* Get spare area vaddr */
	/*FIXME: Currently extra memory is taken for saving dma descriptors*/
	phys_addr_s = ipc_priv->txreq_tb_lbuff_paddr +
		(ipc_ch->bd_ring_size) * ipc_priv->max_txreq_lbuff_size;

	debug_print("Phys_addr_s =%x\n", phys_addr_s);

	/*Making room for some extra variables, and making it aligned*/
	phys_addr_s += 32*sizeof(phys_addr_t);

	if (phys_addr_s % 32)
		phys_addr_s += (32  - phys_addr_s % 32);

	dma_list_mem.phys_addr = phys_addr_s;

	dma_list_mem.vaddr = (*ipc_priv->p2vcb)(phys_addr_s);

	ipc_priv->udma = fsl_uspace_dma_init(dma_list_mem, ipc_priv->pa_ccsr
			, ipc_priv->rat_id);

end:
	if (locked)
		fsl_ipc_unlock(ch_semid[channel_id]);

	EXIT(ret);
	return ret;
}

int fsl_ipc_send_tx_req(uint32_t channel_id, sg_list_t *sgl,
		void *tx_req_vaddr, uint32_t tx_req_len,
		fsl_ipc_t ipc)
{
	int 				ret;
	uint32_t			ctr;
	uint32_t			incr1, incr2;
	void				*vaddr;
	phys_addr_t			phys_addr;
	phys_addr_t			phys_addr2;
	os_het_ipc_bd_t			*bd_base;
	os_het_ipc_bd_t			*bd;
	os_het_ipc_channel_t		*ipc_ch;
	ipc_userspace_t 		*ipc_priv;
	int locked = 0;
	ENTER();

	ipc_priv = (ipc_userspace_t *)ipc;

	ipc_ch = get_channel_vaddr(channel_id, ipc_priv);
	if (ipc_ch->consumer_initialized != OS_HET_INITIALIZED) {
		debug_print("Error: consumer not initialized\n");
		ret = -ERR_CONSUMER_NOT_INIT;
		EXIT(ret);
		goto end;
	}

	ret = fsl_ipc_lock(ch_semid[channel_id]);
	if (ret < 0) {
		EXIT(ret);
		goto end;
	}
	locked = 1;

	/* check if the channel is full */
	if (OS_HET_CH_FULL(ipc_ch)) {
		ret = -ERR_CHANNEL_FULL;
		EXIT(ret);
		goto end;
	}

	fsl_uspace_dma_list_clear(ipc_priv->udma);

	/* copy txreq */
	bd_base = SH_CTRL_VADDR(ipc_ch->bd_base);
	/*OLD	bd = &bd_base[ipc_ch->tracker.producer_num]; */
	bd = &bd_base[ipc_ch->LOCAL_PRODUCER_NUM];

	phys_addr = bd->msg_ptr;
	bd->msg_len = tx_req_len;
	vaddr = (*ipc_priv->p2vcb)(phys_addr);
	if (!vaddr) {
		ret = -1;
		EXIT(ret);
		goto end;
	}

	debug_print("copying %x to %x length %x\n",
			vaddr, tx_req_vaddr, tx_req_len);
	memcpy(vaddr, tx_req_vaddr, tx_req_len);

	/*OLD	phys_addr = ipc_priv->txreq_tb_lbuff_paddr +
	  ipc_ch->tracker.producer_num*ipc_priv->max_txreq_lbuff_size;*/
	phys_addr = ipc_priv->txreq_tb_lbuff_paddr +
		ipc_ch->LOCAL_PRODUCER_NUM * ipc_priv->max_txreq_lbuff_size;

	/*write the lbuff address at the end of the message */
	debug_print("copying %x to %x length %x\n", ((uint32_t)vaddr +
				MAX_TX_REQ_MSG_SIZE), phys_addr,
			sizeof(phys_addr_t));
	memcpy((void *)((uint32_t)vaddr + MAX_TX_REQ_MSG_SIZE),
			&phys_addr, sizeof(phys_addr_t));

	ctr = 0;
	while (sgl->entry[ctr].is_valid) {
		debug_print("%x %x %x\n", sgl->entry[ctr].is_valid,
				sgl->entry[ctr].src_addr, sgl->entry[ctr].len);
		if (sgl->entry[ctr].is_tb_start)	{
			/*check for alignment*/
		}
		fsl_uspace_dma_add_entry(sgl->entry[ctr].src_addr, phys_addr,
				sgl->entry[ctr].len, ipc_priv->udma);
		phys_addr += sgl->entry[ctr].len;
		ctr++;
		/* Alignment */
	}
	/* Get spare area vaddr */
	phys_addr = ipc_priv->txreq_tb_lbuff_paddr +
		(ipc_ch->bd_ring_size) *
		ipc_priv->max_txreq_lbuff_size;

	vaddr = (*ipc_priv->p2vcb)(phys_addr);
	incr1 = ipc_ch->tracker.producer_num + 1;
	incr2 = (ipc_ch->LOCAL_PRODUCER_NUM + 1) % ipc_ch->bd_ring_size;

	debug_print("## Writing P=%x V=%x #=%x\n",
			phys_addr, vaddr, incr1);
	/* Add producer increment */
	memcpy(vaddr, &incr1, sizeof(uint32_t));
	/* Get physical address of producer_num */
	phys_addr2 = get_channel_paddr(channel_id, ipc_priv);
	debug_print("TXREQ 0: %x %x\n", phys_addr2, channel_id);
	phys_addr2 += (uint32_t)&ipc_ch->tracker.producer_num -
		(uint32_t)ipc_ch;
	debug_print("TXREQ: PIaddr=%x val=%x\n",
			phys_addr2, phys_addr);

	fsl_uspace_dma_add_entry(phys_addr, phys_addr2,
			sizeof(phys_addr_t), ipc_priv->udma);

	/* Get physical address of LOCAL_PRODUCER_NUM */
	memcpy((void *)((uint32_t)vaddr + 4), &incr2, sizeof(uint32_t));
	debug_print("## Writing V=%x P=%x #=%x\n",
			phys_addr + 4, vaddr + 4, incr2);

	phys_addr2 = get_channel_paddr(channel_id, ipc_priv);
	debug_print("TXREQ 0: %x %x\n", phys_addr2, channel_id);

	phys_addr2 += (uint32_t)&ipc_ch->LOCAL_PRODUCER_NUM - (uint32_t)ipc_ch;
	debug_print("TXREQ: PILaddr=%x val=%x\n", phys_addr2, phys_addr);

	fsl_uspace_dma_add_entry(phys_addr + 4, phys_addr2,
			sizeof(phys_addr_t), ipc_priv->udma);

	/* VIRQ geneartion */
	if (ipc_ch->ipc_ind == OS_HET_VIRTUAL_INT) {
		phys_addr2 = ipc_priv->dsp_ccsr.phys_addr + GCR_OFFSET +
			ipc_ch->ind_offset;
		memcpy(((void *)(uint32_t)vaddr + 8), &ipc_ch->ind_value, 4);
		debug_print("TXREQ: INDaddr=%x val=%x\n",
				phys_addr2, phys_addr);

		fsl_uspace_dma_add_entry(phys_addr + 8, phys_addr2,
				sizeof(phys_addr_t), ipc_priv->udma);
	}

	fsl_uspace_dma_start(ipc_priv->udma);
	ipc_priv->txreq_inprocess = 1;

end:
	if (locked)
		fsl_ipc_unlock(ch_semid[channel_id]);

	EXIT(ret);
	return ret;
}

int fsl_ipc_get_last_tx_req_status(fsl_ipc_t ipc)
{
	ipc_userspace_t 	*ipc_priv;
	int ret = TXREQ_ERR;
	ENTER();

	ipc_priv = (ipc_userspace_t *)ipc;

	if (ipc_priv->txreq_inprocess) {
		if (fsl_uspace_dma_busy(ipc_priv->udma)) {
			ret = TXREQ_IN_PROCESS;
		} else {
			ipc_priv->txreq_inprocess = 0;
			ret = TXREQ_DONE;
		}
	}

	EXIT(ret);
	return ret;
}


int fsl_ipc_set_consumed_status(uint32_t channel_id, fsl_ipc_t ipc)
{
	os_het_ipc_channel_t	*ipc_ch;
	ipc_userspace_t		*ipc_priv;
	int locked = 0;
	int ret = ERR_SUCCESS;
	ENTER();

	ipc_priv = (ipc_userspace_t *)ipc;

	ipc_ch = get_channel_vaddr(channel_id, ipc_priv);

	ret = fsl_ipc_lock(ch_semid[channel_id]);
	if (ret < 0) {
		EXIT(ret);
		goto end;
	}
	locked = 1;

	if (!OS_HET_CH_EMPTY(ipc_ch)) {
		OS_HET_INCREMENT_CONSUMER(ipc_ch);
		ipc_ch->LOCAL_CONSUMER_NUM = (ipc_ch->LOCAL_CONSUMER_NUM +
				1) % ipc_ch->bd_ring_size;
	}

end:
	if (locked)
		fsl_ipc_unlock(ch_semid[channel_id]);

	return ret;
}

int fsl_ipc_chk_recv_status(uint64_t *bmask, fsl_ipc_t ipc)
{
	int i = 0;
	ipc_channel_us_t	*ch;
	ipc_userspace_t		*ipc_priv;
	os_het_ipc_channel_t	*ipc_ch;

	ENTER();
	ipc_priv = (ipc_userspace_t *)ipc;

	*bmask = 0;
	memset(bmask, 0, sizeof(uint64_t));
#ifdef	TEST_CH_ZERO
	{
		ipc_ch = get_channel_vaddr(0, ipc_priv);

		if (!OS_HET_CH_EMPTY(ipc_ch)) {
			*bmask |= ((uint64_t)1) << 63;
		}
	}
#endif
	/* Loop for all channels
	 * check for the availability */
	for (i = 0; i < ipc_priv->num_channels; i++) {
		ch = ipc_priv->channels[i];
		ipc_ch =  get_channel_vaddr(ch->channel_id, ipc_priv);
		debug_print("%d.\n", ch->channel_id);

		if (!OS_HET_CH_EMPTY(ipc_ch))
			*bmask |=
				((uint64_t)1)<<(63 - ch->channel_id);
	}
	EXIT(0);
	return ERR_SUCCESS;
}

/* Blocking calls*/
int fsl_ipc_send_ptr(uint32_t channel_id, phys_addr_t addr, uint32_t len,
		fsl_ipc_t ipc)
{
	os_het_ipc_bd_t	*bd_base;
	os_het_ipc_bd_t	*bd;
	os_het_ipc_channel_t	*ipc_ch;
	ipc_userspace_t 		*ipc_priv;
	int ret = ERR_SUCCESS;
	int locked = 0;
	ENTER();

	ipc_priv = (ipc_userspace_t *)ipc;

	ipc_ch = get_channel_vaddr(channel_id, ipc_priv);

	/* check if the channel is init by the consumer */
	if (ipc_ch->consumer_initialized != OS_HET_INITIALIZED) {
		debug_print("Error: consumer not initialized\n");
		ret = -ERR_CONSUMER_NOT_INIT;
		EXIT(ret);
		goto end;
	}

	ret = fsl_ipc_lock(ch_semid[channel_id]);
	if (ret < 0) {
		EXIT(ret);
		goto end;
	}
	locked = 1;

	/* check if the channel is full */
	if (OS_HET_CH_FULL(ipc_ch)) {
		ret = -ERR_CHANNEL_FULL;
		EXIT(ret);
		goto end;
	}


	/* virtual address of the bd_ring pointed to by bd_base(phys_addr) */
	bd_base = SH_CTRL_VADDR(ipc_ch->bd_base);

	/*OLD bd = &bd_base[ipc_ch->tracker.producer_num]; */
	bd = &bd_base[ipc_ch->LOCAL_PRODUCER_NUM];
	bd->msg_ptr = addr;
	bd->msg_len = len;

	OS_HET_INCREMENT_PRODUCER(ipc_ch);
	ipc_ch->LOCAL_PRODUCER_NUM = (ipc_ch->LOCAL_PRODUCER_NUM + 1) % ipc_ch->bd_ring_size;

	if (ipc_ch->ipc_ind != OS_HET_NO_INT) {
		generate_indication(ipc_ch, ipc_priv);
	}

end:
	if (locked)
		fsl_ipc_unlock(ch_semid[channel_id]);

	return ret;
}

void dump_ipc_channel(os_het_ipc_channel_t *ipc_ch)
{
	printf("ipc_ch%x PI=%x CI=%x ID=%x PN=%x CN=%x LPI=%x LCI=%x \
			BS=%x MX=%x CH=%x BD=%x II=%x IO=%x IV=%x",
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

int fsl_ipc_open_prod_ch(uint32_t channel_id, fsl_ipc_t ipc)
{
	os_het_ipc_channel_t	*ipc_ch;
	ipc_userspace_t		*ipc_priv;

	ENTER();

	ipc_priv = (ipc_userspace_t *)ipc;
	ipc_ch = get_channel_vaddr(channel_id, ipc_priv);

	ipc_ch->producer_initialized = OS_HET_INITIALIZED;

	EXIT(0);
	return ERR_SUCCESS;
}

int fsl_ipc_send_msg(uint32_t channel_id, void *src_buf_addr, uint32_t len,
		fsl_ipc_t ipc)
{
	void *vaddr;
	int ret = ERR_SUCCESS;
	int locked = 0;
	os_het_ipc_bd_t	*bd_base;
	os_het_ipc_bd_t	*bd;
	os_het_ipc_channel_t	*ipc_ch;
	ipc_userspace_t		*ipc_priv;

	ENTER();

	ipc_priv = (ipc_userspace_t *)ipc;

	ipc_ch = get_channel_vaddr(channel_id, ipc_priv);

	ipc_ch->producer_initialized = OS_HET_INITIALIZED;
	/* check if the channel is init by the consumer */
	if (ipc_ch->consumer_initialized != OS_HET_INITIALIZED) {
		debug_print("Error: consumer not initialized\n");
		ret = -ERR_CONSUMER_NOT_INIT;
		EXIT(ret);
		goto end;
	}
	debug_print("\n num free bds = %d \n", OS_HET_CH_FREE_BDS(ipc_ch));

	ret = fsl_ipc_lock(ch_semid[channel_id]);
	if (ret < 0) {
		EXIT(ret);
		goto end;
	}
	locked = 1;

	/* check if the channel is full */
	if (OS_HET_CH_FULL(ipc_ch)) {
		ret = -ERR_CHANNEL_FULL;
		EXIT(ret);
		goto end;
	}

	/* virtual address of the bd_ring pointed to by bd_base(phys_addr) */
	bd_base = SH_CTRL_VADDR(ipc_ch->bd_base);
	debug_print("V= %x P=%x \n", ipc_ch->bd_base, bd_base);


	/*OLD bd = &bd_base[ipc_ch->tracker.producer_num];*/
	bd = &bd_base[ipc_ch->LOCAL_PRODUCER_NUM];

	debug_print("Vaddr = %x n\n", (uint32_t)bd);
	debug_print("MSG PTR = %x n\n", (uint32_t)bd->msg_ptr);
	/* get the virtual address of the msg ring */
	vaddr =  (*ipc_priv->p2vcb)(bd->msg_ptr);
	if (!vaddr) {
		ret = -1;
		EXIT(ret);
		goto end;
	}
	debug_print("Address of msg P=%x V= %x\n",
			bd->msg_ptr, (uint32_t)vaddr);
	memcpy(vaddr, src_buf_addr, len);
	bd->msg_len = len;

	OS_HET_INCREMENT_PRODUCER(ipc_ch);
	ipc_ch->LOCAL_PRODUCER_NUM = (ipc_ch->LOCAL_PRODUCER_NUM + 1) %
		ipc_ch->bd_ring_size;

	if (ipc_ch->ipc_ind != OS_HET_NO_INT) {
		generate_indication(ipc_ch, ipc_priv);
	}

end:
	if (locked)
		fsl_ipc_unlock(ch_semid[channel_id]);
	return ret;
}

int fsl_ipc_recv_ptr(uint32_t channel_id, phys_addr_t *addr, uint32_t *len,
		fsl_ipc_t ipc)
{
	int ret = ERR_SUCCESS;
	os_het_ipc_bd_t	*bd_base;
	os_het_ipc_bd_t	*bd;
	os_het_ipc_channel_t 	*ipc_ch;
	ipc_userspace_t 		*ipc_priv;
	int locked = 0;
	ENTER();

	ipc_priv = (ipc_userspace_t *)ipc;
	if (channel_id >= ipc_priv->max_channels) {
		ret = -ERR_CHANNEL_NOT_FOUND;
		EXIT(ret);
		goto end;
	}
	ipc_ch = get_channel_vaddr(channel_id, ipc_priv);
	if (ipc_ch->producer_initialized != OS_HET_INITIALIZED) {
		debug_print("Error: producer not initialized\n");
		ret = -ERR_PRODUCER_NOT_INIT;
		EXIT(ret);
		goto end;

	}

	ret = fsl_ipc_lock(ch_semid[channel_id]);
	if (ret < 0) {
		EXIT(ret);
		goto end;
	}
	locked = 1;

	/* check if the channel is full */
	if (OS_HET_CH_EMPTY(ipc_ch)) {
		ret = -ERR_CHANNEL_EMPTY;
		EXIT(ret);
		goto end;
	}
	bd_base = SH_CTRL_VADDR(ipc_ch->bd_base);
	/*bd = &bd_base[ipc_ch->tracker.consumer_num]; */
	bd = &bd_base[ipc_ch->LOCAL_CONSUMER_NUM];

	*addr = (phys_addr_t)bd->msg_ptr;
	*len = bd->msg_len;

	OS_HET_INCREMENT_CONSUMER(ipc_ch);
	ipc_ch->LOCAL_CONSUMER_NUM = (ipc_ch->LOCAL_CONSUMER_NUM + 1) %
		ipc_ch->bd_ring_size;

end:
	if (locked)
		fsl_ipc_unlock(ch_semid[channel_id]);

	EXIT(ret);
	return ret;
}

int fsl_ipc_recv_ptr_hold(uint32_t channel_id, phys_addr_t *addr, uint32_t *len,
		fsl_ipc_t ipc)
{
	int ret = ERR_SUCCESS;
	int locked = 0;
	os_het_ipc_bd_t	*bd_base;
	os_het_ipc_bd_t	*bd;
	os_het_ipc_channel_t 	*ipc_ch;
	ipc_userspace_t 	*ipc_priv;

	ENTER();

	ipc_priv = (ipc_userspace_t *)ipc;

	ipc_ch = get_channel_vaddr(channel_id, ipc_priv);

	ret = fsl_ipc_lock(ch_semid[channel_id]);
	if (ret < 0) {
		EXIT(ret);
		goto end;
	}
	locked = 1;

	/* check if the channel is full */
	if (OS_HET_CH_EMPTY(ipc_ch)) {
		ret =  -ERR_CHANNEL_EMPTY;
		EXIT(ret);
		goto end;
	}

	bd_base = SH_CTRL_VADDR(ipc_ch->bd_base);
	/*bd = &bd_base[ipc_ch->tracker.consumer_num];*/
	bd = &bd_base[ipc_ch->LOCAL_CONSUMER_NUM];

	*addr = (phys_addr_t)bd->msg_ptr;
	*len = bd->msg_len;

end:
	if (locked)
		fsl_ipc_unlock(ch_semid[channel_id]);
	return ret;
}

int fsl_ipc_recv_msg(uint32_t channel_id, void *addr, uint32_t *len,
		fsl_ipc_t ipc)
{
	int ret = ERR_SUCCESS;
	int locked = 0;
	os_het_ipc_bd_t	*bd_base;
	os_het_ipc_bd_t	*bd;
	void			*vaddr;
	os_het_ipc_channel_t 	*ipc_ch;
	ipc_userspace_t 		*ipc_priv;

	ENTER();

	ipc_priv = (ipc_userspace_t *)ipc;

	ipc_ch = get_channel_vaddr(channel_id, ipc_priv);

	ret = fsl_ipc_lock(ch_semid[channel_id]);
	if (ret < 0) {
		EXIT(ret);
		goto end;
	}
	locked = 1;

	/* check if the channel is full */
	if (OS_HET_CH_EMPTY(ipc_ch)) {
		ret = -ERR_CHANNEL_EMPTY;
		EXIT(ret);
		goto end;
	}

	bd_base = SH_CTRL_VADDR(ipc_ch->bd_base);
	/*bd = &bd_base[ipc_ch->tracker.consumer_num];*/
	bd = &bd_base[ipc_ch->LOCAL_CONSUMER_NUM];

	vaddr = (*ipc_priv->p2vcb)(bd->msg_ptr);
	*len = bd->msg_len;

	memcpy((void *)addr, (void *)vaddr, *len);

	OS_HET_INCREMENT_CONSUMER(ipc_ch);
	ipc_ch->LOCAL_CONSUMER_NUM = (ipc_ch->LOCAL_CONSUMER_NUM + 1) %
		ipc_ch->bd_ring_size;

end:
	if (locked)
		fsl_ipc_unlock(ch_semid[channel_id]);
	return ret;
}

int fsl_ipc_recv_msg_ptr(uint32_t channel_id, void **dst_buffer,
		uint32_t *len, fsl_ipc_t ipc)
{
	int ret = ERR_SUCCESS;
	int locked = 0;
	os_het_ipc_bd_t		*bd_base;
	os_het_ipc_bd_t		*bd;
	os_het_ipc_channel_t 	*ipc_ch;
	ipc_userspace_t 	*ipc_priv;

	ENTER();

	ipc_priv = (ipc_userspace_t *)ipc;

	ipc_ch = get_channel_vaddr(channel_id, ipc_priv);

	if (!ipc_ch->producer_initialized) {
		ret = -ERR_PRODUCER_NOT_INIT;
		EXIT(ret);
		goto end;
	}

	ret = fsl_ipc_lock(ch_semid[channel_id]);
	if (ret < 0) {
		EXIT(ret);
		goto end;
	}
	locked = 1;

	/* check if the channel is full */
	if (OS_HET_CH_EMPTY(ipc_ch)) {
		ret = -ERR_CHANNEL_EMPTY;
		EXIT(ret);
		goto end;
	}

	bd_base = SH_CTRL_VADDR(ipc_ch->bd_base);
	/* bd = &bd_base[ipc_ch->tracker.consumer_num]; */
	bd = &bd_base[ipc_ch->LOCAL_CONSUMER_NUM];

	*dst_buffer = (*ipc_priv->p2vcb)(bd->msg_ptr);
	*len = bd->msg_len;

end:
	if (locked)
		fsl_ipc_unlock(ch_semid[channel_id]);
	return ret;
}
/**************** Internal API ************************/
/*
 * @generate_indication
 *
 * This function sends an interrupt to DSP via VIRQ.
 *
 * Type: Internal
 */
void generate_indication(os_het_ipc_channel_t *ipc_ch,
		ipc_userspace_t *ipc_priv)
{
	void *addr;
	uint32_t value = ipc_ch->ind_value;
	ENTER();
	if (ipc_ch->ipc_ind == OS_HET_VIRTUAL_INT) {
		addr = ipc_priv->dsp_ccsr.vaddr + GCR_OFFSET +
			ipc_ch->ind_offset;
		debug_print("Writing %x on %x\n", value, (uint32_t)addr);
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
	os_het_ipc_channel_t 	*ipc_ch;
	os_het_ipc_bd_t	*bd_base;
	os_het_ipc_bd_t	*bd;
	void		*context;

	ipc_channel_us_t *ch 		= NULL;
	ipc_userspace_t	*ipc_priv = (ipc_userspace_t *)data;

	for (i = 0; i < ipc_priv->num_channels; i++) {
		ch = ipc_priv->channels[i];
		if (ch->signal == signo)
			break;
	}
	if (ch) {
		ipc_ch = get_channel_vaddr(ch->channel_id, ipc_priv);
		bd_base = SH_CTRL_VADDR(ipc_ch->bd_base);
		bd	= &bd_base[ipc_ch->LOCAL_CONSUMER_NUM];

		if (ipc_ch->ch_type == OS_HET_IPC_MESSAGE_CH)
			context	= (*ipc_priv->p2vcb)(bd->msg_ptr);
		else
			context = (void *)bd->msg_ptr;
		if (ch->cbfunc)
			(*ch->cbfunc)(ch->channel_id, context, bd->msg_len);
	}
}

/*
 * @get_channels_info
 *
 * Read number of channels and max msg size from sh_ctrl_area
 *
 * Type: Internal function
 */

int get_ipc_inst(ipc_userspace_t *ipc_priv, uint32_t inst_id)
{
	int ret = ERR_SUCCESS;
	ENTER();

	os_het_control_t *sh_ctrl =  ipc_priv->sh_ctrl_area.vaddr;
	os_het_ipc_t *ipc = SH_CTRL_VADDR(sh_ctrl->ipc)
				+ sizeof(os_het_ipc_t)*inst_id;
	if (!ipc) {
		ret = -1;
		goto end;
	}
	if (ipc->num_ipc_channels > MAX_CHANNELS) {
		ret = -1;
		goto end;
	}
	ipc_priv->max_channels = ipc->num_ipc_channels;
	ipc_priv->max_depth = ipc->ipc_max_bd_size;
	ipc_priv->ipc_inst = ipc;
end:
	EXIT(ret);
	return ret;
}

int get_channels_info(ipc_userspace_t *ipc_priv, uint32_t inst_id)
{
	return get_ipc_inst(ipc_priv, inst_id);
}
/*
 * @get_channel_paddr
 *
 * Returns the phyical address of the channel data structure in the
 * share control area.
 *
 * Type: Internal function
 */
static phys_addr_t __get_channel_paddr(uint32_t channel_id,
		ipc_userspace_t *ipc_priv)
{
	phys_addr_t		phys_addr;

	ENTER();

	os_het_ipc_t *ipc = (os_het_ipc_t *)ipc_priv->ipc_inst;

	phys_addr = (phys_addr_t)ipc->ipc_channels +
		sizeof(os_het_ipc_channel_t)*channel_id;
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
static void *__get_channel_vaddr(uint32_t channel_id,
		ipc_userspace_t *ipc_priv)
{
	void *vaddr;
	ENTER();

	vaddr = SH_CTRL_VADDR(get_channel_paddr(channel_id, ipc_priv));

	EXIT(vaddr);
	return vaddr;
}
/*
 * @get_channel_paddr
 *
 * Returns the phyical address of the channel data structure in the
 * share control area.
 *
 * Type: Internal function
 */
static phys_addr_t get_channel_paddr(uint32_t channel_id,
		ipc_userspace_t *ipc_priv)
{
	return chvpaddr_arr[channel_id].phys_addr;
}

/*
 * @get_channel_vaddr
 *
 * Returns the virtual address of the channel data structure in the
 * share control area.
 *
 * Type: Internal function
 */
static void *get_channel_vaddr(uint32_t channel_id, ipc_userspace_t *ipc_priv)
{
	return chvpaddr_arr[channel_id].vaddr;
}
/*
 * @init_het_ipc
 *
 */
int init_het_ipc(ipc_userspace_t *ipc_priv)
{
	int ret = ERR_SUCCESS;
	ENTER();

	ipc_priv->dev_het_ipc = open("/dev/het_ipc", O_RDWR);
	if (ipc_priv->dev_het_ipc == -1) {
		debug_print("Error: Cannot open /dev/het_ipc. %d\n");
		ret = -ERR_DEV_HETIPC_FAIL;
	}

	EXIT(ret);
	return ret;
}
void fsl_ipc_us_reinit(fsl_ipc_t ipc)
{
	int i;
	ipc_userspace_t *ipc_priv = (ipc_userspace_t *)ipc;
	reload_print("Entering func %s.... ipc_us_t num_channels=%d\n"
		, __func__, ipc_priv->num_channels);


#ifdef CONFIG_LOCK
	for (i = 0; i < TOTAL_IPC_CHANNELS; i++)
		fsl_ipc_destroy_lock(ch_semid[i]);
#endif

	/* free memory */
	for (i = 0; i < ipc_priv->num_channels; i++)
		free(ipc_priv->channels[i]);

	ipc_priv->num_channels = 0;
	reload_print("Entering func %s.... ipc_us_t num_channels=%d\n"
		, __func__, ipc_priv->num_channels);
}
int fsl_ipc_reinit(fsl_ipc_t ipc)
{

	os_het_ipc_channel_t    *ipc_ch;
	ipc_userspace_t         *ipc_priv;
	int start_channel_id = 0, max_channel_id = MAX_IPC_CHANNELS;
	int i = 0;
	reload_print("Enter.... %s\n", __func__);
	ENTER();
	ipc_priv = (ipc_userspace_t *)ipc;


	/* os_het_ipc_channel_t size 60 */
	reload_print("dump_ipc_channel_ in %s\n", __func__);
	for (i = start_channel_id ; i < max_channel_id; i++) {

		ipc_ch = get_channel_vaddr(i, ipc_priv);

		ipc_ch->producer_initialized = 0;

		if (ipc_ch->id != 0)
			ipc_ch->consumer_initialized = 0;

		ipc_ch->tracker.consumer_num = 0;
		ipc_ch->tracker.producer_num = 0;

		if (ipc_ch->id != 0)
			ipc_ch->bd_ring_size = 0;

		ipc_ch->max_msg_size = 0;

		if (ipc_ch->id != 0)
			ipc_ch->ch_type = 0;
		else
			ipc_ch->ch_type = OS_HET_IPC_POINTER_CH;

		ipc_ch->ipc_ind = OS_HET_NO_INT;
		ipc_ch->ind_offset = 0;
		ipc_ch->ind_value = 0;
		ipc_ch->pa_reserved[0] = 0;
		ipc_ch->pa_reserved[1] = 0;
		ipc_ch->semaphore_pointer = NULL;
#if DEBUG_RELOAD
		dump_ipc_channel(ipc_ch);
#endif
}

	/* os_het_control_t structure size 48*/
	os_het_control_t *sh_ctrl =  ipc_priv->sh_ctrl_area.vaddr;
	sh_ctrl->initialized.pa_initialized = 0;
	sh_ctrl->initialized.sc_initialized = 0;

	/* PA and SC shared control start address and size not initailized
	memset(&sh_ctrl->pa_shared_mem, 0, sizeof(os_het_mem_t));
	memset(&sh_ctrl->sc_shared_mem, 0, sizeof(os_het_mem_t));
	*/

	/* free user space stucture */
	fsl_ipc_us_reinit(ipc);
	/* will never fail */
	return ERR_SUCCESS;
}
