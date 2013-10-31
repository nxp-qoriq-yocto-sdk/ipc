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
#include <pthread.h>
#include "logdefs.h"
#include "fsl_het_mgr.h"
#include "fsl_user_dma.h"
#include "fsl_ipc_um.h"
#include "fsl_usmmgr.h"
#include "dsp_boot.h"
#include "fsl_ipc_lock.h"
#include "fsl_ipc_kmod.h"
#include "fsl_bsc913x_ipc.h"
#include "fsl_ipc_errorcodes.h"
#include "fsl_heterogeneous.h"
#include "fsl_heterogeneous_ipc.h"
#include "lg_shm.h"
#include "fsl_L1_defense.h"

#define LOCAL_PRODUCER_NUM pa_reserved[0]
#define LOCAL_CONSUMER_NUM pa_reserved[1]

mem_range_t chvpaddr_arr[TOTAL_IPC_CHANNELS];
int ch_semid[TOTAL_IPC_CHANNELS];
extern shm_seg_t shm;
uint32_t g_core_mask;

/*********** Defines ******************/
#define MAX_MSG_SIZE 1020
#define PAGE_SIZE 4096
#define SH_CTRL_VADDR(A) \
	(void *)((unsigned long)(A) \
			- (ipc_priv->sh_ctrl_area.phys_addr) \
			+  ipc_priv->sh_ctrl_area.vaddr)
#define SH_MEM_IPC_VADDR(A) \
	(void *)((unsigned long)(A) \
			- shm.paddr \
			+ shm.vaddr)
#ifdef B913x
#define GCR_OFFSET 0x17000
#define IPC_CH_VADDR(A) SH_CTRL_VADDR(A)
#endif
#ifdef B4860
#define GCR_OFFSET 0x8F2000
#define IPC_CH_VADDR(A) SH_MEM_IPC_VADDR(A)
#endif

int init_het_ipc(ipc_userspace_t *ipc);
uint32_t ipc_get_free_rt_signal(void);
static void *get_channel_vaddr(uint32_t channel_id,
		ipc_userspace_t *ipc_priv);
static void *__get_channel_vaddr(uint32_t channel_id,
		ipc_userspace_t *ipc_priv);
static unsigned long get_channel_paddr(uint32_t channel_id,
		ipc_userspace_t *ipc_priv);
static unsigned long __get_channel_paddr(uint32_t channel_id,
		ipc_userspace_t *ipc_priv);
int get_ipc_vaddr(ipc_userspace_t *ipc_priv, int inst_id);
int get_ipc_inst(ipc_userspace_t *ipc_priv, uint32_t inst_id);
int get_channels_info(ipc_userspace_t *ipc, uint32_t rat_id);
void generate_indication(os_het_ipc_channel_t *ipc_ch,
		ipc_userspace_t *ipc_priv);
void signal_handler(int signo, siginfo_t *siginfo, void *data);
void dump_ipc_channel(os_het_ipc_channel_t *);

/***** Implementation ******************/
fsl_ipc_t fsl_ipc_init(ipc_p2v_t p2vcb, mem_range_t sh_ctrl_area,
		mem_range_t dsp_ccsr, mem_range_t pa_ccsr, char uiodevbuf[])
{
	uint32_t rat_id = 0;
	return fsl_ipc_init_rat(rat_id, p2vcb, sh_ctrl_area,
		dsp_ccsr, pa_ccsr, uiodevbuf);
}


fsl_ipc_t fsl_ipc_init_rat(uint32_t rat_id, ipc_p2v_t p2vcb,
	mem_range_t sh_ctrl_area, mem_range_t dsp_ccsr, mem_range_t pa_ccsr,
	char uiodevbuf[])
{
	int ret = ERR_SUCCESS;
	ipc_userspace_t *ipc_priv = NULL;
	int i, dev_het_mgr = 0, num_ipc_regions = 0;
	ENTER();

	if (!p2vcb) {
		ret = -ERR_P2V_NULL;
		goto end;
	}

	if (rat_id != 0) {
		dev_het_mgr = open("/dev/het_mgr", O_RDWR);
		if (dev_het_mgr == -1) {
			printf("Error: Cannot open /dev/het_mgr\n");
			printf("Frm %s\n", __func__);
			return  NULL;
		}

		ret = ioctl(dev_het_mgr,
			IOCTL_HET_MGR_GET_RAT_MODE,
			&num_ipc_regions);
		if (ret) {
			perror("IOCTL_HET_MGR_GET_RAT_MODE:");
			return NULL;
		} else if (rat_id >= num_ipc_regions) {
			printf("Error -ERR_INCORRECT_RAT_MODE\n");
			return NULL;
		}
	}

	/* Allocate memory for ipc instance */
	ipc_priv = malloc(sizeof(ipc_userspace_t));
	if (!ipc_priv)
		goto end;

	memset(ipc_priv, 0, sizeof(ipc_userspace_t));
	ipc_priv->p2vcb = p2vcb;

	memcpy(&ipc_priv->sh_ctrl_area, &sh_ctrl_area, sizeof(mem_range_t));
	memcpy(&ipc_priv->dsp_ccsr, &dsp_ccsr, sizeof(mem_range_t));
	memcpy(&ipc_priv->pa_ccsr, &pa_ccsr, sizeof(mem_range_t));

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

	memcpy(ipc_priv->uio_dev_buf, uiodevbuf, strlen(uiodevbuf));
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
int ipc_channel_attach_msg_ring(uint32_t channel_id,
		unsigned long msg_phys_addr, uint32_t msg_size,
		ipc_userspace_t *ipc_priv)
{
	int depth;
	int ret = ERR_SUCCESS;
	int i = 0;

	os_het_ipc_channel_t 	*ch;
	os_het_ipc_bd_t	*bd_base;
	os_het_ipc_bd_t	*bd;

	ch =  get_channel_vaddr(channel_id, ipc_priv);
	depth = ch->bd_ring_size;
	bd_base = IPC_CH_VADDR(ch->bd_base);

	for (i = 0; i < depth; i++) {
		bd = &bd_base[i];
		bd->msg_ptr = msg_phys_addr;
		msg_phys_addr += msg_size;
	}

	return ret;
}

int fsl_ipc_configure_channel(uint32_t channel_id, uint32_t depth,
		ipc_ch_type_t channel_type,
		unsigned long msg_ring_paddr, uint32_t msg_size,
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

int fsl_ipc_configure_txreq(uint32_t channel_id, unsigned long phys_addr,
		uint32_t max_txreq_lbuff_size, fsl_ipc_t ipc)
{
	int ret;
	int locked = 0;
	unsigned long phys_addr_s;
	mem_range_t dma_list_mem;
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
	phys_addr_s += 32*sizeof(unsigned long);

	if (phys_addr_s % 32)
		phys_addr_s += (32  - phys_addr_s % 32);

	dma_list_mem.phys_addr = phys_addr_s;

	dma_list_mem.vaddr = (*ipc_priv->p2vcb)(phys_addr_s);
	ipc_priv->udma = fsl_uspace_dma_init(dma_list_mem, ipc_priv->pa_ccsr
			, ipc_priv->rat_id, ipc_priv->uio_dev_buf);

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
	unsigned long			phys_addr;
	unsigned long			phys_addr2;
#ifdef B4860
	uint64_t			phys_addr3;
#endif
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
	bd_base = IPC_CH_VADDR(ipc_ch->bd_base);
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
	debug_print("copying %x to %x length %x\n", ((unsigned long)vaddr +
				MAX_TX_REQ_MSG_SIZE), phys_addr,
			sizeof(unsigned long));
	/* size is 4 since lbuff address is never greater than 4GB
	 * of MSG type channel*/
	memcpy((void *)((unsigned long)vaddr + MAX_TX_REQ_MSG_SIZE),
			&phys_addr, 4);
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

	debug_print("## Writing P=%lx V=%p #=%x\n",
			phys_addr, vaddr, incr1);
	/* Add producer increment */
	memcpy(vaddr, &incr1, sizeof(uint32_t));
	/* Get physical address of producer_num */
	phys_addr2 = get_channel_paddr(channel_id, ipc_priv);
	debug_print("TXREQ 0: %lx %x\n", phys_addr2, channel_id);
	phys_addr2 += (unsigned long)&ipc_ch->tracker.producer_num -
		(unsigned long)ipc_ch;
	debug_print("TXREQ: PIaddr=%lx val=%lx\n",
			phys_addr2, phys_addr);

	fsl_uspace_dma_add_entry(phys_addr, phys_addr2,
			4, ipc_priv->udma);

	/* Get physical address of LOCAL_PRODUCER_NUM */
	memcpy((void *)((unsigned long)vaddr + 4), &incr2, sizeof(incr2));
	debug_print("## Writing V=%lx P=%p #=%x\n",
			phys_addr + 4, vaddr + 4, incr2);

	phys_addr2 = get_channel_paddr(channel_id, ipc_priv);
	debug_print("TXREQ 0: %lx %x\n", phys_addr2, channel_id);

	phys_addr2 += (unsigned long)&ipc_ch->LOCAL_PRODUCER_NUM -
		(unsigned long)ipc_ch;
	debug_print("TXREQ: PILaddr=%lx val=%lx\n", phys_addr2, phys_addr);

	fsl_uspace_dma_add_entry(phys_addr + 4, phys_addr2,
			4, ipc_priv->udma);

	/* VIRQ geneartion */
	if (ipc_ch->ipc_ind == OS_HET_VIRTUAL_INT) {
		phys_addr2 = ipc_priv->dsp_ccsr.phys_addr + GCR_OFFSET +
			ipc_ch->ind_offset;
		memcpy(((void *)(unsigned long)vaddr + 8), &ipc_ch->ind_value,
				sizeof(ipc_ch->ind_value));
		debug_print("TXREQ: INDaddr=%lx val=%p %p\n",
				phys_addr2, (void *)(long)vaddr+8, vaddr);
		debug_print("TXREQ: ipc_ch->ind_value=%x size %x\n",
				ipc_ch->ind_value, sizeof(ipc_ch->ind_value));
#ifdef B913x
		fsl_uspace_dma_add_entry(phys_addr + 8, phys_addr2,
				4, ipc_priv->udma);
#endif
#ifdef B4860
		phys_addr3 = ipc_priv->dsp_ccsr.phys_addr + GCR_OFFSET +
			ipc_ch->ind_offset;
		fsl_uspace_dma_add_entry(phys_addr + 8, phys_addr3,
				4, ipc_priv->udma);
#endif
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
		ipc_ch->LOCAL_CONSUMER_NUM = (ipc_ch->LOCAL_CONSUMER_NUM + 1) %
			ipc_ch->bd_ring_size;
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
int fsl_ipc_send_ptr(uint32_t channel_id, unsigned long addr, uint32_t len,
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
	bd_base = IPC_CH_VADDR(ipc_ch->bd_base);

	/*OLD bd = &bd_base[ipc_ch->tracker.producer_num]; */
	bd = &bd_base[ipc_ch->LOCAL_PRODUCER_NUM];
	bd->msg_ptr = addr;
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

void dump_ipc_channel(os_het_ipc_channel_t *ipc_ch)
{
	printf("ipc_ch%lx PI=%x CI=%x ID=%x PN=%x CN=%x LPI=%x LCI=%x \
			BS=%x MX=%x CH=%x BD=%lx II=%x IO=%x IV=%x",
			(unsigned long)ipc_ch,
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
			(unsigned long)ipc_ch->bd_base,
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
	debug_print("\n num free bds = %d \n", os_het_ch_free_bds(ipc_ch));

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
	bd_base = IPC_CH_VADDR(ipc_ch->bd_base);


	/*OLD bd = &bd_base[ipc_ch->tracker.producer_num];*/
	bd = &bd_base[ipc_ch->LOCAL_PRODUCER_NUM];

	/* get the virtual address of the msg ring */
	vaddr =  (*ipc_priv->p2vcb)(bd->msg_ptr);
	if (!vaddr) {
		ret = -1;
		EXIT(ret);
		goto end;
	}

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

int fsl_ipc_recv_ptr(uint32_t channel_id, unsigned long *addr, uint32_t *len,
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
	bd_base = IPC_CH_VADDR(ipc_ch->bd_base);
	/*bd = &bd_base[ipc_ch->tracker.consumer_num]; */
	bd = &bd_base[ipc_ch->LOCAL_CONSUMER_NUM];

	*addr = (unsigned long)bd->msg_ptr;
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

int fsl_ipc_recv_ptr_hold(uint32_t channel_id, unsigned long *addr,
		uint32_t *len, fsl_ipc_t ipc)
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

	bd_base = IPC_CH_VADDR(ipc_ch->bd_base);
	/*bd = &bd_base[ipc_ch->tracker.consumer_num];*/
	bd = &bd_base[ipc_ch->LOCAL_CONSUMER_NUM];

	*addr = (unsigned long)bd->msg_ptr;
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

	bd_base = IPC_CH_VADDR(ipc_ch->bd_base);
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

	bd_base = IPC_CH_VADDR(ipc_ch->bd_base);
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
	os_het_ipc_t *ipc = IPC_CH_VADDR(sh_ctrl->ipc)
				+ sizeof(os_het_ipc_t)*inst_id;
	if (!ipc) {
		ret = -1;
		goto end;
	}
	if (ipc->num_ipc_channels > MAX_CHANNELS) {
		ret = -1;
		goto end;
	}

	/* ipc_channels is 64 bits but, area of hugetlb/DDR will always
	 * less than 4GB(B4),for 913x it is only 2GB, so the value is
	 * always in 32 bits, that is why bitwise and with 0xFFFFFFFF
	 */
	if ((ipc->ipc_channels & 0xFFFFFFFF) == 0) {
		ret = -ERR_INCORRECT_RAT_MODE;
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
static unsigned long __get_channel_paddr(uint32_t channel_id,
		ipc_userspace_t *ipc_priv)
{
	unsigned long		phys_addr;

	ENTER();

	os_het_ipc_t *ipc = (os_het_ipc_t *)ipc_priv->ipc_inst;

	phys_addr = (unsigned long)ipc->ipc_channels +
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

	vaddr = IPC_CH_VADDR(get_channel_paddr(channel_id, ipc_priv));

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
static unsigned long get_channel_paddr(uint32_t channel_id,
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
#ifdef B913x
		ipc_ch->semaphore_pointer = NULL;
#endif
#ifdef B4860
		ipc_ch->semaphore_pointer = 0;
#endif
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

/*
 * Open channel zero and initialize it to Pointer type
 * The Star Core polls on channel zero
 * to check whether the IPC is up and runing
 */
void open_channel_zero(os_het_ipc_channel_t *ch, uint16_t channel_depth)
{
	ch[0].consumer_initialized = OS_HET_INITIALIZED;
	ch[0].id = 0;
	ch[0].bd_ring_size = channel_depth;
	ch[0].ch_type = OS_HET_IPC_POINTER_CH;
	return;
}
#ifdef B4860
int fsl_B4_ipc_init(void *dsp_bt)
{

	debug_print("Enter func %s\n", __func__);
	uint32_t num_ipc_regions = 0;
	uint32_t ipc_param_muxed;
	uint32_t ipc_ch_start_paddr = 0, ipc_het_t_start_paddr = 0;
	uint16_t max_num_ipc_channels = MAX_IPC_CHANNELS;
	uint16_t max_channel_depth = DEFAULT_CHANNEL_DEPTH;
	void *vaddr;
	int dev_mem = ((dsp_bt_t *)dsp_bt)->dev_mem;
	int dev_het_mgr = ((dsp_bt_t *)dsp_bt)->het_mgr;
	mem_strt_addr_t sh_ctrl_area =
		((dsp_bt_t *)dsp_bt)->het_sys_map.sh_ctrl_area;
	ipc_ch_start_paddr = shm.paddr + IPC_METADATA_AREA_PADDR_OFFSET +
		IPC_HET_T_SZ_1K;
	debug_print("ipc_ch_start_paddr = %x\n", ipc_ch_start_paddr);
	ipc_het_t_start_paddr = shm.paddr + IPC_METADATA_AREA_PADDR_OFFSET;
	/* From module params */
	uint16_t num_channels, channel_depth;

	/* RAT mode*/
	uint8_t rat_inst = 0;

	int ret = 0;
	int i = 0, size = 0, j = 0;
	uint32_t phys_addr1;


	/*IPC */
	os_het_ipc_channel_t *ch;
	os_het_ipc_t *ipc;
	os_het_control_t *ctrl;

	/* Create os_het_ipc_t
	 * sh_ctrl_area is local here
	 */

	vaddr = mmap(0, sh_ctrl_area.size, (PROT_READ | \
			PROT_WRITE), MAP_SHARED, dev_mem, \
			sh_ctrl_area.phys_addr);     \
	if (vaddr == MAP_FAILED) {
		perror("MAP failed:");
		return -1;
	}
	/*os_het_control_t struct*/
	ctrl = vaddr;
	ctrl->ipc = ipc_het_t_start_paddr;
	ipc = (os_het_ipc_t *)(shm.vaddr + (uint32_t)ctrl->ipc - shm.paddr);

	/* Get RAT MODE */
	ret = ioctl(dev_het_mgr,
		     IOCTL_HET_MGR_GET_RAT_MODE,
		     &num_ipc_regions);
	if (ret) {
		perror("IOCTL_HET_MGR_GET_RAT_MODE:");
		return -1;
	} else if (num_ipc_regions == DEFAULT_RAT_INST)
		rat_inst = 1;

	/* Get IPC_PARAMS */
	ret = ioctl(dev_het_mgr,
		     IOCTL_HET_MGR_GET_IPC_PARAMS,
		     &ipc_param_muxed);
	if (ret) {
		perror("IOCTL_HET_MGR_GET_IPC_PARAMS:");
		return -1;
	} else {
		max_num_ipc_channels = ipc_param_muxed & 0x0000FFFF;
		max_channel_depth = ((ipc_param_muxed & 0xFFFF0000) >> 16);
	}

	num_channels = max_num_ipc_channels;
	if (num_channels <= 0) {
		num_channels = MAX_IPC_CHANNELS;
		printf("warning: max_num_ipc_channels not set"
			" properly,setting default value = %d\n",
			num_channels);
	}

	channel_depth = max_channel_depth;
	if (channel_depth <= 0) {
		channel_depth = DEFAULT_CHANNEL_DEPTH;
		printf("warning: max_channel_depth not set"
			"setting default value = %d\n",
			channel_depth);
	}

	if (rat_inst == 0) {

		ipc->ipc_channels = ipc_ch_start_paddr;
		ch = (os_het_ipc_channel_t *)
			(shm.vaddr + ipc_ch_start_paddr - shm.paddr);

		ipc->num_ipc_channels = num_channels;
		ipc->ipc_max_bd_size = channel_depth;
		ipc->start_validation_value = HET_START_VALID_VALUE;
		ipc->end_validation_value = HET_END_VALID_VALUE;

		phys_addr1 = ipc_ch_start_paddr +
			sizeof(os_het_ipc_channel_t)*num_channels;

		/*
		In a loop of num_channels, set the ptr of channel structures
		in ipc->channels
		*/
		for (i = 0; i < num_channels; i++) {
			ch[i].start_validation_value = HET_START_VALID_VALUE;
			ch[i].end_validation_value = HET_END_VALID_VALUE;
			ch[i].ipc_ind = OS_HET_NO_INT;
			ch[i].id = i;
			ch[i].bd_base = phys_addr1;
			phys_addr1 += sizeof(os_het_ipc_bd_t)*channel_depth;
		}

		open_channel_zero(ch, channel_depth);

	} else if (rat_inst == 1) {

		/*calculate size of ipc channels*/
		size = sizeof(os_het_ipc_channel_t)*num_channels +
		/* array to hold channel pointers */
			sizeof(os_het_ipc_bd_t)*num_channels*channel_depth;
		/* ptr channel ring buffer */


		for (j = 0; j <= rat_inst; j++) {

			ipc = (void *)(shm.vaddr + (uint32_t)ctrl->ipc -
				shm.paddr + j*sizeof(os_het_ipc_t));

			ipc_ch_start_paddr += j * size;
			ipc->ipc_channels = ipc_ch_start_paddr;
			ch = (os_het_ipc_channel_t *)
				(shm.vaddr + ipc_ch_start_paddr - shm.paddr);

			ipc->num_ipc_channels = num_channels;
			ipc->ipc_max_bd_size = channel_depth;
			ipc->start_validation_value = HET_START_VALID_VALUE;
			ipc->end_validation_value = HET_END_VALID_VALUE;

			phys_addr1 = ipc_ch_start_paddr +
				sizeof(os_het_ipc_channel_t)*num_channels;

			/*
			In a loop of num_channels, set the ptr of channel
			structures in ipc->channels
			*/

			for (i = 0; i < num_channels; i++) {
				ch[i].start_validation_value =
					HET_START_VALID_VALUE;
				ch[i].end_validation_value =
					HET_END_VALID_VALUE;
				ch[i].ipc_ind = OS_HET_NO_INT;
				ch[i].id = i;
				ch[i].bd_base = phys_addr1;
				phys_addr1 +=
					sizeof(os_het_ipc_bd_t)*channel_depth;
			}


			open_channel_zero(ch, channel_depth);
		}
	}

	return ERR_SUCCESS;
}
#endif

int fsl_913x_ipc_init(void *dsp_bt)
{

	debug_print("Enter func %s\n", __func__);
	uint32_t num_ipc_regions = 0;
	uint32_t ipc_param_muxed;
	uint32_t ipc_ch_start_paddr = 0;
	uint16_t max_num_ipc_channels = MAX_IPC_CHANNELS;
	uint16_t max_channel_depth = DEFAULT_CHANNEL_DEPTH;
	void *vaddr;
	int dev_mem = ((dsp_bt_t *)dsp_bt)->dev_mem;
	int dev_het_mgr = ((dsp_bt_t *)dsp_bt)->het_mgr;
	mem_strt_addr_t sh_ctrl_area =
		((dsp_bt_t *)dsp_bt)->het_sys_map.sh_ctrl_area;

	/* From module params */
	uint16_t num_channels, channel_depth;

	/* RAT mode*/
	uint8_t rat_inst = 0;

	int ret = 0;
	int i = 0, size = 0, j = 0;
	uint32_t phys_addr1;


	/*IPC */
	os_het_ipc_channel_t *ch;
	os_het_ipc_t *ipc;
	os_het_control_t *ctrl;

	/* Create os_het_ipc_t
	 * sh_ctrl_area is local here
	 */

	vaddr = mmap(0, sh_ctrl_area.size, (PROT_READ | \
			PROT_WRITE), MAP_SHARED, dev_mem, \
			sh_ctrl_area.phys_addr);     \
	if (vaddr == MAP_FAILED) {
		perror("MAP failed:");
		return -1;
	}
	/*os_het_control_t struct*/
	ctrl = vaddr;
	/*error prone check again*/
	ipc = vaddr + ctrl->ipc - sh_ctrl_area.phys_addr;
	ipc_ch_start_paddr = (ctrl->ipc + sizeof(os_het_ipc_t));

	/* Get RAT MODE */
	ret = ioctl(dev_het_mgr,
		IOCTL_HET_MGR_GET_RAT_MODE,
		&num_ipc_regions);
	if (ret) {
		perror("IOCTL_HET_MGR_GET_RAT_MODE:");
		return -1;
	} else if (num_ipc_regions == DEFAULT_RAT_INST)
		rat_inst = 1;


	/* Get IPC_PARAMS */
	ret = ioctl(dev_het_mgr,
		IOCTL_HET_MGR_GET_IPC_PARAMS,
		&ipc_param_muxed);
	if (ret) {
		perror("IOCTL_HET_MGR_GET_IPC_PARAMS:");
		return -1;
	} else {
		max_num_ipc_channels = ipc_param_muxed & 0x0000FFFF;
		max_channel_depth = ((ipc_param_muxed & 0xFFFF0000) >> 16);
	}

	num_channels = max_num_ipc_channels;
	if (num_channels <= 0) {
		num_channels = MAX_IPC_CHANNELS;
		printf("warning: max_num_ipc_channels not set"
			" properly,setting default value = %d\n",
			num_channels);
	}

	channel_depth = max_channel_depth;
	if (channel_depth <= 0) {
		channel_depth = DEFAULT_CHANNEL_DEPTH;
		printf("warning: max_channel_depth not set"
			"setting default value = %d\n",
			channel_depth);
	}


	if (rat_inst == 0) {

		memcpy(&ipc->ipc_channels, &ipc_ch_start_paddr,
				sizeof(uint32_t));
		ch = (os_het_ipc_channel_t *)
			(vaddr + ipc_ch_start_paddr - sh_ctrl_area.phys_addr);
		ipc->num_ipc_channels = num_channels;
		ipc->ipc_max_bd_size = channel_depth;

		phys_addr1 = ipc_ch_start_paddr +
			sizeof(os_het_ipc_channel_t)*num_channels;

		/*
		In a loop of num_channels, set the ptr of channel structures
		in ipc->channels
		*/
		for (i = 0; i < num_channels; i++) {
			ch[i].ipc_ind = OS_HET_NO_INT;
			ch[i].id = i;
			memcpy(&ch[i].bd_base, &phys_addr1,
					sizeof(uint32_t));
			phys_addr1 += sizeof(os_het_ipc_bd_t)*channel_depth;
		}

		open_channel_zero(ch, channel_depth);

	} else if (rat_inst == 1) {

		/*calculate size of ipc channels*/
		size = sizeof(os_het_ipc_channel_t)*num_channels +
		/* array to hold channel pointers */
			sizeof(os_het_ipc_bd_t)*num_channels*channel_depth;
		/* ptr channel ring buffer */


		for (j = 0; j <= rat_inst; j++) {

			ipc = vaddr + ctrl->ipc - sh_ctrl_area.phys_addr +
				j*sizeof(os_het_ipc_t);

			ipc_ch_start_paddr += j *
				(size + sizeof(os_het_ipc_t));
			memcpy(&ipc->ipc_channels, &ipc_ch_start_paddr,
					sizeof(uint32_t));
			ch = (os_het_ipc_channel_t *)
				(vaddr + ipc_ch_start_paddr -
				 sh_ctrl_area.phys_addr);

			ipc->num_ipc_channels = num_channels;
			ipc->ipc_max_bd_size = channel_depth;

			phys_addr1 = ipc_ch_start_paddr +
				sizeof(os_het_ipc_channel_t)*num_channels;

			/*
			In a loop of num_channels, set the ptr of channel
			structures in ipc->channels
			*/

			for (i = 0; i < num_channels; i++) {
				ch[i].ipc_ind = OS_HET_NO_INT;
				ch[i].id = i;
				memcpy(&ch[i].bd_base, &phys_addr1,
						sizeof(uint32_t));
				phys_addr1 +=
					sizeof(os_het_ipc_bd_t)*channel_depth;
			}


			open_channel_zero(ch, channel_depth);
		}
	}

	return ERR_SUCCESS;
}

#ifdef B4860
int fsl_B4_ipc_reinit(fsl_ipc_t ipc, void *dsp_bt)
{
	/* Re create the IPC channels*/
	l1d_printf("enter func %s\n", __func__);
	if (ipc) {
		if (fsl_B4_ipc_init(dsp_bt)) {
			printf("Error in fsl_B4_ipc_init from %s\n", __func__);
			return -1;
		}
	}

	/* free user space stucture */
	if (ipc)
		fsl_ipc_us_reinit(ipc);
	return ERR_SUCCESS;
}

int check_validation_fields(uint32_t *sh_ctrl, void *dsp_bt)
{

	l1d_printf("Enter func %s\n", __func__);
	uint32_t num_ipc_regions = 0;
	uint32_t ipc_param_muxed;
	uint32_t ipc_ch_start_paddr = 0;
	uint16_t max_num_ipc_channels = MAX_IPC_CHANNELS;
	uint16_t max_channel_depth = DEFAULT_CHANNEL_DEPTH;
	int dev_het_mgr = ((dsp_bt_t *)dsp_bt)->het_mgr;
	ipc_ch_start_paddr = shm.paddr + IPC_METADATA_AREA_PADDR_OFFSET +
		IPC_HET_T_SZ_1K;
	l1d_printf("ipc_ch_start_paddr = %x\n", ipc_ch_start_paddr);
	/* From module params */
	uint16_t num_channels, channel_depth;

	/* RAT mode*/
	uint8_t rat_inst = 0;

	int ret = 0, k = 0;
	int i = 0, size = 0, j = 0;

	/*IPC */
	os_het_ipc_channel_t *ch;
	os_het_ipc_t *ipc;
	os_het_control_t *ctrl = (os_het_control_t *)sh_ctrl;

	/* Create os_het_ipc_t
	 * sh_ctrl_area is local here
	 */

	ipc = (os_het_ipc_t *)(shm.vaddr + (uint32_t)ctrl->ipc - shm.paddr);
	/* Error prone code check here*/
	uint32_t phys_addr_sh_ctrl =
		((dsp_bt_t *)dsp_bt)->het_sys_map.sh_ctrl_area.phys_addr;
	os_het_l1d_t *l1_defense = (os_het_l1d_t *)(
		ctrl->l1d + ctrl - phys_addr_sh_ctrl);
	os_het_smartdsp_log_t *smartdsp_debug = (os_het_smartdsp_log_t *)(
		ctrl->smartdsp_debug + ctrl - phys_addr_sh_ctrl);


	/* Get RAT MODE */
	ret = ioctl(dev_het_mgr,
		     IOCTL_HET_MGR_GET_RAT_MODE,
		     &num_ipc_regions);
	if (ret) {
		perror("IOCTL_HET_MGR_GET_RAT_MODE:");
		return -1;
	} else if (num_ipc_regions == DEFAULT_RAT_INST)
		rat_inst = 1;

	/* Get IPC_PARAMS */
	ret = ioctl(dev_het_mgr,
		     IOCTL_HET_MGR_GET_IPC_PARAMS,
		     &ipc_param_muxed);
	if (ret) {
		perror("IOCTL_HET_MGR_GET_IPC_PARAMS:");
		return -1;
	} else {
		max_num_ipc_channels = ipc_param_muxed & 0x0000FFFF;
		max_channel_depth = ((ipc_param_muxed & 0xFFFF0000) >> 16);
	}

	num_channels = max_num_ipc_channels;
	if (num_channels <= 0) {
		printf("ipc channel was found to be = %d\n",
			num_channels);
		return -1;
	}

	channel_depth = max_channel_depth;
	if (channel_depth <= 0) {
		printf("ipc channel depth value found to be = %d\n",
			channel_depth);
		return -1;
	}

	if (rat_inst == 0) {
		if (ipc->start_validation_value != HET_START_VALID_VALUE ||
		    ipc->end_validation_value != HET_END_VALID_VALUE)
			return OS_HET_ERR_L1D_MEMORY_CORRUPTED;

		ch = (os_het_ipc_channel_t *)
			(shm.vaddr + ipc_ch_start_paddr - shm.paddr);

		for (i = 0; i < num_channels; i++) {
			if (ch[i].start_validation_value !=
			HET_START_VALID_VALUE ||
			ch[i].end_validation_value !=
			HET_END_VALID_VALUE)
				return OS_HET_ERR_L1D_MEMORY_CORRUPTED;

		}

	} else if (rat_inst == 1) {

		/*calculate size of ipc channels*/
		size = sizeof(os_het_ipc_channel_t)*num_channels +
		/* array to hold channel pointers */
			sizeof(os_het_ipc_bd_t)*num_channels*channel_depth;
		/* ptr channel ring buffer */


		for (j = 0; j <= rat_inst; j++) {

			ipc = (void *)(shm.vaddr + (uint32_t)ctrl->ipc -
				shm.paddr + j*sizeof(os_het_ipc_t));

			ipc_ch_start_paddr += j * size;
			if (ipc->start_validation_value !=
			HET_START_VALID_VALUE ||
			ipc->end_validation_value !=
			HET_END_VALID_VALUE)
				return OS_HET_ERR_L1D_MEMORY_CORRUPTED;

			ch = (os_het_ipc_channel_t *)
				(shm.vaddr + ipc_ch_start_paddr - shm.paddr);


			/*
			In a loop of num_channels, set the ptr of channel
			structures in ipc->channels
			*/

			for (i = 0; i < num_channels; i++) {
				if (ch[i].start_validation_value !=
				    HET_START_VALID_VALUE ||
				    ch[i].end_validation_value !=
				    HET_END_VALID_VALUE)
					return OS_HET_ERR_L1D_MEMORY_CORRUPTED;

			}

		}
	}


	if (l1_defense->start_validation_value != HET_START_VALID_VALUE ||
	    l1_defense->end_validation_value != HET_END_VALID_VALUE)
			return OS_HET_ERR_L1D_MEMORY_CORRUPTED;
	else if (ctrl->start_validation_value != HET_START_VALID_VALUE ||
		 ctrl->end_validation_value != HET_END_VALID_VALUE)
			return OS_HET_ERR_L1D_MEMORY_CORRUPTED;
	else {
		for (k = 0; k <= 5; k++) {
			if (smartdsp_debug[i].start_validation_value !=
			    HET_START_VALID_VALUE ||
			    smartdsp_debug[i].end_validation_value !=
			    HET_END_VALID_VALUE)
				return OS_HET_ERR_L1D_MEMORY_CORRUPTED;
			}
		}

	return ERR_SUCCESS;
}

void create_ioctl_thread(void *thread_arg)
{
	l1d_printf("Enter func %s\n", __func__);
	fsl_defense_cb cb = thread_arg;
	uint32_t ret = 0;
	int dev_fsl_l1d = open("/dev/fsl_l1d", O_RDWR);

	if (dev_fsl_l1d == -1) {
		printf("Error: Cannot open /dev/fsl_l1d\n");
		printf("Frm %s\n", __func__);
		return;
	}

	l1d_printf("here %s &g_core_mask %p\n", __func__, &g_core_mask);

	while (1) {

		ret = ioctl(dev_fsl_l1d,
			IOCTL_FSL_L1D_GET_WSRSR_BITMASK,
			&g_core_mask);
		l1d_printf("after ioctl %s g_core_mask %#x\n", __func__,
				g_core_mask);

		if (ret < 0) {
			perror("IOCTL_FSL_L1D_GET_WSRSR_BITMASK");
			return;
		} else if (g_core_mask != 0) {
			/*Call the call back handler here */
			l1d_printf("Callback function reached\n");
			(*cb)(g_core_mask);
			g_core_mask = 0;
		}
	}
}

void fsl_L1_defense_register_cb(fsl_defense_cb cb)
{
	l1d_printf("Enter func %s\n", __func__);
	pthread_t thread1;
	pthread_create(&thread1, NULL, (void *)&create_ioctl_thread,
			(void *)cb);
	return;
}

#endif
