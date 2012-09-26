/*
 * Copyright 2011-2012 Freescale Semiconductor, Inc.
 *
 * Author: Manish Jaggi <manish.jaggi@freescale.com>
 */
#include <stdint.h>
#include <stdio.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/shm.h>
#include <sys/ioctl.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <signal.h>
#include "logdefs.h"
#include "fsl_types.h"
#include "fsl_het_mgr.h"
#include "fsl_usmmgr.h"
#include "fsl_ipc_shm.h"
#include "logdefs.h"
#include "fsl_ipc_errorcodes.h"
#include "bsc913x_heterogeneous.h"
#include "fsl_ipc_lock.h"

#define BSC9132_MODEL_STR	"fsl,bsc9132"

#define SMAP(I, A, B, C) do {	\
			priv->map[I].phys_addr = A;   \
			priv->map[I].vaddr = B; \
			priv->map[I].size = C;\
			} while (0);

#define MMAP(P, SZ)	do {	\
			vaddr = mmap(0, SZ, (PROT_READ | \
			PROT_WRITE), MAP_SHARED, priv->dev_mem, \
			P);     \
			SMAP(priv->mapidx, P, vaddr, SZ);\
			priv->mapidx++;\
			} while (0);

#define TLB1_SH_MEM_SIZE	0x1000000
#define MAX_MAP_NUM		16
#define STATUS_FAIL		-1
#define STATUS_FILE_EMPTY	-2
#define FILE_NOT_FOUND		-3
#define MMAP_FAILED		-4
#define MEM_DUMP_CFG_FILE	"mem_dump_cfg.txt"
#define PAGE_SIZE 		0x1000

typedef struct {
	int mapidx;
	int dev_het_mgr;
	int dev_mem;
	int shmid;

	range_t	map[MAX_MAP_NUM];
	sys_map_t het_sys_map;
	range_t	shared_area;
	range_t dsp_ccsr;
	range_t pa_ccsr;
} usmmgr_priv;

void cleanup(usmmgr_priv *priv);
/* */

int fsl_usmmgr_alloc(range_t *r, fsl_usmmgr_t usmmgr)
{
	if (!r->size)
		return -1;

	r->vaddr = shm_alloc(r->size);
	if (!r->vaddr)
		return -1;

	r->phys_addr = (phys_addr_t)shm_vtop(r->vaddr);

	return 0;
}

int fsl_usmmgr_memalign(range_t *r, unsigned long align, fsl_usmmgr_t usmmgr)
{
	if (!r->size)
		return -1;

	r->vaddr = shm_memalign(r->size, align);
	if (!r->vaddr)
		return -1;

	r->phys_addr = (phys_addr_t)shm_vtop(r->vaddr);

	return 0;
}

void fsl_usmmgr_free(range_t *r, fsl_usmmgr_t usmmgr)
{
	shm_free(r->vaddr);
}

void cleanup(usmmgr_priv *priv)
{
	if (priv->dev_het_mgr != -1)
		close(priv->dev_het_mgr);

	if (priv->dev_mem != -1)
		close(priv->dev_mem);

}

int init_het_mgr(usmmgr_priv *priv)
{
	int ret = ERR_SUCCESS;
	ENTER();
	priv->dev_het_mgr = open("/dev/het_mgr", O_RDWR);
	if (priv->dev_het_mgr == -1) {
		debug_print("Error: Cannot open /dev/het_mgr. %d\n");
		ret = -ERR_DEV_HETMGR_FAIL;
	}
	EXIT(ret);
	return ret;
}

int init_dev_mem(usmmgr_priv *priv)
{
	int ret = ERR_SUCCESS;
	ENTER();
	priv->dev_mem = open("/dev/mem", O_RDWR);
	if (priv->dev_mem == -1) {
		debug_print("Error: Cannot open /dev/mem.\n");
		ret = -ERR_DEV_MEM_FAIL;
	}
	EXIT(ret);
	return ret;
}

int map_shared_mem(usmmgr_priv *priv)
{
	int ret = ERR_SUCCESS;
	void *vaddr;
	FILE *fp;
	char model_str[16] = {0};
	int bytes =  strlen(BSC9132_MODEL_STR);
	ENTER();
	/* open /proc/device-tree/model to find
	basc9132/bsc9131
	*/
	fp = fopen("/proc/device-tree/model", "rb");
	if (!fp) {
		printf("Unable to open /proc/device-tree/model\n");
		ret = -1;
		goto end;
	}

	ret = fread(model_str, bytes, 1, fp);
	fclose(fp);

	if (!ret)
		goto end;

	/* open /dev/mem
	 * map dsp m2/m3/ddr
	 */
	/* Send IOCTL to get system map */
	ret = ioctl(priv->dev_het_mgr,
		IOCTL_HET_MGR_GET_SYS_MAP, &priv->het_sys_map);
	if (ret)
		goto end;

	if (priv->het_sys_map.smart_dsp_os_priv_area.phys_addr == 0xffffffff ||
		priv->het_sys_map.dsp_core0_m2.phys_addr == 0xffffffff ||
		priv->het_sys_map.dsp_core1_m2.phys_addr == 0xffffffff ||
		priv->het_sys_map.dsp_m3.phys_addr == 0xffffffff ||
		priv->het_sys_map.pa_shared_size == 0xffffffff ||
		priv->het_sys_map.dsp_shared_size == 0xffffffff) {
			printf("Incorrect Het Sys params\n");
			return -1;
	}

	/* MAP DSP private area in ddr */
	MMAP(priv->het_sys_map.dsp_core0_m2.phys_addr,
		priv->het_sys_map.dsp_core0_m2.size);
	if (vaddr == MAP_FAILED)
		return -1;

	if (!memcmp(model_str, BSC9132_MODEL_STR, bytes)) {
		MMAP(priv->het_sys_map.dsp_core1_m2.phys_addr,
			priv->het_sys_map.dsp_core1_m2.size);
		if (vaddr == MAP_FAILED)
			return -1;

		MMAP(priv->het_sys_map.dsp_m3.phys_addr,
			priv->het_sys_map.dsp_m3.size);
		if (vaddr == MAP_FAILED)
			return -1;
	}
end:
	EXIT(ret);
	return ret;
}

static int fsl_usmmgr_sem_lock()
{
	int rc = 0;
	int semid;

	semid = fsl_ipc_sem_init(getpid());
	if (semid < 0) {
		printf("%s: Error in initializing semaphore\n", __func__);
		rc = -1;
	} else
		rc = fsl_ipc_sem_lock(semid);

	return rc;
}

static int fsl_usmmgr_sem_unlock()
{
	int rc = 0;
	int semid;

	semid = fsl_ipc_sem_init(getpid());
	if (semid < 0) {
		printf("%s: Error in initializing semaphore\n", __func__);
		rc = -1;
	} else
		rc = fsl_ipc_sem_unlock(semid);

	return rc;
}

static int fsl_usmmgr_sem_destroy()
{
	key_t key = getpid();
	int semid;
	int rc = 0;

	semid = semget(key, 1, 0);
	if (semid == -1) {
		perror("Unable to obtain semid for"
					" fsl_usmmgr semaphore\r\n");
		rc = -1;
		goto out;
	}

	rc = fsl_ipc_sem_destroy(semid);
	if (rc < 0)
		printf("%s: Unable to destroy semid %d semaphore\r\n",
			__func__, semid);
out:
	return rc;
}

fsl_usmmgr_t fsl_usmmgr_init(void)
{
	int ret = ERR_SUCCESS;
	void *ptr_ret;
	static uint8_t usmmgr_initialized;
	static usmmgr_priv *priv;

	ENTER();

	ret = fsl_usmmgr_sem_lock();
	if (ret)
		return NULL;

	if (usmmgr_initialized) {
		fsl_usmmgr_sem_unlock();
		return priv;
	}

	priv = malloc(sizeof(usmmgr_priv));
	if (!priv)
		goto end;

	priv->mapidx = 0;
	priv->dev_het_mgr = 0;
	priv->dev_mem = 0;

	memset(&priv->shared_area, 0, sizeof(range_t));
	memset(&priv->dsp_ccsr, 0, sizeof(range_t));
	memset(&priv->pa_ccsr, 0, sizeof(range_t));
	memset(priv->map, 0, MAX_MAP_NUM*sizeof(range_t));
	memset(&priv->het_sys_map, 0, sizeof(sys_map_t));

	ptr_ret = fsl_shm_init(0);
	if (!ptr_ret)
		goto end;

	ret = init_het_mgr(priv);
	if (ret)
		goto end;

	ret = init_dev_mem(priv);
	if (ret)
		goto end;

	ret = map_shared_mem(priv);
	if (ret)
		goto end;

	usmmgr_initialized = 1;
end:
	if (ret) {
		cleanup(priv);
		free(priv);
		priv = NULL;
	}

	fsl_usmmgr_sem_unlock();
	EXIT(ret);
	return priv;
}

int fsl_usmmgr_exit(fsl_usmmgr_t usmmgr)
{
	int rc = 0;

	rc = fsl_usmmgr_sem_destroy();

	return rc;
}

int get_shared_ctrl_area(range_t *r, fsl_usmmgr_t usmmgr)
{
	int ret = ERR_SUCCESS;
	ENTER();

	usmmgr_priv *priv = (usmmgr_priv *)usmmgr;

	if (!priv->shared_area.vaddr) {

		r->phys_addr = priv->het_sys_map.sh_ctrl_area.phys_addr;
		r->size = priv->het_sys_map.sh_ctrl_area.size;
		r->vaddr = mmap(0, r->size, (PROT_READ |
			PROT_WRITE), MAP_SHARED, priv->dev_mem,
				r->phys_addr);

		if (r->vaddr == MAP_FAILED) {
			EXIT(-1);
			return -1;
		}

		memcpy(&priv->shared_area, r, sizeof(range_t));
		DUMPR(&priv->shared_area);
	} else
		memcpy(r, &priv->shared_area, sizeof(range_t));

	DUMPR(r);
	EXIT(ret);
	return ret;
}

int get_dsp_ccsr_area(range_t *r, fsl_usmmgr_t usmmgr)
{
	int ret = ERR_SUCCESS;
	ENTER();

	usmmgr_priv *priv = (usmmgr_priv *)usmmgr;

	if (!priv->dsp_ccsr.vaddr) {

		r->phys_addr = priv->het_sys_map.dsp_ccsrbar.phys_addr;
		r->size = priv->het_sys_map.dsp_ccsrbar.size;
		r->vaddr = mmap(0, r->size, (PROT_READ |
			PROT_WRITE), MAP_SHARED, priv->dev_mem,
				r->phys_addr);

		if (r->vaddr == MAP_FAILED) {
			EXIT(-1);
			return -1;
		}

		memcpy(&priv->dsp_ccsr, r, sizeof(range_t));
	} else
		memcpy(r, &priv->dsp_ccsr, sizeof(range_t));

	EXIT(ret);
	return ret;
}

int get_pa_ccsr_area(range_t *r, fsl_usmmgr_t usmmgr)
{
	int ret = ERR_SUCCESS;
	ENTER();

	usmmgr_priv *priv = (usmmgr_priv *)usmmgr;

	if (!priv->pa_ccsr.vaddr) {

		r->phys_addr = priv->het_sys_map.pa_ccsrbar.phys_addr;
		r->size = priv->het_sys_map.pa_ccsrbar.size;
		r->vaddr = mmap(0, r->size, (PROT_READ |
			PROT_WRITE), MAP_SHARED, priv->dev_mem,
				r->phys_addr);

		if (r->vaddr == MAP_FAILED) {
			EXIT(-1);
			return -1;
		}

		memcpy(&priv->pa_ccsr, r, sizeof(range_t));
	} else
		memcpy(r, &priv->pa_ccsr, sizeof(range_t));

	EXIT(ret);
	return ret;
}

phys_addr_t fsl_usmmgr_v2p(void *vaddr, fsl_usmmgr_t usmmgr)
{
	phys_addr_t paddr;

	paddr = (phys_addr_t)shm_vtop(vaddr);

	return paddr;
}

void *fsl_usmmgr_p2v(phys_addr_t phys_addr, fsl_usmmgr_t usmmgr)
{
	int i;
	void *vaddr = NULL;
	ENTER();
	debug_print("%x \n", phys_addr);
	usmmgr_priv *priv = (usmmgr_priv *)usmmgr;

	vaddr = shm_ptov((void *) phys_addr);
	if (vaddr)
		goto end;

	for (i = 0; i < priv->mapidx; i++)
		if (phys_addr >= priv->map[i].phys_addr &&
			phys_addr < priv->map[i].phys_addr + priv->map[i].size)
			vaddr = (void *)(phys_addr - priv->map[i].phys_addr) +
				(uint32_t)priv->map[i].vaddr;
end:
	EXIT(vaddr);
	return vaddr;
}

static int get_phy_addr_from_file(unsigned long *buf_in)
{

	FILE *file_d;
	int i = 0, num_of_entry_in_buf_in = 0;
	file_d = fopen(MEM_DUMP_CFG_FILE, "r");
	if (file_d == NULL) {
		perror(MEM_DUMP_CFG_FILE);
		return FILE_NOT_FOUND;
	}

	while (!feof(file_d)) {

		if (EOF != fscanf(file_d, "%lx", &buf_in[i])) {
			i++;
		} else {
			num_of_entry_in_buf_in = i/2;
			break;
		}

	}

	fclose(file_d);
	return num_of_entry_in_buf_in;
}

int fsl_usmmgr_dump_memory(void *mem_dump_buf, size_t size)
{

	int fd, i = 0;
	void *base;
	unsigned long calculated_total_size = 0, mmap_size = 0, pad_len = 0;
	unsigned int size_left = size;
	unsigned long current_addr, buf_in[500], mmap_addr_aligned;
	int count, status;
	int destination_buf_full_flag = 0;

	status = get_phy_addr_from_file(buf_in);

	if (0 == status) {
		printf("%s was empty\n", MEM_DUMP_CFG_FILE);
		return STATUS_FILE_EMPTY;
	} else if (FILE_NOT_FOUND == status)
		return FILE_NOT_FOUND;
	else
		count = status;

	fd = open("/dev/mem", O_RDWR);
	if (fd < 0) {
		printf("Failed to open /dev/mem\n");
		return STATUS_FAIL;
	}

	current_addr = (unsigned long)mem_dump_buf;
	for (i = 0; i < count; i++) {

		pad_len = buf_in[i*2] % PAGE_SIZE;

		if (pad_len)
			mmap_addr_aligned = buf_in[i*2] - pad_len;
		else
			mmap_addr_aligned = buf_in[i*2];

		mmap_size = buf_in[(i*2)+1];
		calculated_total_size += mmap_size;

		printf("physical_address=0x%lx  size=0x%lx"
			" mmap_addr_aligned=0x%lx pad_len=0x%lx\n",
			buf_in[(i*2)], mmap_size, mmap_addr_aligned, pad_len);

		if (mmap_size == 0)
			continue;

		if (calculated_total_size <= size) {
			size_left -=  mmap_size;
		} else {
			mmap_size = size_left;
			destination_buf_full_flag = 1;
		}

		if (mmap_size != 0) {
			base = mmap(0, (mmap_size + pad_len),
				PROT_READ|PROT_WRITE, MAP_SHARED,
				fd, mmap_addr_aligned);
			if (base == MAP_FAILED) {
				perror("mmap failed");
				close(fd);
				return MMAP_FAILED;
			}
		 } else {
			close(fd);
			return size;
		}

		if (base == (void *)0xffffffff) {
				close(fd);
				printf("mmap failed\n");
				return STATUS_FAIL;
		} else {
				memcpy((void *)current_addr, (base + pad_len),
					mmap_size);
				if (destination_buf_full_flag == 1) {
					calculated_total_size = size;
					munmap(base, (mmap_size + pad_len));
					break;
				}
				current_addr += mmap_size;
				munmap(base, (mmap_size + pad_len));
		}
	}

	printf("\n");
	close(fd);
	return calculated_total_size;
}
