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
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include "logdefs.h"
#include "fsl_types.h"
#include "fsl_het_mgr.h"
#include "fsl_ipc_errorcodes.h"
#include "bsc913x_heterogeneous.h"

#define SMAP(I, A, B, C)   map[I].phys_addr = A;   \
			map[I].vaddr = B; \
			map[I].size = C; \
			printf("%x %x %x MAP[%d] P:%x V:%x SZ:%x\n", A, B, C, \
			I, map[I].phys_addr, \
			map[I].vaddr, map[I].size);

#define MMAP(P, SZ)	vaddr = mmap(0, SZ, (PROT_READ | \
			PROT_WRITE), MAP_SHARED, dev_mem, \
			P);     \
			printf("%x \n", vaddr);\
			if (vaddr == 0xffffffff) return -1;\
			SMAP(mapidx, P, vaddr, SZ);\
			mapidx++;

#define TLB1_SH_MEM_SIZE	0x1000000

int mapidx;
int dev_het_mgr;
int dev_mem;
int shmid;

range_t	map[10];
sys_map_t het_sys_map;
range_t	shared_area;
range_t dsp_ccsr;
range_t pa_ccsr;

void cleanup();
/* external methods */
void *fsl_ipc_helper_p2v(phys_addr_t);
int get_shared_ctrl_area(range_t *r);
int get_free_pool(range_t *r);

int fsl_ipc_helper_init(void)
{
	int ret = ERR_SUCCESS;
	mapidx = 0;
	dev_het_mgr = 0;
	dev_mem = 0;
	shmid = 0;
	ENTER();

	memset(&shared_area, 0, sizeof(range_t));
	memset(&dsp_ccsr, 0, sizeof(range_t));
	memset(&pa_ccsr, 0, sizeof(range_t));
	memset(map, 0, 10*sizeof(range_t));
	memset(&het_sys_map, 0, sizeof(sys_map_t));

	ret = init_het_mgr();
	if (ret)
		goto end;

	ret = init_dev_mem();
	if (ret)
		goto end;

	ret = map_shared_mem();
	if (ret)
		goto end;
end:
	if (ret)
		cleanup();
	EXIT(ret);
	return ret;
}

void cleanup()
{
	if (dev_het_mgr != -1)
		close(dev_het_mgr);

	if (dev_mem != -1)
		close(dev_mem);

}

int init_het_mgr()
{
	int ret = ERR_SUCCESS;
	ENTER();
	dev_het_mgr = open("/dev/het_mgr", O_RDWR);
	if (dev_het_mgr == -1) {
		printf("Error: Cannot open /dev/het_mgr. %d\n");
		ret = -ERR_DEV_HETMGR_FAIL;
	}
	EXIT(ret);
	return ret;
}

int init_dev_mem()
{
	int ret = ERR_SUCCESS;
	ENTER();
	dev_mem = open("/dev/mem", O_RDWR);
	if (dev_mem == -1) {
		printf("Error: Cannot open /dev/mem.\n");
		ret = -ERR_DEV_MEM_FAIL;
	}
	EXIT(ret);
	return ret;
}

int map_shared_mem()
{
	int ret = ERR_SUCCESS;
	void *vaddr;
	range_t r;
	ENTER();
	/* open /dev/mem
	 * map dsp m2/m3/ddr
	 */
	/* Send IOCTL to get system map */
	ret = ioctl(dev_het_mgr, IOCTL_HET_MGR_GET_SYS_MAP, &het_sys_map);
	if (ret)
		goto end;

	printf("MMAP --- \n");

	/* MAP DSP private area in ddr */
	MMAP(het_sys_map.dsp_core0_m2.phys_addr,
		het_sys_map.dsp_core0_m2.size);
#ifdef PSC9132
	MMAP(het_sys_map.dsp_core1_m2.phys_addr,
		het_sys_map.dsp_core1_m2.size);

	MMAP(het_sys_map.dsp_m3.phys_addr,
		het_sys_map.dsp_m3.size);
#endif
	printf("----MMAP  \n");

	/* Send IOCTL to get shmid */
	ret = ioctl(dev_het_mgr, IOCTL_HET_MGR_GET_SHMID, &shmid);
	if (ret)
		goto end;

	/* shm attach */
	printf("HugeTLB shmid: 0x%x\n", shmid);
	r.vaddr = shmat(shmid, 0, 0);

	if (r.vaddr == (char *)-1) {
		perror("Shared memory attach failure");
		shmctl(shmid, IPC_RMID, NULL);
		EXIT(-1);
		return -1;
	}

	printf("clear the memory \n");
	memset(r.vaddr, 0, 4); /* try with 4 bytes */

	r.size = TLB1_SH_MEM_SIZE;

	ret = ioctl(dev_het_mgr, IOCTL_HET_MGR_V2P, &r);
	if (ret)
		goto end;

	printf("V2P %x %x \n", (uint32_t)r.vaddr, r.phys_addr);

	map[mapidx].phys_addr = r.phys_addr;
	map[mapidx].vaddr = r.vaddr;
	map[mapidx].size = r.size;
	mapidx++;

end:
	EXIT(ret);
	return ret;
}

int get_shared_ctrl_area(range_t *r)
{
	int ret = ERR_SUCCESS;
	ENTER();
	if (!shared_area.vaddr) {
		r->phys_addr = het_sys_map.sh_ctrl_area.phys_addr;
		r->size = het_sys_map.sh_ctrl_area.size;
		r->vaddr = mmap(0, r->size, (PROT_READ |
			PROT_WRITE), MAP_SHARED, dev_mem,
				r->phys_addr);
		if (r->vaddr == 0xffffffff) {
			EXIT(-1);
			return -1;
		}
		memcpy(&shared_area, r, sizeof(range_t));
		DUMPR(&shared_area);
	} else
		memcpy(r, &shared_area, sizeof(range_t));
	DUMPR(r);
	EXIT(ret);
	return ret;
}

int get_dsp_ccsr_area(range_t *r)
{
	int ret = ERR_SUCCESS;
	ENTER();
	if (!dsp_ccsr.vaddr) {
		r->phys_addr = het_sys_map.dsp_ccsrbar.phys_addr;
		r->size = het_sys_map.dsp_ccsrbar.size;
		r->vaddr = mmap(0, r->size, (PROT_READ |
			PROT_WRITE), MAP_SHARED, dev_mem,
				r->phys_addr);

		if (r->vaddr == 0xffffffff) {
			EXIT(-1);
			return -1;
		}

		memcpy(&dsp_ccsr, r, sizeof(range_t));
	} else
		memcpy(r, &dsp_ccsr, sizeof(range_t));
	EXIT(ret);
	return ret;
}

int get_pa_ccsr_area(range_t *r)
{
	int ret = ERR_SUCCESS;
	ENTER();
	if (!pa_ccsr.vaddr) {
		r->phys_addr = het_sys_map.pa_ccsrbar.phys_addr;
		r->size = het_sys_map.pa_ccsrbar.size;
		r->vaddr = mmap(0, r->size, (PROT_READ |
			PROT_WRITE), MAP_SHARED, dev_mem,
				r->phys_addr);

		if (r->vaddr == 0xffffffff) {
			EXIT(-1);
			return -1;
		}

		memcpy(&pa_ccsr, r, sizeof(range_t));
	} else
		memcpy(r, &pa_ccsr, sizeof(range_t));

	EXIT(ret);
	return ret;
}


/*
 * @p2v
 *
 */
void *fsl_ipc_helper_p2v(phys_addr_t phys_addr)
{
	int i, j;
	void *vaddr = NULL;
	ENTER();
	printf("%x \n", phys_addr);
	for (i = 0; i < mapidx; i++)
		if (phys_addr >= map[i].phys_addr &&
			phys_addr < map[i].phys_addr + map[i].size)
			vaddr = (void *)(phys_addr - map[i].phys_addr) +
				(uint32_t)map[i].vaddr;
	printf("Return =%x\n", (uint32_t)vaddr);
	EXIT(vaddr);
	return vaddr;
}

int get_free_pool(range_t *r)
{
	range_t	shctrl;
	int ret;
	ENTER();
	ret = get_shared_ctrl_area(&shctrl);
	if (ret)
		goto end;

	volatile os_het_control_t *ctrl = (os_het_control_t *) shctrl.vaddr;
	r->phys_addr = ctrl->pa_shared_mem.start_addr;
	r->size = ctrl->pa_shared_mem.size;
	r->vaddr = fsl_ipc_helper_p2v(r->phys_addr);
	DUMPR(r);

end:
	EXIT(ret);
	return ret;
}
