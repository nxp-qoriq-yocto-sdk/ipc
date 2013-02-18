/*
 * @file: libdspboot.c
 *
 * Copyright 2013 Freescale Semiconductor, Inc.
 *
 * Author: Ashish Kumar <ashish.kumar@freescale.com>
 */

#include <stdio.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/shm.h>
#include <sys/ioctl.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <signal.h>
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <time.h>
#include "fsl_het_mgr.h"
#include "fsl_types.h"
#include "fsl_ipc_shm.h"
#include "bsc913x_dsp_boot.h"

/* defines */
#define MAX_ENTRIES	10
/* PA CCSR */
#define	DSPSR	0xE00D8
#define DSPSR_DSP1_READY 0x80000000
#define DSPSR_DSP2_READY 0x40000000

/* DSP CCSR */
#define DSP_GCR	0x18000
#define GIC_VIGR 0x17000
#define GIC_VIGR_VALUE 0x107
#define PASTATE	0x104
#define DSPSTATE 0x100
#define BOOT_JMP_ADDR	0x108
#define PASTATE_PAREADY	0x1
#define RESET	0x0

#define MB_256		0x10000000
#define IPC_SHMEM	MB_256

#define ADDTOMAP(PHYS, SZ)	\
			do {	\
				map[mapidx].phys_addr = PHYS;	\
				map[mapidx].vaddr = NULL;	\
				map[mapidx].size = SZ;	\
			} while (0);	\
			mapidx++
static int init_hetmgr();
static int init_devmem();
static int assign_memory_areas(sys_map_t *, int);
static int init_hugetlb(sys_map_t *, int);
static int dsp_ready_set(sys_map_t *, int);
static int load_dsp_image(char *, int);
static int set_sh_ctrl_pa_init(int);
static int set_ppc_ready(sys_map_t *, int);
static int reset_ppc_ready();
static int check_dsp_boot(int, int);
void cleanup(int, int);

/*
 * Global Variables
 *
 */

static range_t map[MAX_ENTRIES];
static int mapidx;

void dump_sys_map(sys_map_t het_sys_map)
{
	printf("SYSTEM MAP\n");
	printf("DSP PrivArea: Addr=%lx Size=%x\n",
		het_sys_map.smart_dsp_os_priv_area.phys_addr,
		het_sys_map.smart_dsp_os_priv_area.size);

	printf("Shared CtrlArea: Addr=%lx Size=%x\n",
		het_sys_map.sh_ctrl_area.phys_addr,
		het_sys_map.sh_ctrl_area.size);

	printf("DSP Core0 M2: Addr=%lx Size=%x\n",
		het_sys_map.dsp_core0_m2.phys_addr,
		het_sys_map.dsp_core0_m2.size);

	printf("DSP Core1 M2: Addr=%lx Size=%x\n",
		het_sys_map.dsp_core1_m2.phys_addr,
		het_sys_map.dsp_core1_m2.size);

	printf("DSP M3: Addr=%lx Size=%x\n",
		het_sys_map.dsp_m3.phys_addr,
		het_sys_map.dsp_m3.size);

	printf("PA CCSRBAR: Addr =%lx Size=%x\n",
		het_sys_map.pa_ccsrbar.phys_addr,
		het_sys_map.pa_ccsrbar.size);

	printf("DSP CCSRBAR: Addr =%lx Size=%x\n",
		het_sys_map.dsp_ccsrbar.phys_addr,
		het_sys_map.dsp_ccsrbar.size);
}

void *map_area(phys_addr_t phys_addr, unsigned int  *sz, int dev_mem)
{
	int i;
	void *vaddr = NULL;;
	uint32_t diff;
	int size = *sz;
	phys_addr_t nphys_addr;
	nphys_addr = phys_addr & 0xfffff000;
	if (phys_addr + size > nphys_addr + 0x1000)
		size = (size + 0x1000) & 0xfffff000;

	size += 0x1000 - size % 0x1000;
	diff = phys_addr - nphys_addr;

	for (i = 0; i < mapidx; i++)
		if (phys_addr >= map[i].phys_addr &&
		    phys_addr < map[i].phys_addr + map[i].size) {
			vaddr = mmap(0, size, (PROT_READ | \
				PROT_WRITE), MAP_SHARED, \
					dev_mem, nphys_addr);
			break;
		}

	if (vaddr) {
		*sz = size;
		vaddr += diff;
		return vaddr;
	}

	return NULL;

}

void unmap_area(void *vaddr, unsigned int size)
{
	munmap((void *)((uint32_t)vaddr & 0xfffff000), size);
}

/*
 * @get_hw_sem_value
 *
 */
static inline int get_hw_sem_value(uint32_t *val, int dev_het_mgr)
{
	int ret = 0;
	hw_sem_t hsem;
	hsem.sem_no = 1;
	ret = ioctl(dev_het_mgr, IOCTL_HW_SEM_GET_VALUE, &hsem);
	if (ret)
		return ret;

	*val = hsem.value;
	return ret;
}

/*
 * @set_hw_sem_value
 *
 */
static inline int set_hw_sem_value(uint32_t val, int dev_het_mgr)
{
	hw_sem_t hsem;
	hsem.sem_no = 1;
	hsem.value = val;
	return ioctl(dev_het_mgr, IOCTL_HW_SEM_SET_VALUE, &hsem);
}

/*
 * @get_dsp_uniq_sem_value
 *
 */
static inline int get_uniq_hw_sem_value(hw_sem_info_t *hseminfo,
		int dev_het_mgr)
{
	return ioctl(dev_het_mgr, IOCTL_HW_SEM_GET_UVALUE, hseminfo);
}

static int check_dsp_boot(int dev_mem, int dev_het_mgr)
{
	uint32_t val;
	int ret;

	hw_sem_info_t hwu;

	ret = get_uniq_hw_sem_value(&hwu, dev_het_mgr);
	if (ret)
		goto end;
	ret = get_hw_sem_value(&val, dev_het_mgr);
	if (ret)
		goto end;
	reload_print("val=%d hwu.dsp_uniq_val=%d\n", val, hwu.dsp_uniq_val);

	if (val == hwu.dsp_uniq_val) {
		val = 0;
		set_hw_sem_value(val, dev_het_mgr);
		printf("\n == DSP Booted up == \n");
	} else {
		printf("\n DSP HW Sem1 value not correctly set, value=%x\n",
		       val);
		printf("\n DSP Boot Failed, Please reset\n");
	}
end:
	/* in the end call this */
	cleanup(dev_mem, dev_het_mgr);
	if (ret) {
		printf("%s: Error in loading DSP\n", __func__);
		return -1;
	} else
		return DSP_BOOT_SUCCESS;
}

/*
 * @copy_file_part
 *
 * simple byte copy from file
 */
static int copy_file_part(void *virt_addr, uint32_t size, FILE * fp)
{
	int i, sz;
	unsigned char ch;
	unsigned char *vaddr = virt_addr;
	for (i = 0; i < size; i++) {
		sz = fread(&ch, 1, 1, fp);
		if (!sz) {
			printf("Error reading from file \n");
			return -1;
		}
		vaddr[i] = ch;
	}
	return 0;
}

static int init_devmem()
{
	reload_print("%s\n", __func__);
	int dev_mem = open("/dev/mem", O_RDWR);
	if (dev_mem == -1) {
		printf("Error: Cannot open /dev/mem.\n");
		return -1;
	}
	reload_print("dev_mem =%d\n", dev_mem);
	return dev_mem;
}

static int init_hetmgr()
{
	/* query ranges from /dev/het_mgr */
	reload_print("%s\n", __func__);
	int dev_het_mgr = open("/dev/het_mgr", O_RDWR);
	if (dev_het_mgr == -1) {
		printf("Error: Cannot open /dev/het_mgr\n");
		return  -1;
	}
	reload_print("dev_het_mgr =%d\n", dev_het_mgr);
	return dev_het_mgr;
}

static int assign_memory_areas(sys_map_t *het_sys_map, int dev_het_mgr)
{
	reload_print("%s\n", __func__);
	int ret = 0;
	/* open /dev/mem
	 * map dsp m2/m3/ddr
	 */
	/* Send IOCTL to get system map */
/*	ret = ioctl(dev_het_mgr, IOCTL_HET_MGR_GET_SYS_MAP, &het_sys_map);
	if (ret)
		return ret;
*/
	dump_sys_map(*het_sys_map);

	if ((*het_sys_map).smart_dsp_os_priv_area.phys_addr == 0xffffffff ||
	(*het_sys_map).dsp_core0_m2.phys_addr == 0xffffffff ||
	(*het_sys_map).dsp_core1_m2.phys_addr == 0xffffffff ||
	(*het_sys_map).dsp_m3.phys_addr == 0xffffffff ||
	(*het_sys_map).dsp_shared_size == 0xffffffff) {
		printf("Incorrect params\n");
		return -1;
	}
	mapidx = 0;
	ADDTOMAP((*het_sys_map).smart_dsp_os_priv_area.phys_addr,
		(*het_sys_map).smart_dsp_os_priv_area.size);
	ADDTOMAP((*het_sys_map).dsp_core0_m2.phys_addr,
		(*het_sys_map).dsp_core0_m2.size);
	ADDTOMAP((*het_sys_map).dsp_core1_m2.phys_addr,
		(*het_sys_map).dsp_core1_m2.size);
	ADDTOMAP((*het_sys_map).dsp_m3.phys_addr,
		(*het_sys_map).dsp_m3.size);
	ADDTOMAP((*het_sys_map).pa_ccsrbar.phys_addr,
		(*het_sys_map).pa_ccsrbar.size);
	ADDTOMAP((*het_sys_map).dsp_ccsrbar.phys_addr,
		(*het_sys_map).dsp_ccsrbar.size);

	return ret;
}

static int load_dsp_image(char *fname, int dev_mem)
{
	/* Read First 4 bytes */
	/* Read phys address and size */
	/* get vaddr from phys addr */
	/* call copy part */
	reload_print("%s\n", __func__);

	FILE *dspbin;
	uint8_t endian;
	uint32_t addr, size;
	void *vaddr;
	int ret = 0;
	uint32_t tsize = 0;
	dspbin = fopen(fname, "rb");
	if (!dspbin) {
		printf("%s File not found, exiting", fname);
		ret = -1;
		goto end;
	}

	/*Read first Byte */
	ret = fread(&endian, 1, 1, dspbin);
	while (!feof(dspbin)) {

		/*Read addr and size */
		ret = fread(&addr, 4, 1, dspbin);
		if (!ret) {
			if (ferror(dspbin))
				printf("%s: File read error - %d\n",
				       fname, errno);
			break;
		}

		ret = fread(&size, 4, 1, dspbin);
		if (!ret) {
			if (ferror(dspbin))
				printf("%s: File read error - %d\n",
				       fname, errno);
			break;
		}

		if (!size)
			continue;

		tsize = size;
		vaddr = map_area(addr, &tsize, dev_mem);
		if (!vaddr) {
			ret = -1;
			printf("\n Error in mapping physical address %x"
			" to virtual address\n", addr);
			goto end_close_file;
		}

		printf("\n Copy Part %x %x\n", addr, size);
		ret = copy_file_part(vaddr, size, dspbin);
		reload_print("%s: ret =%d\n", __func__, ret);
		unmap_area(vaddr, tsize);

		if (ret)
			goto end_close_file;
	}

end_close_file:
	fclose(dspbin);
end:
	return ret;
}

static inline int dsp_ready_set(sys_map_t *het_sys_map, int dev_mem)
{
	reload_print("%s\n", __func__);
	void *vaddr;
	int ret;
	uint32_t val;
	ret = 0;
	uint32_t tsize = 4;

	phys_addr_t phys_addr = (*het_sys_map).pa_ccsrbar.phys_addr + DSPSR;
	reload_print("\%s physical address %lx to virtual address\n",
			__func__, phys_addr);
	vaddr = map_area(phys_addr, &tsize, dev_mem);
	if (!vaddr) {
		printf("\nError in mapping physical address %lx to virtual"
		       "address\n", phys_addr);
		return -1;
	}
	val = *((volatile uint32_t *)vaddr);
	unmap_area((void *)vaddr, tsize);
	reload_print("dsp_SR val =%x (%i)\n", val, val);

	if (val & DSPSR_DSP1_READY) {
		printf("DSP READY SET\n");
		return ret;
	}
	return -1;
}

static int set_sh_ctrl_pa_init(int dev_het_mgr)
{
	reload_print("%s\n", __func__);
	return ioctl(dev_het_mgr, IOCTL_HET_MGR_SET_INITIALIZED, 0);
}

static int set_ppc_ready(sys_map_t *het_sys_map, int dev_mem)
{
	reload_print("%s\n", __func__);
	volatile uint32_t *vaddr;
	uint32_t tsize = 4;
	/* write to ppc ready */
	phys_addr_t phys_addr = (*het_sys_map).dsp_ccsrbar.phys_addr + DSP_GCR
				+ PASTATE;
	vaddr = map_area(phys_addr, &tsize, dev_mem);
	reload_print("%s: physical address %lx to virtual address\n",
		__func__ , (*het_sys_map).dsp_ccsrbar.phys_addr
		+ DSP_GCR + PASTATE);
	if (!vaddr) {
		printf("\nError in physical address %lx to virtual"
		       "address\n",
		       (*het_sys_map).dsp_ccsrbar.phys_addr +
		       DSP_GCR + PASTATE);
		return -1;
	}

	*vaddr |= PASTATE_PAREADY;
	unmap_area((void *)vaddr, tsize);
	return 0;
}

static int reset_ppc_ready(sys_map_t *het_sys_map, int dev_mem)
{
	reload_print("Entering func %s\n", __func__);
	volatile void *vaddr, *vaddr_pastate, *vaddr_boot_jmp_addr,
		 *vaddr_dspstate;
	int size = 0x1000;

	phys_addr_t phys_addr = (*het_sys_map).dsp_ccsrbar.phys_addr + DSP_GCR;
	reload_print("het_sys_map.dsp_ccsrbar.phys_addr %x DSP_GCR%x\n",
		(uint32_t)(*het_sys_map).dsp_ccsrbar.phys_addr,
		DSP_GCR);

	vaddr = mmap(0, size, (PROT_READ | PROT_WRITE),
			MAP_SHARED, dev_mem, phys_addr);
	if (!vaddr) {
		printf("\nError in physical address %lx to virtual"
		       "address\n",
		       (*het_sys_map).dsp_ccsrbar.phys_addr + DSP_GCR);
		return -1;
	}

	/* write to ppc ready */
	puts("Clearing PA_STATE");
	vaddr_pastate = vaddr + PASTATE;
	reload_print("before PASTATE= %d\n", *(int *)vaddr_pastate);
	*(int *)vaddr_pastate = RESET;
	reload_print("after PASTATE= %d\n", *(int *)vaddr_pastate);

	/* clear DSP STATE register */
	puts("Clearing DSP_STATE");

	vaddr_dspstate = vaddr + DSPSTATE;

	reload_print("before vaddr_dspstate= %d\n", *(int *)vaddr_dspstate);
	*(int *)vaddr_dspstate = RESET;
	reload_print("after vaddr_dspstate= %d\n", *(int *)vaddr_dspstate);

	/* clear BOOT_JMP_ADDR register */
	puts("Clearing BOOT_JMP_ADDR");

	vaddr_boot_jmp_addr = vaddr + BOOT_JMP_ADDR;

	reload_print("before BOOT_JMP_ADDR= %d\n", *(int *)vaddr_boot_jmp_addr);
	*(int *)vaddr_boot_jmp_addr = RESET;
	reload_print("after BOOT_JMP_ADDR= %d\n", *(int *)vaddr_boot_jmp_addr);
	unmap_area((void *)vaddr, size);
	return 0;
}
int send_vnmi_func(sys_map_t *het_sys_map, int dev_mem, int dev_het_mgr)
{
	reload_print("Entering func %s\n", __func__);
	volatile void *vaddr;
	int size = 0x8;
	phys_addr_t phys_addr = (*het_sys_map).dsp_ccsrbar.phys_addr + GIC_VIGR;

	reload_print("het_sys_map.dsp_ccsrbar.phys_addr %x GIC_VIGR%x\n",
		(uint32_t)(*het_sys_map).dsp_ccsrbar.phys_addr,
		GIC_VIGR);

	vaddr = mmap(0, size, (PROT_READ | PROT_WRITE),
			MAP_SHARED, dev_mem, phys_addr);

	if (!vaddr) {
		printf("\nError in physical address %lx to virtual"
		       "address\n",
		       (*het_sys_map).dsp_ccsrbar.phys_addr + GIC_VIGR);
		return -1;
	}
	*(int *)vaddr |= GIC_VIGR_VALUE;
	return 0;

}

static inline int init_hugetlb(sys_map_t *het_sys_map, int dev_het_mgr)
{
	reload_print("%s\n", __func__);
	int ret;
	void *pa_p;
	shared_area_t shared_area;
	pa_p = (void *)fsl_shm_init((*het_sys_map).dsp_shared_size);
	if (!pa_p) {
		ret = -1;
		goto end;
	}

	(shared_area).pa_ipc_shared.phys_addr = (uint32_t)pa_p;
	(shared_area).pa_ipc_shared.size = MB_256
				- (*het_sys_map).dsp_shared_size;

	(shared_area).dsp_ipc_shared.phys_addr = (uint32_t)pa_p + MB_256
					- (*het_sys_map).dsp_shared_size;
	(shared_area).dsp_ipc_shared.size = (*het_sys_map).dsp_shared_size;

	printf("PA Shared Area: Addr=%lx Size=%x\n",
		(shared_area).pa_ipc_shared.phys_addr,
		(shared_area).pa_ipc_shared.size);

	printf("DSP Shared Area: Addr=%lx Size=%x\n",
		(shared_area).dsp_ipc_shared.phys_addr,
		(shared_area).dsp_ipc_shared.size);

	/* attach the physical address with shared mem */
	ret = ioctl(dev_het_mgr, IOCTL_HET_MGR_SET_SHARED_AREA, &shared_area);
	if (ret) {
		perror("ioctl IOCTL_HET_MGR_SET_SHARED failedi:\n");
		ret = -1;
		goto end;
	}

end:
	return ret;
}

void cleanup(int dev_mem, int dev_het_mgr)
{
	reload_print("%s\n", __func__);
	close(dev_mem);
	close(dev_het_mgr);
}

int fsl_send_vnmi()
{
	reload_print("calling %s\n", __func__);
	int dev_mem = -1;
	int dev_het_mgr = -1;
	int ret = 0;
	sys_map_t het_sys_map;

	dev_het_mgr = init_hetmgr();
	if (dev_het_mgr == -1)
		goto end_vnmi;

	dev_mem = init_devmem();
	if (dev_mem == -1)
		goto end_vnmi;

	ret = ioctl(dev_het_mgr, IOCTL_HET_MGR_GET_SYS_MAP, &het_sys_map);
	if (ret) {
		perror("IOCTL_HET_MGR_GET_SYS_MAP:");
		goto end_vnmi;
	}

	if (reset_ppc_ready(&het_sys_map, dev_mem))
		goto end_vnmi;

	/* send vnmi */
	if (send_vnmi_func(&het_sys_map, dev_mem , dev_het_mgr))
		goto end_vnmi;

	cleanup(dev_mem, dev_het_mgr);
	return 0;

end_vnmi:
	cleanup(dev_mem, dev_het_mgr);
	printf("VNMI failed\n");
	return -1;
}

int fsl_load_dsp_image(char *fname)
{
	int dev_mem = -1;
	int dev_het_mgr = -1;
	int ret = 0;
	sys_map_t het_sys_map;
	static int dspbt_calld;

	if (dspbt_calld >= 1)
		printf("Reloading DSP image \"%s\"\n", fname);

	dev_het_mgr = init_hetmgr();
	if (dev_het_mgr == -1)
		goto end1;

	dev_mem = init_devmem();
	if (dev_mem == -1)
		goto end1;


	ret = ioctl(dev_het_mgr, IOCTL_HET_MGR_GET_SYS_MAP, &het_sys_map);
	if (ret) {
		perror("IOCTL_HET_MGR_GET_SYS_MAP:");
		goto end1;
	}

	if (assign_memory_areas(&het_sys_map, dev_het_mgr))
		goto end1;

	if (dspbt_calld == 0) {
		if (init_hugetlb(&het_sys_map, dev_het_mgr))
			goto end1;
	}

	dspbt_calld++;
	reload_print("dspbt_calld = %d\n", dspbt_calld);

	if (dsp_ready_set(&het_sys_map, dev_mem))
		goto end1;

	ret = load_dsp_image(fname, dev_mem);
	reload_print("%s: ret = (%i)\n", __func__, ret);
	if (ret)
		goto end1;

	printf("DSP image copied\n");
	if (set_sh_ctrl_pa_init(dev_het_mgr))
		goto end1;

	set_ppc_ready(&het_sys_map, dev_mem);

	/* sleep for 2 sec */
	sleep(2);

	if (DSP_BOOT_SUCCESS != check_dsp_boot(dev_mem, dev_het_mgr))
		goto error;
	else
		return 0;
end1:
	cleanup(dev_mem, dev_het_mgr);
error:
	printf("%s: Error in loading Dsp image Error\n", __func__);
	return -1;
}

int fsl_restart_L1(fsl_ipc_t ipc, char *fname)
{

	if (fsl_ipc_reinit(ipc) < 0) {
		printf("Error in fsl_ipc_reinit\n");
		return -1;
	}

	if (fsl_send_vnmi() < 0) {
		printf("Error in fsl_send_vnmi\n");
		return -1;
	}

	puts("sleep 10 sec");
	sleep(10);

	if (fsl_load_dsp_image(fname) < 0) {
		printf("%s: Error in fsl_load_dsp_image\n", __func__);
		return -1;
	}
	return 0;
}
