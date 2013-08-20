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
#include <stdarg.h>
#include <stdio.h>
#include <stdint.h>
#include <time.h>
#include "fsl_het_mgr.h"
#include "fsl_ipc_types.h"
#include "fsl_ipc_shm.h"
#include "dsp_boot.h"
#include "dsp_compact.h"

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

static int init_hetmgr();
static int init_devmem();
static int assign_memory_areas(void *);
static int init_hugetlb(void *);
static int dsp_ready_check(void *);
static int load_dsp_image(char *, void *);
static int set_sh_ctrl_pa_init(int);
static int set_ppc_ready(void *);
static int reset_ppc_ready(void *);
static int check_dsp_boot(void *);
static void cleanup(int, int);

void func_add2map(void *dsp_bt, uint64_t phys, uint32_t sz)
{

	int i = ((dsp_bt_t *)dsp_bt)->map_id;

	((dsp_bt_t *)dsp_bt)->map_d[i].phys_addr = phys;
	((dsp_bt_t *)dsp_bt)->map_d[i].vaddr = NULL;
	((dsp_bt_t *)dsp_bt)->map_d[i].size = sz;

	((dsp_bt_t *)dsp_bt)->map_id += 1;

	return;
}

void dump_sys_map(sys_map_t het_sys_map)
{
	printf("SYSTEM MAP\n");
	printf("DSP PrivArea: Addr=%llx Size=%x\n",
		het_sys_map.smart_dsp_os_priv_area.phys_addr,
		het_sys_map.smart_dsp_os_priv_area.size);

	printf("Shared CtrlArea: Addr=%llx Size=%x\n",
		het_sys_map.sh_ctrl_area.phys_addr,
		het_sys_map.sh_ctrl_area.size);

	printf("DSP Core0 M2: Addr=%llx Size=%x\n",
		het_sys_map.dsp_core0_m2.phys_addr,
		het_sys_map.dsp_core0_m2.size);

	printf("DSP Core1 M2: Addr=%llx Size=%x\n",
		het_sys_map.dsp_core1_m2.phys_addr,
		het_sys_map.dsp_core1_m2.size);

	printf("DSP M3: Addr=%llx Size=%x\n",
		het_sys_map.dsp_m3.phys_addr,
		het_sys_map.dsp_m3.size);

	printf("PA CCSRBAR: Addr =%llx Size=%x\n",
		het_sys_map.pa_ccsrbar.phys_addr,
		het_sys_map.pa_ccsrbar.size);

	printf("DSP CCSRBAR: Addr =%llx Size=%x\n",
		het_sys_map.dsp_ccsrbar.phys_addr,
		het_sys_map.dsp_ccsrbar.size);
}

void *map_area(uint64_t phys_addr, uint32_t  *sz, void *dsp_bt)
{
	int i;
	void *vaddr = NULL;;
	uint64_t diff;
	int size = *sz;
	uint64_t nphys_addr;
	int dev_mem = ((dsp_bt_t *)dsp_bt)->dev_mem;
	int mapidx = ((dsp_bt_t *)dsp_bt)->map_id;

	nphys_addr = phys_addr & MAP_AREA_MASK;
	if (phys_addr + size > nphys_addr + 0x1000)
		size = (size + 0x1000) & MAP_AREA_MASK;

	size += 0x1000 - size % 0x1000;
	diff = phys_addr - nphys_addr;

	for (i = 0; i < mapidx; i++)
		if (phys_addr >= ((dsp_bt_t *)dsp_bt)->map_d[i].phys_addr &&
		    phys_addr < ((dsp_bt_t *)dsp_bt)->map_d[i].phys_addr +
		    ((dsp_bt_t *)dsp_bt)->map_d[i].size) {
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

void *map_area64(uint64_t phys_addr, uint64_t  *sz, void *dsp_bt)
{
	int i;
	void *vaddr = NULL;;
	uint64_t diff;
	int size = *sz;
	uint64_t nphys_addr;
	int dev_mem = ((dsp_bt_t *)dsp_bt)->dev_mem;
	int mapidx = ((dsp_bt_t *)dsp_bt)->map_id;

	nphys_addr = phys_addr & MAP_AREA_MASK;
	if (phys_addr + size > nphys_addr + 0x1000)
		size = (size + 0x1000) & MAP_AREA_MASK;

	size += 0x1000 - size % 0x1000;
	diff = phys_addr - nphys_addr;

	for (i = 0; i < mapidx; i++)
		if (phys_addr >= ((dsp_bt_t *)dsp_bt)->map_d[i].phys_addr &&
		    phys_addr < ((dsp_bt_t *)dsp_bt)->map_d[i].phys_addr +
		    ((dsp_bt_t *)dsp_bt)->map_d[i].size) {
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

void unmap_area(void *vaddr, uint64_t size)
{
	munmap((void *)((uint32_t)vaddr & VIR_ADDR32_MASK), size);
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

static int check_dsp_boot(void *dsp_bt)
{
	uint32_t val;
	int ret;
	int dev_het_mgr = ((dsp_bt_t *)dsp_bt)->het_mgr;

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
	return dev_het_mgr;
}

static int assign_memory_areas(void *dsp_bt)
{
	reload_print("%s\n", __func__);
	int ret = 0;
	/* open /dev/mem
	 * map dsp m2/m3/ddr
	 * sys_map_t *het_sys_map, int dev_het_mgr)
	 */
	sys_map_t het_sys_map = ((dsp_bt_t *)dsp_bt)->het_sys_map;

	dump_sys_map(het_sys_map);

	if ((het_sys_map).smart_dsp_os_priv_area.phys_addr == 0xffffffff ||
	    (het_sys_map).dsp_core0_m2.phys_addr == 0xffffffff ||
	    (het_sys_map).dsp_core1_m2.phys_addr == 0xffffffff ||
	    (het_sys_map).dsp_m3.phys_addr == 0xffffffff ||
	    (het_sys_map).dsp_shared_size == 0xffffffff) {
		printf("Incorrect params\n");
		return -1;
	}

	((dsp_bt_t *)dsp_bt)->map_id = 0;
	func_add2map(dsp_bt, (het_sys_map).smart_dsp_os_priv_area.phys_addr,
		(het_sys_map).smart_dsp_os_priv_area.size);
	func_add2map(dsp_bt, (het_sys_map).dsp_core0_m2.phys_addr,
		(het_sys_map).dsp_core0_m2.size);
	func_add2map(dsp_bt, (het_sys_map).dsp_core1_m2.phys_addr,
		(het_sys_map).dsp_core1_m2.size);
	func_add2map(dsp_bt, (het_sys_map).dsp_m3.phys_addr,
		(het_sys_map).dsp_m3.size);
	func_add2map(dsp_bt, (het_sys_map).pa_ccsrbar.phys_addr,
		(het_sys_map).pa_ccsrbar.size);
	func_add2map(dsp_bt, (het_sys_map).dsp_ccsrbar.phys_addr,
		(het_sys_map).dsp_ccsrbar.size);

	return ret;
}

static int load_dsp_image(char *fname, void *dsp_bt)
{
	/* Read First 4 bytes */
	/* Read phys address and size */
	/* get vaddr from phys addr */
	/* call copy part */

	FILE *dspbin;
	uint8_t endian;
	uint64_t addr;
	void *vaddr;
	int ret = 0;
	uint64_t tsize = 0, size;
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
		ret = fread(&addr, ADDR_SIZE, 1, dspbin);
		if (!ret) {
			if (ferror(dspbin))
				printf("%s: File read error - %d\n",
				       fname, errno);
			break;
		}

		ret = fread(&size, ADDR_SIZE, 1, dspbin);
		if (!ret) {
			if (ferror(dspbin))
				printf("%s: File read error - %d\n",
				       fname, errno);
			break;
		}

		if (!size)
			continue;

		tsize = size;
		/* Read the intvec_addr and skip its mmap*/
		if (addr == 0xffffffffffffffff && size == 8) {
			ret = fread(&((dsp_bt_t *)dsp_bt)->intvec_addr,
				ADDR_SIZE, 1, dspbin);
			printf("intvec_addr =%llx in L1 Binary\n",
				((dsp_bt_t *)dsp_bt)->intvec_addr);
			if (!ret) {
				if (ferror(dspbin))
					printf("%s: File read error - %d\n",
				       fname, errno);
				break;
			}
			continue;
		}
		vaddr = map_area64(addr, &tsize, dsp_bt);
		if (!vaddr) {
			ret = -1;
			printf("\n Error in mapping physical address"
			" %llx to virtual address in %s\n",
			(long long unsigned int)addr, __func__);
			goto end_close_file;
		}

		printf("\n Copy Part %llx %llx\n", (long long unsigned int)addr,
			       (long long unsigned int)size);
			ret = copy_file_part(vaddr, size, dspbin);
			unmap_area(vaddr, tsize);
			ret = 0;

		reload_print("%s: ret =%d\n", __func__, ret);

		if (ret)
			goto end_close_file;
	}

end_close_file:
	fclose(dspbin);
end:
	return ret;
}

static int dsp_ready_check(void *dsp_bt)
{
	reload_print("%s\n", __func__);
	void *vaddr;
	int ret;
	uint32_t val;
	ret = 0;
	uint32_t tsize = 4;
	sys_map_t *het_sys_map = &((dsp_bt_t *)dsp_bt)->het_sys_map;

	uint64_t phys_addr = (*het_sys_map).pa_ccsrbar.phys_addr + DSPSR;
	vaddr = map_area(phys_addr, &tsize, dsp_bt);
	if (!vaddr) {
		printf("\nError in mapping physical address %llx to virtual"
		       "address\n", phys_addr);
		return -1;
	}
	reload_print("%s physical address %llx to virtual address %lx\n",
			__func__, phys_addr, (long)vaddr);
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
	return ioctl(dev_het_mgr, IOCTL_HET_MGR_SET_INITIALIZED, 0);
}

static int set_ppc_ready(void *dsp_bt)
{
	reload_print("%s\n", __func__);
	sys_map_t *het_sys_map = &((dsp_bt_t *)dsp_bt)->het_sys_map;
	volatile unsigned long *vaddr;
	uint32_t tsize = 4;
	/* write to ppc ready */
	uint64_t phys_addr = (*het_sys_map).dsp_ccsrbar.phys_addr +
		DSP_GCR + PASTATE;
	vaddr = map_area(phys_addr, &tsize, dsp_bt);
	reload_print("%s: physical address %lx to virtual address\n",
		__func__ , (*het_sys_map).dsp_ccsrbar.phys_addr
		+ DSP_GCR + PASTATE);
	if (!vaddr) {
		printf("\nError in physical address %llx to virtual"
		       "address\n",
		       (*het_sys_map).dsp_ccsrbar.phys_addr +
		       DSP_GCR + PASTATE);
		return -1;
	}

	*vaddr |= PASTATE_PAREADY;
	unmap_area((void *)vaddr, tsize);
	return 0;
}

static int reset_ppc_ready(void *dsp_bt)
{
	reload_print("Entering func %s\n", __func__);
	sys_map_t *het_sys_map = &((dsp_bt_t *)dsp_bt)->het_sys_map;
	int dev_mem = ((dsp_bt_t *)dsp_bt)->dev_mem;
	volatile void *vaddr, *vaddr_pastate, *vaddr_boot_jmp_addr,
		 *vaddr_dspstate;
	int size = 0x1000;

	unsigned long phys_addr = (*het_sys_map).dsp_ccsrbar.phys_addr +
		DSP_GCR;
	reload_print("het_sys_map.dsp_ccsrbar.phys_addr %x DSP_GCR%x\n",
		(uint32_t)(*het_sys_map).dsp_ccsrbar.phys_addr,
		DSP_GCR);

	vaddr = mmap(0, size, (PROT_READ | PROT_WRITE),
			MAP_SHARED, dev_mem, phys_addr);
	if (!vaddr) {
		printf("\nError in physical address %llx to virtual"
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

int send_vnmi_func(void *dsp_bt)
{
	reload_print("Entering func %s\n", __func__);
	sys_map_t *het_sys_map = &((dsp_bt_t *)dsp_bt)->het_sys_map;
	int dev_mem = ((dsp_bt_t *)dsp_bt)->dev_mem;
	volatile void *vaddr;
	int size = 0x8;
	uint64_t phys_addr = (*het_sys_map).dsp_ccsrbar.phys_addr +
		GIC_VIGR;

	reload_print("het_sys_map.dsp_ccsrbar.phys_addr %x GIC_VIGR%x\n",
		(uint32_t)(*het_sys_map).dsp_ccsrbar.phys_addr,
		GIC_VIGR);

	vaddr = mmap(0, size, (PROT_READ | PROT_WRITE),
			MAP_SHARED, dev_mem, phys_addr);

	if (!vaddr) {
		printf("\nError in physical address %llx to virtual"
		       "address\n",
		       (*het_sys_map).dsp_ccsrbar.phys_addr + GIC_VIGR);
		return -1;
	}
	*(int *)vaddr |= GIC_VIGR_VALUE;
	return 0;

}

static int init_hugetlb(void *dsp_bt)
{
	reload_print("%s\n", __func__);
	int ret;
	void *pa_p;
	shared_area_t shared_area;
	sys_map_t *het_sys_map = &((dsp_bt_t *)dsp_bt)->het_sys_map;
	int dev_het_mgr = ((dsp_bt_t *)dsp_bt)->het_mgr;
	pa_p = (void *)fsl_shm_init((*het_sys_map).dsp_shared_size);
	if (!pa_p) {
		ret = -1;
		goto end;
	}

	(shared_area).pa_ipc_shared.phys_addr = (unsigned long)pa_p;
	(shared_area).pa_ipc_shared.size = MB_256
				- (*het_sys_map).dsp_shared_size;
	(shared_area).dsp_ipc_shared.phys_addr = (unsigned long)pa_p + MB_256
					- (*het_sys_map).dsp_shared_size;
	(shared_area).dsp_ipc_shared.size = (*het_sys_map).dsp_shared_size;

	printf("PA Shared Area: Addr=%llx Size=%x\n",
		(shared_area).pa_ipc_shared.phys_addr,
		(shared_area).pa_ipc_shared.size);

	printf("DSP Shared Area: Addr=%llx Size=%x\n",
		(shared_area).dsp_ipc_shared.phys_addr,
		(shared_area).dsp_ipc_shared.size);

	/* attach the physical address with shared mem */
	ret = ioctl(dev_het_mgr, IOCTL_HET_MGR_SET_SHARED_AREA, &shared_area);
	if (ret) {
		perror("ioctl IOCTL_HET_MGR_SET_SHARED failed:\n");
		ret = -1;
		goto end;
	}

end:
	return ret;
}

static void cleanup(int dev_mem, int dev_het_mgr)
{
	reload_print("%s\n", __func__);
	close(dev_mem);
	close(dev_het_mgr);
}

int fsl_send_vnmi(void *dsp_bt)
{
	reload_print("calling %s\n", __func__);
	int ret = 0;

	((dsp_bt_t *)dsp_bt)->het_mgr = init_hetmgr();
	if (((dsp_bt_t *)dsp_bt)->het_mgr == -1) {
		printf("error in init_hetmgr frm %s", __func__);
		return -1;
	}

	((dsp_bt_t *)dsp_bt)->dev_mem = init_devmem();
	if (((dsp_bt_t *)dsp_bt)->dev_mem == -1) {
		printf("error in dev_mem frm %s", __func__);
		return -1;
	}

	ret = ioctl(((dsp_bt_t *)dsp_bt)->het_mgr,
			IOCTL_HET_MGR_GET_SYS_MAP,
			&((dsp_bt_t *)dsp_bt)->het_sys_map);
	if (ret) {
		perror("IOCTL_HET_MGR_GET_SYS_MAP:");
		printf("frm %s\n", __func__);
		return -1;
	}

	if (reset_ppc_ready(dsp_bt) < 0) {
		printf("error in init_hetmgr frn %s", __func__);
		return -1;
	}

	/* send vnmi */
	if (send_vnmi_func(dsp_bt) < 0) {
		printf("error in send_vnmi_func frm %s", __func__);
		return -1;
	}

	return ret;
}

int pre_load_913x(int count, ...)
{
	int ret = 0;
	va_list arg_l;
	va_start(arg_l, count);
	void *dsp_bt = va_arg(arg_l, void *);
	((dsp_bt_t *)dsp_bt)->het_mgr = init_hetmgr();
	if (((dsp_bt_t *)dsp_bt)->het_mgr == -1) {
		printf("error in het_mgr from %s\n", __func__);
		return -1;
	}

	((dsp_bt_t *)dsp_bt)->dev_mem = init_devmem();
	if (((dsp_bt_t *)dsp_bt)->dev_mem == -1) {
		printf("error in dev_mem from %s\n", __func__);
		return -1;
	}

	ret = ioctl(((dsp_bt_t *)dsp_bt)->het_mgr,
			IOCTL_HET_MGR_GET_SYS_MAP,
		    &((dsp_bt_t *)dsp_bt)->het_sys_map);
	if (ret) {
		perror("IOCTL_HET_MGR_GET_SYS_MAP:");
		return -1;
	}

	if (assign_memory_areas(dsp_bt)) {
		printf("error in assign_memory_ar frm %s\n", __func__);
		return -1;
	}

	if (init_hugetlb(dsp_bt)) {
		printf("error in init_hugetlb frm %s\n", __func__);
		return -1;
	}

	if (fsl_913x_ipc_init(dsp_bt)) {
		printf("error in fsl_913x_ipc_init frm %s\n", __func__);
		return -1;
	}

	if (dsp_ready_check(dsp_bt)) {
		printf("error in dsp_ready_check frm %s\n", __func__);
		return -1;
	}
	va_end(arg_l);
	return ret;
}

int load_913x(char *fname, void *dsp_bt)
{
	int ret = 0;

	ret = load_dsp_image(fname, dsp_bt);
	if (ret < 0) {
		printf("error in load_dsp_image frm %s\n", __func__);
		return -1;
	}
	printf("DSP image copied\n");
	return ret;
}

int post_load_913x(void *dsp_bt)
{
	int dev_het_mgr = ((dsp_bt_t *)dsp_bt)->het_mgr;
	int ret = 0;

	if (set_sh_ctrl_pa_init(dev_het_mgr)) {
		printf("error in set_sh_ctrl_pa_init frm %s\n",
			__func__);
		return -1;
	}

	if (set_ppc_ready(dsp_bt) < 0) {
		printf("error in set_ppc_ready frm %s\n", __func__);
		return -1;
	}
	/* sleep for 4 sec */
	puts("sleep 4 sec for dsp to be ready");
	sleep(4);

	if (DSP_BOOT_SUCCESS != check_dsp_boot(dsp_bt)) {
		printf("error in check_dsp_boot frm %s\n", __func__);
		return -1;
	}
	return ret;
}

int b913x_load_dsp_image(char *fname)
{
	void *dsp_bt;
	dsp_bt = (void *)malloc(sizeof(dsp_bt_t));

	((dsp_bt_t *)dsp_bt)->het_mgr = -1;
	((dsp_bt_t *)dsp_bt)->dev_mem = -1;

	/* Assign func Pointers*/
	((dsp_bt_t *)dsp_bt)->pre_load = &pre_load_913x;
	((dsp_bt_t *)dsp_bt)->load_image = &load_913x;
	((dsp_bt_t *)dsp_bt)->post_load = &post_load_913x;


	/* Call func Pointers*/
	if (((dsp_bt_t *)dsp_bt)->pre_load(1, dsp_bt) < 0)
		goto end_b913x_load_dsp_image;

	if (((dsp_bt_t *)dsp_bt)->load_image(fname, dsp_bt) < 0)
		goto end_b913x_load_dsp_image;

	if (((dsp_bt_t *)dsp_bt)->post_load(dsp_bt) < 0)
		goto end_b913x_load_dsp_image;
	else {
		cleanup(((dsp_bt_t *)dsp_bt)->dev_mem,
			((dsp_bt_t *)dsp_bt)->het_mgr);
		free(dsp_bt);
		return 0;
	}

end_b913x_load_dsp_image:
	cleanup(((dsp_bt_t *)dsp_bt)->dev_mem,
		((dsp_bt_t *)dsp_bt)->het_mgr);
	return -1;
}

int pre_reload_913x(int count, ...)
{
	va_list arg_r;
	va_start(arg_r, count);
	void *dsp_bt = va_arg(arg_r, void *);
	void *ipc = va_arg(arg_r, void *);

	if (fsl_ipc_reinit(ipc) < 0) {
		printf("Error in fsl_ipc_reinit\n");
		return -1;
	}

	if (fsl_send_vnmi(dsp_bt) < 0) {
		printf("VNMI failed\n");
		printf("Error in fsl_send_vnmi\n");
		return -1;
	}
	puts("sleep 10 sec");
	sleep(10);

	if (assign_memory_areas(dsp_bt)) {
		printf("error in assign_memory_ar frm %s\n", __func__);
		return -1;
	}


	if (dsp_ready_check(dsp_bt)) {
		printf("error in dsp_ready_check frm %s\n", __func__);
		return -1;
	}

	va_end(arg_r);
	return 0;
}

int fsl_restart_L1(fsl_ipc_t ipc, char *fname)
{
	void *dsp_bt;
	dsp_bt = (void *)malloc(sizeof(dsp_bt_t));

	((dsp_bt_t *)dsp_bt)->het_mgr = -1;
	((dsp_bt_t *)dsp_bt)->dev_mem = -1;

	/* Assign func Pointers*/
	((dsp_bt_t *)dsp_bt)->pre_load = &pre_reload_913x;
	((dsp_bt_t *)dsp_bt)->load_image = &load_913x;
	((dsp_bt_t *)dsp_bt)->post_load = &post_load_913x;


	/* Call func Pointers*/
	if (((dsp_bt_t *)dsp_bt)->pre_load(2, dsp_bt, ipc) < 0)
		goto end_fsl_restart_L1;


	if (((dsp_bt_t *)dsp_bt)->load_image(fname, dsp_bt) < 0)
		goto end_fsl_restart_L1;

	if (((dsp_bt_t *)dsp_bt)->post_load(dsp_bt) < 0)
		goto end_fsl_restart_L1;
	else {
		cleanup(((dsp_bt_t *)dsp_bt)->dev_mem,
			((dsp_bt_t *)dsp_bt)->het_mgr);
		return 0;
	}

end_fsl_restart_L1:
	cleanup(((dsp_bt_t *)dsp_bt)->dev_mem,
		((dsp_bt_t *)dsp_bt)->het_mgr);
	((dsp_bt_t *)dsp_bt)->het_mgr = -1;
	((dsp_bt_t *)dsp_bt)->dev_mem = -1;
	return -1;

}

/*dsp_bt code for B4860*/
#ifdef B4860
static int release_starcore_B4(void *dsp_bt)
{
	int core_id = ((dsp_bt_t *)dsp_bt)->core_id;
	uint32_t intvec_addr = ((dsp_bt_t *)dsp_bt)->intvec_addr;
	sys_map_t *het_sys_map = &((dsp_bt_t *)dsp_bt)->het_sys_map;

	/*map to this value size = 0x1000000*/
	uint64_t size = het_sys_map->pa_ccsrbar.size;
	volatile uint32_t *vaddr = NULL;

	/*map to this value phys_addr = 0xffe000000*/
	uint64_t phys_addr = het_sys_map->pa_ccsrbar.phys_addr;

	core_id += 4;

	vaddr = map_area64(phys_addr, &size, dsp_bt);
	if (!vaddr) {
		printf("\nError in mapping physical address %llx to virtual"
		       "address\n", (long long unsigned int)phys_addr);
		return -1;
	}

	printf("Before StarCore release ========\n");
	printf("LCC_BSTRH=0x%x\n", *(vaddr + LCC_BSTRH));
	printf("LCC_BSTRL=0x%x\n", *(vaddr + LCC_BSTRL));
	printf("LCC_BSTAR=0x%x\n", *(vaddr + LCC_BSTAR));
	printf("GCR_CDCER0=0x%x\n", *(vaddr + GCR_CDCER0));
	printf("GCR_CHMER0=0x%x\n", *(vaddr + GCR_CHMER0));
	printf("DCFG_BRR=0x%x\n", *(vaddr + DCFG_BRR));

	*(vaddr + LCC_BSTRH) =  0x00000000;
	asm("lwsync");
	*(vaddr + LCC_BSTRL) =  intvec_addr;
	asm("lwsync");
	*(vaddr + LCC_BSTAR) =  0x8100000C;
	asm("lwsync");
	*(vaddr + GCR_CDCER0) =  0x000003f0;
	asm("lwsync");
	*(vaddr + GCR_CHMER0) =  0x00003f00;
	asm("lwsync");
	*(vaddr + DCFG_BRR) |=  1 << core_id;
	asm("lwsync");

	printf(" After StarCore release ========\n");
	printf("LCC_BSTRH=0x%x\n", *(vaddr + LCC_BSTRH));
	printf("LCC_BSTRL=0x%x\n", *(vaddr + LCC_BSTRL));
	printf("LCC_BSTAR=0x%x\n", *(vaddr + LCC_BSTAR));
	printf("GCR_CDCER0=0x%x\n", *(vaddr + GCR_CDCER0));
	printf("GCR_CHMER0=0x%x\n", *(vaddr + GCR_CHMER0));
	printf("DCFG_BRR=0x%x\n", *(vaddr + DCFG_BRR));

	unmap_area((void *)vaddr, size);

	printf("BSTRL,BSTAR,CDCERO,CHMERO n DCFG_BRR set now\n");
	return 0;
}

int check_dsp_boot_B4(void *dsp_bt)
{

	int hw_handshake_sem_no = ((dsp_bt_t *)dsp_bt)->semaphore_num;
	sys_map_t *het_sys_map = &((dsp_bt_t *)dsp_bt)->het_sys_map;
	uint64_t size = (*het_sys_map).dsp_ccsrbar.size;
	volatile uint32_t *vaddr = NULL;

	/*map to this value phys_addr = 0xffe000000*/
	uint64_t phys_addr = (*het_sys_map).dsp_ccsrbar.phys_addr;
	volatile int sem_val = 0;
	uint32_t hw_sem_offset;

	vaddr = map_area64(phys_addr, &size, dsp_bt);
	if (!vaddr) {
		printf("\nError in mapping physical address %#llx to virtual"
		       "address %p\n", phys_addr, vaddr);
		return -1;
	}

	if (hw_handshake_sem_no <= 7 && hw_handshake_sem_no >= 1) {
		hw_sem_offset = SC_BOOT_HW_SEMAPHORE0 + 2 * hw_handshake_sem_no;
		while (!sem_val) {
			sem_val = *(vaddr + hw_sem_offset);
			/* sleep for 1 sec*/
			puts("sleep 1");
			sleep(1);
		}

		printf("HS_MPR[%d]=0x%x\n", hw_handshake_sem_no, sem_val);
		if (sem_val == OS_HET_SC_SEMAPHORE_VAL) {
			*(vaddr + hw_sem_offset) = 0x00000000;
			asm("lwsync");
			printf("HS_MPR[%d]=0x%x\n",\
				hw_handshake_sem_no, *(vaddr + hw_sem_offset));
			printf("\n \n == DSP Booted up ==\n");
		}
	}

	return 0;
}

int pre_load_B4(int count, ...)
{
	int ret = 0;
	va_list arg_b;
	va_start(arg_b, count);
	void *dsp_bt = va_arg(arg_b, void*);

	((dsp_bt_t *)dsp_bt)->het_mgr = init_hetmgr();
	if (((dsp_bt_t *)dsp_bt)->het_mgr == -1) {
		printf("error in het_mgr frm %s\n", __func__);
		return -1;
	}

	((dsp_bt_t *)dsp_bt)->dev_mem = init_devmem();
	if (((dsp_bt_t *)dsp_bt)->dev_mem == -1) {
		printf("error in dev_mem frm %s\n", __func__);
		return -1;
	}

	ret = ioctl(((dsp_bt_t *)dsp_bt)->het_mgr,
		    IOCTL_HET_MGR_GET_SYS_MAP,
		    &((dsp_bt_t *)dsp_bt)->het_sys_map);
	if (ret) {
		perror("IOCTL_HET_MGR_GET_SYS_MAP:");
		printf("frm %s\n", __func__);
		return -1;
	}

	if (assign_memory_areas(dsp_bt)) {
		printf("error in assign_memory_areas frm %s\n", __func__);
		return -1;
	}

	/* get PA shared area and DSP shared AREA*/
	if (init_hugetlb(dsp_bt)) {
		printf("error in init_hugetlb frm %s\n", __func__);
		return -1;
	}

	if (fsl_B4_ipc_init(dsp_bt)) {
		printf("error in fsl_B4_ipc_init frm %s\n", __func__);
		return -1;
	}

	va_end(arg_b);
	return 0;
}

int load_B4(char *fname, void *dsp_bt)
{
	static int j;
	if (load_dsp_image(fname, dsp_bt)) {
		printf("Error in loading Dsp image StarCore\n");
		return -1;
	}

	if (j == 0) {
		if (set_sh_ctrl_pa_init(((dsp_bt_t *)dsp_bt)->het_mgr)) {
			printf("Error initialising PA Share"
				" CTRL StarCore\n");
			return -1;
		} else
			j++;
	}

	if (release_starcore_B4(dsp_bt)) {
		printf("Error in releasing StarCore\n");
		return -1;
	}

	return 0;
}

int post_load_B4(void *dsp_bt)
{
	return check_dsp_boot_B4(dsp_bt);
}

int b4860_load_dsp_image(int argc, char *argv[])
{

	int i = 0;
	void *dsp_bt;
	dsp_bt = (void *)malloc(sizeof(dsp_bt_t));

	((dsp_bt_t *)dsp_bt)->het_mgr = -1;
	((dsp_bt_t *)dsp_bt)->dev_mem = -1;

	/* Assign func Pointers*/
	((dsp_bt_t *)dsp_bt)->pre_load = &pre_load_B4;
	((dsp_bt_t *)dsp_bt)->load_image = &load_B4;
	((dsp_bt_t *)dsp_bt)->post_load = &post_load_B4;


	/* Call func Pointers*/
	if (((dsp_bt_t *)dsp_bt)->pre_load(1, dsp_bt) < 0)
		goto end_b4860_load_dsp_image;

	/* Load image and release StarCore */
	argc -= 1 ;
	i += 1;

	while (argc > 1) {
		((dsp_bt_t *)dsp_bt)->core_id = atoi(argv[i + 1]);

		if (((dsp_bt_t *)dsp_bt)->load_image(argv[i + 2], dsp_bt) < 0)
			goto end_b4860_load_dsp_image;

		argc -= 2;
		i += 2;
	}

	/* copy semaphore number*/
	((dsp_bt_t *)dsp_bt)->semaphore_num = atoi(argv[1]);
	if (((dsp_bt_t *)dsp_bt)->post_load(dsp_bt) < 0)
		goto end_b4860_load_dsp_image;
	else {
		cleanup(((dsp_bt_t *)dsp_bt)->dev_mem,
			((dsp_bt_t *)dsp_bt)->het_mgr);
		free(dsp_bt);
		return 0;
	}

end_b4860_load_dsp_image:
	cleanup(((dsp_bt_t *)dsp_bt)->dev_mem,
		((dsp_bt_t *)dsp_bt)->het_mgr);
	return -1;
}
#endif
