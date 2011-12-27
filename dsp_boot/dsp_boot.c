/*
 * @file: dsp_boot.c
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

/*
 * DSP Image Binary Format
 *
 * ENDIANNESS_BYTE (1Byte)
 * <ADDRESS - 4Bytes><SIZE_IN_BYTES - 4Bytes><data_payload>
 * <ADDRESS - 4Bytes><SIZE_IN_BYTES - 4Bytes><data_payload>
 * ....
 * <ADDRESS - 4Bytes><SIZE_IN_BYTES - 4Bytes><data_payload>
 * <ADDRESS = START_ADDRESS0_ADDRESS><SIZE = 4><value_of_entrypoint>
 * <ADDRESS = START_ADDRESS0_ADDRESS><SIZE = 4><value_of_entrypoint>
 * NOTE ENDIANNESS_BYTE has the value 2 for MSB and 1 for LSB. This feature can
 * be used to dump the executable .elf file into a fast download format. File
 * can be parsed and loaded into target’s memory.
 *
 */
#include <stdio.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/shm.h>
#include <sys/ioctl.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <time.h>
#include "fsl_het_mgr.h"
#include "fsl_types.h"
#include "fsl_ipc_shm.h"

#define VERSION 	"1.1"
/* defines */
#define MAX_ENTRIES	10
/* PA CCSR */
#define	DSPSR	0xE00D8
#define DSPSR_DSP1_READY 0x80000000
#define DSPSR_DSP2_READY 0x40000000

/* DSP CCSR */
#define DSP_GCR	0x18000
#define PASTATE	0x104
#define PASTATE_PAREADY	0x1

#define MB_256		0x10000000
#define IPC_SHMEM	MB_256

#define SMAP(I, A, B, C)	\
			do {	\
				map[I].phys_addr = A;	\
				map[I].vaddr = B; \
				map[I].size = C; \
			} while (0);

#define MMAP(P, SZ)	\
			do {	\
				vaddr = mmap(0, SZ, (PROT_READ | \
						PROT_WRITE), MAP_SHARED,\
						dev_mem, P);	\
				SMAP(mapidx, P, vaddr, SZ);\
				mapidx++;	\
			} while (0);

#define MUNMAP(P, SZ)	munmap((void *)(P), (SZ));

/*
 * Global Variables
 *
 */

int dev_mem;
int dev_het_mgr;
sys_map_t het_sys_map;

range_t map[MAX_ENTRIES];
range_t map[MAX_ENTRIES];
int mapidx;

int shmid;
shared_area_t shared_area;
struct sigaction act;

void cleanup();

void dump_sys_map()
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
#ifdef PSC9132
	printf("DSP Core1 M2: Addr=%x Size=%x\n",
		het_sys_map.dsp_core1_m2.phys_addr,
		het_sys_map.dsp_core1_m2.size);

	printf("DSP M3: Addr=%x Size=%x\n",
		het_sys_map.dsp_m3.phys_addr,
		het_sys_map.dsp_m3.size);
#endif
	printf("PA CCSRBAR: Addr =%lx Size=%x\n",
		het_sys_map.pa_ccsrbar.phys_addr,
		het_sys_map.pa_ccsrbar.size);

	printf("DSP CCSRBAR: Addr =%lx Size=%x\n",
		het_sys_map.dsp_ccsrbar.phys_addr,
		het_sys_map.dsp_ccsrbar.size);
}

/*
 * @p2v
 *
 */
void *p2v(phys_addr_t phys_addr)
{
	int i;
	for (i = 0; i < mapidx; i++) {
		if (phys_addr >= map[i].phys_addr &&
		    phys_addr < map[i].phys_addr + map[i].size) {
			return (void *)(phys_addr - map[i].phys_addr) +
			    (uint32_t) map[i].vaddr;
		}
	}
	return NULL;
}

/*
 * @get_hw_sem_value
 *
 */
static inline int get_hw_sem_value(uint32_t *val)
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
static inline int set_hw_sem_value(uint32_t val)
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
static inline int get_uniq_hw_sem_value(hw_sem_info_t *hseminfo)
{
	return ioctl(dev_het_mgr, IOCTL_HW_SEM_GET_UVALUE, hseminfo);
}

void check_dsp_boot()
{
	uint32_t val;
	int ret;

	hw_sem_info_t hwu;

	ret = get_uniq_hw_sem_value(&hwu);
	if (ret)
		goto end;
	ret = get_hw_sem_value(&val);
	if (ret)
		goto end;
	if (val == hwu.dsp_uniq_val) {
		val = 0;
		set_hw_sem_value(val);
		printf("\n == DSP Booted up == \n");
	} else {
		printf("\n DSP HW Sem1 value not correctly set, value=%x\n",
		       val);
		printf("\n DSP Boot Failed, Please reset\n");
	}
end:
	/* in the end call this */
	cleanup();
	if (ret)
		printf("Error in loading DSP\n");
	exit(1);
}

/*
 * @copy_file_part
 *
 * simple byte copy from file
 */
int copy_file_part(void *virt_addr, uint32_t size, FILE * fp)
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

int init_devmem()
{
	int ret = 0;
	dev_mem = open("/dev/mem", O_RDWR);
	if (dev_mem == -1) {
		printf("Error: Cannot open /dev/mem.\n");
		ret = -1;
	}
	return ret;
}

int init_hetmgr()
{
	int ret = 0;
	/* query ranges from /dev/het_mgr */
	dev_het_mgr = open("/dev/het_mgr", O_RDWR);
	if (dev_het_mgr == -1) {
		printf("Error: Cannot open /dev/het_mgr\n");
		ret = -1;
	}
	return ret;
}

int map_memory_areas()
{
	int ret = 0;
	void *vaddr;
	/* open /dev/mem
	 * map dsp m2/m3/ddr
	 */
	/* Send IOCTL to get system map */
	ret = ioctl(dev_het_mgr, IOCTL_HET_MGR_GET_SYS_MAP, &het_sys_map);
	if (ret)
		return ret;

	dump_sys_map();

	if (het_sys_map.smart_dsp_os_priv_area.phys_addr == 0xffffffff ||
	het_sys_map.dsp_core0_m2.phys_addr == 0xffffffff ||
	het_sys_map.dsp_core1_m2.phys_addr == 0xffffffff ||
	het_sys_map.dsp_m3.phys_addr == 0xffffffff ||
	het_sys_map.dsp_shared_size == 0xffffffff) {
		printf("Incorrect params\n");
		return -1;
	}

	/* MAP DSP private area in ddr */
	MMAP(het_sys_map.smart_dsp_os_priv_area.phys_addr,
	     het_sys_map.smart_dsp_os_priv_area.size);

	if (vaddr == MAP_FAILED)
		return -1;

	MMAP(het_sys_map.dsp_core0_m2.phys_addr, het_sys_map.dsp_core0_m2.size);
	if (vaddr == MAP_FAILED)
		return -1;
#ifdef PSC9132
	MMAP(het_sys_map.dsp_core1_m2.phys_addr, het_sys_map.dsp_core1_m2.size);
	if (vaddr == MAP_FAILED)
		return -1;

	MMAP(het_sys_map.dsp_m3.phys_addr, het_sys_map.dsp_m3.size);
	if (vaddr == MAP_FAILED)
		return -1;
#endif
	MMAP(het_sys_map.pa_ccsrbar.phys_addr, het_sys_map.pa_ccsrbar.size);
	if (vaddr == MAP_FAILED)
		return -1;

	MMAP(het_sys_map.dsp_ccsrbar.phys_addr, het_sys_map.dsp_ccsrbar.size);
	if (vaddr == MAP_FAILED)
		return -1;

	return ret;
}

int load_dsp_image(char *fname)
{
	/* Read First 4 bytes */
	/* Read phys address and size */
	/* get vaddr from phys addr */
	/* call copy part */

	FILE *dspbin;
	uint8_t endian;
	uint32_t addr, size;
	void *vaddr;
	int ret = 0;
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
		if (!ret)
			break;
		ret = fread(&size, 4, 1, dspbin);

		if (!ret)
			break;

		vaddr = p2v(addr);
		if (!vaddr) {
			ret = -1;
			printf("\n Error in translating physical address %#x"
			       " to virtual address\n", addr);
			goto end_close_file;
		}

		ret = copy_file_part(vaddr, size, dspbin);
		if (ret)
			goto end_close_file;
	}

end_close_file:
	fclose(dspbin);
end:
	return ret;
}

static inline int dsp_ready_set()
{
	volatile uint32_t *vaddr;
	int ret;
	uint32_t val;
	ret = 0;
	vaddr = p2v(het_sys_map.pa_ccsrbar.phys_addr + DSPSR);
	if (!vaddr) {
		printf("\nError in translating physical address %lx to virtual"
		       "address\n", het_sys_map.pa_ccsrbar.phys_addr + DSPSR);
		return -1;
	}

	val = *vaddr;
	if (val & DSPSR_DSP1_READY) {
		printf("DSP READY SET\n");
		return ret;
	}
	return -1;
}

int set_sh_ctrl_pa_init()
{
	return ioctl(dev_het_mgr, IOCTL_HET_MGR_SET_INITIALIZED, 0);
}

static inline void set_ppc_ready()
{
	uint32_t *vaddr;

	/* write to ppc ready */
	vaddr = p2v(het_sys_map.dsp_ccsrbar.phys_addr + DSP_GCR + PASTATE);
	if (!vaddr) {
		printf("\nError in translating physical address %lx to virtual"
		       "address\n",
		       het_sys_map.dsp_ccsrbar.phys_addr + DSP_GCR + PASTATE);
		return;
	}

	*vaddr |= PASTATE_PAREADY;
}

static inline int init_hugetlb(void)
{
	int ret;
	void *pa_p;
	pa_p = (void *)fsl_shm_init(het_sys_map.dsp_shared_size);
	if (!pa_p) {
		ret = -1;
		goto end;
	}

	shared_area.pa_ipc_shared.phys_addr = (uint32_t)pa_p;
	shared_area.pa_ipc_shared.size = MB_256
				- het_sys_map.dsp_shared_size;

	shared_area.dsp_ipc_shared.phys_addr = (uint32_t)pa_p + MB_256
					- het_sys_map.dsp_shared_size;
	shared_area.dsp_ipc_shared.size = het_sys_map.dsp_shared_size;

	printf("PA Shared Area: Addr=%lx Size=%x\n",
		shared_area.pa_ipc_shared.phys_addr,
		shared_area.pa_ipc_shared.size);

	printf("DSP Shared Area: Addr=%lx Size=%x\n",
		shared_area.dsp_ipc_shared.phys_addr,
		shared_area.dsp_ipc_shared.size);

	/* attach the physical address with shared mem */
	ret = ioctl(dev_het_mgr, IOCTL_HET_MGR_SET_SHARED_AREA, &shared_area);
	if (ret) {
		printf("ioctl IOCTL_HET_MGR_SET_SHARED failed\n");
		ret = -1;
		goto end;
	}

end:
	return ret;
}

int main(int argc, char **argv)
{
	int ret = 0;
	mapidx = 0;
	printf("===DSP boot Application===(%s)==\n", VERSION);
	if (argc < 2) {
		printf("Usage:\n dsp_boot <fname>\n");
		goto end;
	}

	if (init_hetmgr())
		goto end;

	if (init_devmem())
		goto end;

	if (map_memory_areas())
		goto end;

	if (init_hugetlb())
		goto end;

	if (dsp_ready_set())
		goto end;

	if (load_dsp_image(argv[1]))
		goto end;

	if (set_sh_ctrl_pa_init())
		goto end;

	set_ppc_ready();

	/* sleep for 1 sec */
	sleep(1);

	check_dsp_boot();
	do {
	} while (1);
end:
	cleanup();
	/*TBD: add a get_error_string(ret) */
	printf("Error in loading Dsp image Error %d", ret);
	return ret;
}

void unmap_memory_areas()
{
	MUNMAP(het_sys_map.smart_dsp_os_priv_area.phys_addr,
	       het_sys_map.smart_dsp_os_priv_area.size);

	MUNMAP(het_sys_map.dsp_core0_m2.phys_addr,
	       het_sys_map.dsp_core0_m2.size);
#ifdef PSC9132
	MUNMAP(het_sys_map.dsp_core1_m2.phys_addr,
	       het_sys_map.dsp_core1_m2.size);

	MUNMAP(het_sys_map.dsp_m3.phys_addr, het_sys_map.dsp_m3.size);
#endif
	MUNMAP(het_sys_map.pa_ccsrbar.phys_addr, het_sys_map.pa_ccsrbar.size);
	MUNMAP(het_sys_map.dsp_ccsrbar.phys_addr, het_sys_map.dsp_ccsrbar.size);
}

void cleanup()
{
	unmap_memory_areas();
	close(dev_mem);
	close(dev_het_mgr);
}
