/**
 ** @dsp_boot.h
 **
 ** Copyright 2013 Freescale Semiconductor, Inc.
 **
 ** Author: Ashish Kumar <ashish.kumar@freescale.com>
 **/
#ifndef __DSP_BOOT_H
#define __DSP_BOOT_H

#ifdef DEBUG_RELOAD
#define reload_print(...)  printf(__VA_ARGS__);
#else
#define reload_print(...)
#endif
typedef void *fsl_ipc_t;

typedef struct{
	int het_mgr;
	int dev_mem;
	int map_id;
	sys_map_t het_sys_map;
	mem_range_t map_d[20];
	uint64_t intvec_addr;
	uint32_t core_id;
	uint32_t semaphore_num;

	int (*pre_load)(int, ...);
	int (*load_image)(char *, void *);
	int (*post_load)(void *);
} dsp_bt_t;

int send_vnmi_func();
int fsl_ipc_reinit(fsl_ipc_t *);
int b913x_load_dsp_image(char *);
int b4860_load_dsp_image(int , char **);
int fsl_restart_L1(fsl_ipc_t, char*);
#define DSP_BOOT_SUCCESS 2
#endif
