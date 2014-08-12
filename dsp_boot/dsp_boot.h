/**
 ** @dsp_boot.h
 **
 ** Copyright (c) 2011-2013, Freescale Semiconductor Inc.
 **
 ** Redistribution and use in source and binary forms, with or without
 ** modification, are permitted provided that the following conditions
 ** are met:
 ** 1. Redistributions of source code must retain the above copyright
 **    notice, this list of conditions and the following disclaimer.
 ** 2. Redistributions in binary form must reproduce the above copyright
 **    notice, this list of conditions and the following disclaimer in the
 **    documentation and/or other materials provided with the distribution.
 ** 3. Neither the name of Freescale Semiconductor Inc nor the names of its
 **    contributors may be used to endorse or promote products derived from
 **    this software without specific prior written permission.
 **
 ** THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND
 ** ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 ** IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 ** ARE DISCLAIMED.  IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE
 ** FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 ** DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 ** OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 ** HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 ** LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 ** OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 ** SUCH DAMAGE.
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

#ifdef DEBUG_L1D
#define l1d_printf(...)	printf(__VA_ARGS__);
#else
#define l1d_printf(...)
#endif

typedef void *fsl_ipc_t;

typedef struct{
	int het_mgr;
	int dev_mem;
	int fsl_l1d;
	int map_id;
	sys_map_t het_sys_map;
	mem_range_t map_d[20];
	mem_range_t sh_ctrl_area;
	mem_range_t os_het_ipc_t;
	uint64_t intvec_addr;
	uint32_t core_id;
	uint32_t semaphore_num;

	int (*pre_load)(int, ...);
	int (*load_image)(char *, void *);
	int (*post_load)(void *);
} dsp_bt_t;

typedef struct{
	int core_id;
	char *image_name;
} dspbt_core_info;

typedef struct{
	int reset_core_flag;
	int core_id;
	char *dsp_filename;
	uint32_t wpt_begin_addr;
	uint32_t wpt_end_addr;
	char wpt_type;
} reload_dsp_core_info;

typedef struct{
	int hw_sem_num;
	int reset_mode;
	int maple_reset_mode;
	int debug_print;
	int cfg_wpt;
	reload_dsp_core_info shDspCoreInfo[6];
	reload_dsp_core_info reDspCoreInfo[6];
} dsp_core_info;


int send_vnmi_func();
int fsl_ipc_reinit(fsl_ipc_t ipc);
int fsl_B4_ipc_init(void *);
int fsl_913x_ipc_init(void *);
int b913x_load_dsp_image(char *);
int b4860_load_dsp_image(int , dspbt_core_info []);
int fsl_restart_L1(fsl_ipc_t, char*);
int fsl_start_L1_defense(fsl_ipc_t , dsp_core_info *);
int fsl_B4_ipc_reinit(fsl_ipc_t , void *);
int check_validation_fields(void *);
#define DSP_BOOT_SUCCESS 2
#define ERR_L1_DEFENSE_API_FAIL 30
#define SIZE_1MB 0x10000

#endif
