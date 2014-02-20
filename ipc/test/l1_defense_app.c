/*
 * Copyright 2013 Freescale Semiconductor, Inc.
 *
 * Author: Ashish Kumar <ashish.kumar@freescale.com>
 */
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include "fsl_usmmgr.h"
#include "fsl_bsc913x_ipc.h"
#include "fsl_ipc_errorcodes.h"
#include "fsl_heterogeneous_l1_defense.h"
#define DSP_IMAGE_NAME "/dsp_images/vnmi15_9132_recovery_dbg.bin"
#define UIO_INTERFACE   "/dev/uio0"

static int dsp_wsrsr_core_mask[6] = {0x0010000, 0x00040000,
				0x00100000, 0x00400000,
				0x01000000, 0x04000000};

static char dsp_image_name[][2000] = {"/ipc/c0.bin", "/ipc/c1.bin",
					"/ipc/c2.bin", "/ipc/c3.bin",
					"/ipc/c4.bin", "/ipc/c5.bin"};

static char shared_image_name[][2000] = {"/ipc/sh0.bin", "/ipc/sh1.bin",
					"/ipc/sh2.bin", "/ipc/sh3.bin"};
#define ENTER()	printf(">> %s %d %s\n", __FILE__, __LINE__, __func__)
#define EXIT(A)	printf("<< (%d) %s %d %s\n", A, __FILE__, __LINE__, __func__)

fsl_usmmgr_t usmmgr;
fsl_ipc_t ipc;
int rat_id;
int ipc_in_use;
static uint32_t core_mask;
static uint32_t nr_dsp_core;
dsp_core_info *DspCoreInfo;

static void usage(char arg[30])
{
	printf("Invalid parameter in %s\n", arg);
	exit(EXIT_FAILURE);
}

static void test_init(int rat_id);

void l1d_callback(uint32_t core_mask1)
{
	int i;
	core_mask = core_mask1;
	for (i = 0; i < nr_dsp_core; i++) {
		if (core_mask & dsp_wsrsr_core_mask[i])
			DspCoreInfo->reDspCoreInfo[i].reset_core_flag = 1;
		else
			DspCoreInfo->reDspCoreInfo[i].reset_core_flag = 0;

		if (rat_id != 0 && (
		(DspCoreInfo->reset_mode == MODE_3_ACTIVE) ||
		(DspCoreInfo->reset_mode == MODE_2_ACTIVE)))
			DspCoreInfo->reDspCoreInfo[i].reset_core_flag = 1;

	}

	if (core_mask != 0) {
		if (ipc_in_use)
			fsl_start_L1_defense(ipc, DspCoreInfo);
		else
			fsl_start_L1_defense(NULL, DspCoreInfo);

		puts("sleep 5 sec");
		sleep(5);

	}
	return;
}
int main(int argc, char **argv)
{
	if (argc > 3 || argc == 2) {
		printf("Usage:\n %s <single core/multi core> <ipc in use>\n",
				argv[0]);
		printf("Where as,\n 0: single core\n 1: multi core\n");
		printf(" 0: IPC not used\n 1: IPC used\n\n");
		printf("OR \nUsage:\n %s\n", argv[0]);
		printf("Where as,\n \"%s\" means <single core> <ipc not"
				" used>\n", argv[0]);
		printf(" Same as \"%s 0 0\"\n", argv[0]);
		goto end;
	}
	if (argc == 1) {
		rat_id = 0;
		ipc_in_use = 0;
		printf("\nOnly 1 argument means <single core> and "
				"<IPC not used>\n");
	} else if (argc == 3)
		rat_id = atoi(argv[1]);
		ipc_in_use = atoi(argv[2]);

	test_init(rat_id);
end:
	return 0;
}

void *test_p2v(unsigned long phys_addr)
{
	return fsl_usmmgr_p2v(phys_addr, usmmgr);
}

void test_init(int rat_id)
{
	int ret = 0, i = 0;
	uint32_t warm_reset_mode = 3 , maple_reset_mode = 0;
	uint32_t hw_sem = 0, debug_print = 0;
	uint32_t nr_sh = 0;
	mem_range_t sh_ctrl;
	mem_range_t dsp_ccsr;
	mem_range_t pa_ccsr;

	DspCoreInfo = (dsp_core_info *)malloc(sizeof(dsp_core_info));

	printf("\n=========$DSP RECOVERY N RELOAD$====%s %s====\n",
			__DATE__, __TIME__);

	usmmgr = fsl_usmmgr_init();
	if (!usmmgr) {
		printf("Error in Initializing User Space Memory Manager\n");
		return;
	}

	/* get values from mmgr */
	ret = get_pa_ccsr_area(&pa_ccsr, usmmgr);
	if (ret) {
		printf("Error in obtaining PA CCSR Area information\n");
		return;
	}

	ret = get_dsp_ccsr_area(&dsp_ccsr, usmmgr);
	if (ret) {
		printf("Error in obtaining DSP CCSR Area information\n");
		return;
	}

	ret = get_shared_ctrl_area(&sh_ctrl, usmmgr);
	if (ret) {
		printf("Error in obtaining Shared Control Area information\n");
		return;
	}

	if (rat_id == 0 && ipc_in_use == 1) {
		/* use of fsl_ipc_init is deprecated
		* Instead use fsl_ipc_init_rat with rat_id 0,
		* for non-MULTI RAT scenarios*/
		ipc = fsl_ipc_init(
			test_p2v, sh_ctrl, dsp_ccsr, pa_ccsr, UIO_INTERFACE);
	} else if (ipc_in_use == 1) {
		ipc = fsl_ipc_init_rat(
			rat_id,
			test_p2v, sh_ctrl, dsp_ccsr, pa_ccsr, UIO_INTERFACE);
	}

	if (!ipc && (ipc_in_use == 1)) {
		printf("Issue with fsl_ipc_init %d\n", ret);
		return;
	}

	for (i = 0; i <= 5; i++) {
		DspCoreInfo->reDspCoreInfo[i].dsp_filename = malloc(2000);
		DspCoreInfo->reDspCoreInfo[i].dsp_filename =
					(dsp_image_name[i]);
		DspCoreInfo->reDspCoreInfo[i].core_id = i;
	}
	puts("Enter your choice");
	puts(" 0 means not in use for all Parameters"
			"\n Only Values mentioned below are valid,"
			" rest all values are invalid\n");
	puts("WARM_RESET_MODE <1 or 2 or 3> Enter value as <0x1, 0x2, 0x4>"
			"\nMAPLE_RESET_MODE <0x0,0x2,0x4"
			",0x8,0x6,0xA,0xC,0xE>\n"
			"Debug_print <0x0,0x1>\nHW_SEM_NUM <0x0,0x1,"
			"0x2,0x3,0x4,0x5,0x6,0x7>\n"
			"Number of Shared images <0x0,0x1,0x2,0x3,0x4>");
	scanf("%x %x %x %x %x", &warm_reset_mode, &maple_reset_mode,
		&debug_print, &hw_sem, &nr_sh);

	if (!(warm_reset_mode == MODE_1_ACTIVE ||
	    warm_reset_mode == MODE_2_ACTIVE ||
	    warm_reset_mode == MODE_3_ACTIVE))
		usage("warm_reset_mode");

	if ((ipc_in_use == 1) && (rat_id == 1)) {
		puts("\nNR_DSP_CORE <0x2,0x6>");
		scanf("%x", &nr_dsp_core);
	} else
		nr_dsp_core = 6;

	for (i = 0; i < nr_sh; i++) {
		DspCoreInfo->shDspCoreInfo[i].reset_core_flag = 1;
		DspCoreInfo->shDspCoreInfo[i].dsp_filename = malloc(2000);
		DspCoreInfo->shDspCoreInfo[i].dsp_filename =
					(shared_image_name[i]);
		DspCoreInfo->shDspCoreInfo[i].core_id = -1;
	}


	DspCoreInfo->reset_mode = warm_reset_mode;
	DspCoreInfo->maple_reset_mode = maple_reset_mode;
	DspCoreInfo->debug_print = debug_print;
	DspCoreInfo->hw_sem_num = hw_sem;

	fsl_L1_defense_register_cb(l1d_callback);

	while (1)
		;

	return;
}
