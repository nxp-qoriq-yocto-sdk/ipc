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
#include "fsl_usmmgr.h"
#include "fsl_bsc913x_ipc.h"
#include "fsl_ipc_errorcodes.h"
#define DSP_IMAGE_NAME "/dsp_images/vnmi15_9132_recovery_dbg.bin"
#define UIO_INTERFACE   "/dev/uio0"

#define ENTER()	printf(">> %s %d %s\n", __FILE__, __LINE__, __func__)
#define EXIT(A)	printf("<< (%d) %s %d %s\n", A, __FILE__, __LINE__, __func__)

fsl_usmmgr_t usmmgr;
fsl_ipc_t ipc;
void test_init(int rat_id);
int rat_id;

int main(int argc, char **argv)
{
	if (argc > 2) {
		printf("Usage:\n %s <rat_id>\n", argv[0]);
		printf("OR \nUsage:\n %s\n", argv[0]);
		goto end;
	}
	if (argc == 1)
		rat_id = 0;
	else if (argc == 2)
		rat_id = atoi(argv[1]);

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
	int ret = 0;
	mem_range_t sh_ctrl;
	mem_range_t dsp_ccsr;
	mem_range_t pa_ccsr;

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

	if (rat_id == 0) {
		/* use of fsl_ipc_init is deprecated
		* Instead use fsl_ipc_init_rat with rat_id 0,
		* for non-MULTI RAT scenarios*/
		ipc = fsl_ipc_init(
			test_p2v, sh_ctrl, dsp_ccsr, pa_ccsr, UIO_INTERFACE);
	} else {
		ipc = fsl_ipc_init_rat(
			rat_id,
			test_p2v, sh_ctrl, dsp_ccsr, pa_ccsr, UIO_INTERFACE);
	}

	if (!ipc) {
		printf("Issue with fsl_ipc_init %d\n", ret);
		return;
	}
restart:
	fsl_restart_L1(ipc, DSP_IMAGE_NAME);
	puts("sleep 5 sec");
	sleep(5);
	goto restart;
}
