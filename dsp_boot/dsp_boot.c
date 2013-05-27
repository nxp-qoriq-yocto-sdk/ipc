/*
 * @file: dsp_boot.c
 *
 * Copyright 2011-2013 Freescale Semiconductor, Inc.
 *
 * Author: Ashish Kumar <ashish.kumar@freescale.com>
 * Author: Manish Jaggi <manish.jaggi@freescale.com>
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
#include <errno.h>
#include <signal.h>
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <time.h>
#include "fsl_het_mgr.h"
#include "fsl_ipc_types.h"
#include "fsl_ipc_shm.h"
#include "dsp_boot.h"
#include "dsp_compact.h"
#define VERSION 	"2.0.0"

int main(int argc, char *argv[])
{
#if B913x
	printf("===DSP boot Application===(%s)==\n", VERSION);
	if (argc < 2) {
		printf("Usage:\n dsp_boot <fname>\n");
		exit(-1);
	}

	b913x_load_dsp_image(argv[1]);
#endif

#if B4860
	printf("===B4860QDS DSP boot Application (%s) ===\n", VERSION);
	if ((argc % 2) || argc > 14) {
		printf("Usage: dsp_bt <PA_SC_Handshake_Sem> <core>"
				" <fname>\n");
		exit(-1);
	}
	b4860_load_dsp_image(argc, argv);
#endif
	return 0;
}
