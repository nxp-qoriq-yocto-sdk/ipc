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
#include <ctype.h>
#include <stdint.h>
#include <time.h>
#include "fsl_het_mgr.h"
#include "fsl_ipc_types.h"
#include "fsl_ipc_shm.h"
#include "dsp_boot.h"
#include "dsp_compact.h"
#if B913x
#define VERSION 	"2.0.0"
#else
#define VERSION 	"3.1.0"
#define CMD_LINE_OPT "i:c:s:h:d:"
#define HW_SEM "hw_sem"

void usage(char **argv)
{
	printf("Usage: %s -h <hw_sem> -c <core_id> -i <\"image_name\">"
		" -d <\"DDR Controller ID\">"
		" -s <\"shared_image\">\n",	argv[0]);
	printf("whereas,\n <hw_sem> : 1 to 7 for use hardware semaphore\n"
		"          : 0 No hardware semaphore used\n"
		" <core_id> : 0 to 5 dsp core number \n"
		" <image_name> : dsp image name\n"
		" 		 -c option should be followed by -i option\n"
		" <DDR Controller ID> : 0x10, 0x11, is an optional parameter \n"
		" <shared_image> : Shared Image name \n");
	exit(EXIT_FAILURE);
}

void bubble_sort(int argc, dspbt_core_info *CoreInfo2)
{
	dspbt_core_info tmp;
	tmp.image_name = malloc(500);
	int i, j;

	for (i = 0 ; i < argc; i++) {
		for (j = 1; j < (argc - i - 1); j++) {
			if (CoreInfo2[j].core_id > CoreInfo2[j + 1].core_id) {
				memcpy((void *)&tmp, (void *)&CoreInfo2[j],
						sizeof(dspbt_core_info));
				memcpy(&CoreInfo2[j], &CoreInfo2[j+1],
					sizeof(dspbt_core_info));
				memcpy(&CoreInfo2[j+1], (void *)&tmp,
					sizeof(dspbt_core_info));
			}
		}
	}
	return;
}
#endif


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
	int opt;
	dspbt_core_info *CoreInfo;
	CoreInfo = malloc(20*sizeof(dspbt_core_info));
	int i = 1, cnt_c = 0, cnt_i = 0, k = 0;
	char *endptr;

	CoreInfo[0].core_id = 0;
	CoreInfo[0].DDRC_trg_id = DDRC1_TRG_ID;
	CoreInfo[0].image_name = malloc(200);
	memcpy(CoreInfo[0].image_name, HW_SEM, sizeof(HW_SEM));

	while ((opt = getopt(argc, argv, CMD_LINE_OPT)) != -1) {
		switch (opt) {
		case 'c':
			if (isdigit(optarg[0])) {
				CoreInfo[i].core_id = atoi(optarg);
				cnt_c++;
				break;
			} else
				usage(argv);
		case 'i':
			((CoreInfo + i)->image_name) = malloc(500);
			memcpy(CoreInfo[i].image_name, optarg, strlen(optarg));
			cnt_i++;
			i++;
			if (cnt_c != cnt_i)
				usage(argv);
			else
				break;
		case 's':
			CoreInfo[i].core_id = -1;
			((CoreInfo + i)->image_name) = malloc(500);
			memcpy(CoreInfo[i].image_name, optarg, strlen(optarg));
			i++;
			break;
		case 'd':
			errno = 0;
			CoreInfo[0].DDRC_trg_id = strtoul(optarg, &endptr, 16);

			if (errno != 0) {
				perror("strtol");
				goto err;
			}
			if (endptr == optarg) {
				printf("No digits were found\n");
				goto err;
			}
			if (*endptr != '\0') {
				printf("Further characters after number\n");
				goto err;
			}
			break;
err:
			printf("Setting default DDRC target ID\n");
			CoreInfo[0].DDRC_trg_id = DDRC1_TRG_ID;
			printf("DDRC Trg ID=%#x\n", CoreInfo[0].DDRC_trg_id);
			break;
		case 'h':
			if (isdigit(optarg[0])) {
				CoreInfo[0].core_id = atoi(optarg);
				break;
			} else
				usage(argv);
		default:
			usage(argv);
		}

	}

	if (optind == 1 || (cnt_c != cnt_i))
		usage(argv);
	else {
		l1d_printf("i = %i cnt_i=%d cnt_c=%d\n", i, cnt_i, cnt_c);
		for (k = 0; k < i; k++)
			l1d_printf("i=%i id=%i name=%s \n", i,
				CoreInfo[k].core_id, CoreInfo[k].image_name);
		bubble_sort(i, CoreInfo);
		l1d_printf("Sorted now\n");
		for (k = 0; k < i; k++)
			l1d_printf("i=%i id=%i name=%s \n", i,
				CoreInfo[k].core_id, CoreInfo[k].image_name);
		/* End of Core specific images marked with -1 core ID */
		b4860_load_dsp_image(i, CoreInfo);
	}
#endif
	return 0;
}
