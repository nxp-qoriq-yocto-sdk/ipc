/* *
 * * Copyright 2011-2012 Freescale Semiconductor, Inc.
 * *
 * * Author: Ashish Kumar <Ashish.kumar@freescale.com>
 * */

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include "fsl_usmmgr.h"

#define OUTPUT_TEXT_FILE	"Memory_dumped.txt"
#define OUTPUT_BINARY_TXT	"Memory_dumped.dat"

int main(int arg_c, char *argv[])
{
	if (arg_c != 2) {
		printf("Usage:\"./dump_mem <des_buf_size>\"\n");
		printf("des_buf_size is the Max size of destination buffer\n");
		exit(-1);
	}

	int i, status = 0;
	int size = atoll(argv[1]);
	printf("size = %d\n", size);

	void *d_buf = malloc(size);
	if (d_buf == NULL) {
		printf("malloc fail\n");
		printf("Use des_buf_size less than %d \n", size);
		exit(-1);
	}

	FILE *fd = fopen(OUTPUT_TEXT_FILE, "w");
	if (fd == NULL) {
		printf("%s open fail\n", OUTPUT_TEXT_FILE);
		exit(-1);
	}
	FILE *fd_b = fopen(OUTPUT_BINARY_TXT, "wb");
	if (fd == NULL) {
		printf("%s  open fail\n", OUTPUT_BINARY_TXT);
		exit(-1);
	}

	status = fsl_usmmgr_dump_memory(d_buf, size);

	if (status < 0) {
		fprintf(fd , "Error dump_memory_dsp status is (%i) 0x%x \n",
			status, status);
	} else {
		unsigned long *dbuff = (unsigned long *)d_buf;
		fprintf(fd, "dump_memory_dsp status is (%i) 0x%x\n",
			status, status);

		fprintf(fd, "%s", "\n");

		fwrite(d_buf, 1, status, fd_b);

		for (i = 0 ; i < status/4; i++) {
			fprintf(fd, "  %08lx", dbuff[i]);
			if (((i+1)%4 == 0))
				fprintf(fd, "%s", "\n");

			if (((i+1)%16 == 0))
				fprintf(fd, "%s", "\n");

		}
	}

	free(d_buf);
	fclose(fd);
	fclose(fd_b);
	return 0;
}
