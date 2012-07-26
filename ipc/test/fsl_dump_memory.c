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

#define SIZE 0x4000

int main()
{
	int i, status = 0;
	void *d_buf = malloc(400000);
	if (d_buf == NULL) {
		printf("malloc fail\n");
		exit(-1);
	}

	FILE *fd = fopen("Memory_dumped.txt", "w");
	if (fd == NULL) {
		printf("dumped_memory.txt open fail\n");
		exit(-1);
	}

	status = fsl_usmmgr_dump_memory(d_buf, SIZE);

	if (status < 0) {
		fprintf(fd , "Error dump_memory_dsp status is (%i) 0x%x \n",
			status, status);
	} else {
		unsigned long *dbuff = (unsigned long *)d_buf;
		fprintf(fd, "dump_memory_dsp status is (%i) 0x%x\n",
			status, status);

		fprintf(fd, "%s", "\n");

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
	return 0;
}
