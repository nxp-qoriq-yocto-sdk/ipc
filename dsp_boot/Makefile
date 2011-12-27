CC=$(CROSS_COMPILE)gcc
HET_DIR=$(KERNEL_DIR)/drivers/misc
FSL_SHM_DIR=../fsl_shm/include
SOURCES= ./dsp_boot.c ../fsl_shm/lib/lg_shm.c
CFLAGS= -I $(HET_DIR) -I $(FSL_SHM_DIR) -Wall

dsp_bt:
	$(CC) $(CFLAGS) $(SOURCES)  -o $@

clean:
	rm -rf *.o dsp_bt
