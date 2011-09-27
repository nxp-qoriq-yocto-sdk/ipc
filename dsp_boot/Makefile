CC=$(CROSS_COMPILE)gcc
HET_DIR=$(KERNEL_DIR)/drivers/misc
SOURCES= ./dsp_boot.c
CFLAGS= -I $(HET_DIR)

dsp_bt:
	$(CC) $(CFLAGS) $(SOURCES)  -o $@

clean:
	rm -rf *.o dsp_bt
