CC=$(CROSS_COMPILE)gcc
HET_DIR=$(KERNEL_DIR)/drivers/misc
FSL_SHM_DIR=../fsl_shm/include
SOURCES= ./dsp_boot.c
SOURCES_FSL_SHM= ../fsl_shm/lib/lg_shm.c
CFLAGS= -I $(HET_DIR) -I $(FSL_SHM_DIR) -Wall

all: fsl_shm lib_dsp_boot dsp_bt
lib_dsp_boot:
	$(CC) -c ./libdspboot.c $(CFLAGS) -o ./libdspboot.o
	$(CC) -shared -fPIC ./libdspboot.o -o libdspboot.so
fsl_shm:
	$(CC) -c $(CFLAGS) $(SOURCES_FSL_SHM)
dsp_bt:
	$(CC) $(CFLAGS) $(SOURCES) $(SOURCES_FSL_SHM) -L. -ldspboot -L../ipc -lipc -o $@
	cp libdspboot.so  ../ipc/ -v
clean:
	rm -rf *.o dsp_bt libdspboot.so
