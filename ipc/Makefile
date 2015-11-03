CC=$(CROSS_COMPILE)gcc
AR=$(CROSS_COMPILE)ar
KERNEL_DIR1=$(PWD)/kernel
HET_DIR=$(KERNEL_DIR1)
CFLAGS=-fPIC -O3 -lpthread -I ./include -I ../fsl_shm/include/ -I ../fsl_shm/lib/ -I ../dsp_boot/ -I $(HET_DIR) -D_FILE_OFFSET_BITS=64 -D_LARGEFILE_SOURCE -Wall
CFLAGS2=-fPIC -I ./include -I ../fsl_shm/lib/  -I ../fsl_shm/include/ -I ../dsp_boot/ -I $(HET_DIR) -D_FILE_OFFSET_BITS=64 -D_LARGEFILE_SOURCE -Wall

ifeq ($(CONFIG_LOCK), 1)
	CFLAGS+=-DCONFIG_LOCK
endif

ifeq ($(B4860), 1)
  CFLAGS2+=-DB4860
  CFLAGS+=-DB4860
ifeq ($(PROD_REL), 1)
  CFLAGS2+=-DPRODUCTION_REL
  CFLAGS+=-DPRODUCTION_REL
endif
endif

ifeq ($(B913x), 1)
  CFLAGS2+=-DB913x
  CFLAGS+=-DB913x
endif

all: mem_lib ipc_lib ipc_test
ipc_lib:
	$(CC) -c ./lib/fsl_ipc_channel.c $(CFLAGS) -o ./lib/fsl_ipc_channel.o
	$(CC) -c ./lib/fsl_user_dma.c $(CFLAGS) -o ./lib/fsl_user_dma.o
	$(CC) -c ./lib/fsl_ipc_lock.c $(CFLAGS) -o ./lib/fsl_ipc_lock.o
	$(CC) -c ../dsp_boot/libdspboot.c $(CFLAGS2) -o ../dsp_boot/libdspboot.o
	$(CC) -shared -fPIC -lpthread ./lib/fsl_user_dma.o ./lib/fsl_ipc_channel.o ./lib/fsl_ipc_lock.o ../dsp_boot/libdspboot.o -L../fsl_shm -llg_shm -o libipc.so
	$(AR) rcs libipc.a ./lib/fsl_ipc_channel.o ./lib/fsl_ipc_lock.o ../dsp_boot/libdspboot.o ./lib/fsl_user_dma.o ../fsl_shm/lib/lg_shm.o

mem_lib:
	$(CC) -c ./lib/fsl_usmmgr.c $(CFLAGS) -o ./lib/fsl_usmmgr.o
	$(CC) -c ./lib/fsl_ipc_lock.c $(CFLAGS) -o ./lib/fsl_ipc_lock.o
	$(CC) -shared -fPIC ./lib/fsl_usmmgr.o ./lib/fsl_ipc_lock.o -L../fsl_shm -llg_shm  -o libmem.so
	$(AR) rcs libmem.a ./lib/fsl_usmmgr.o ./lib/fsl_ipc_lock.o ../fsl_shm/lib/lg_shm.o

ipc_test:
	$(CC) ./test/ipc_test.c $(CFLAGS) -L. -lipc -lmem -o ipc_test
	$(CC) ./test/ipc_test67.c $(CFLAGS) -L. -lipc -lmem -o ipc_test67
	$(CC) ./test/dsp_recover.c $(CFLAGS) -L. -lipc -lmem -o dsp_recover
	$(CC) ./test/l1_defense_app.c $(CFLAGS) -L. -lipc -lmem -o l1d_app
	$(CC) ./test/fsl_dump_memory.c $(CFLAGS) -L. -lipc -lmem -o dump_mem

clean:
	rm -rf dump_mem ipc_test ipc_test67 dsp_recover l1d_app ./lib/*.o *.a *.so

