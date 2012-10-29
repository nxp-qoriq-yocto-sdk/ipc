CC=$(CROSS_COMPILE)gcc
AR=$(CROSS_COMPILE)ar
HET_DIR=$(KERNEL_DIR)/drivers/misc
CFLAGS=-Wall -O3 -lpthread -I ./include -I../fsl_shm/include -I $(HET_DIR) -Wall -DCONFIG_MULTI_RAT

ifeq ($(CONFIG_LOCK), 1)
	CFLAGS+=-DCONFIG_LOCK
endif

all: mem_lib ipc_lib ipc_test
ipc_lib:
	$(CC) -c ./lib/fsl_ipc_channel.c $(CFLAGS) -o ./lib/fsl_ipc_channel.o
	$(CC) -c ./lib/fsl_user_dma.c $(CFLAGS) -o ./lib/fsl_user_dma.o
	$(CC) -c ./lib/fsl_ipc_lock.c $(CFLAGS) -o ./lib/fsl_ipc_lock.o
	$(CC) -shared -fPIC ./lib/fsl_user_dma.o ./lib/fsl_ipc_channel.o ./lib/fsl_ipc_lock.o -o libipc.so
#	$(AR) rcs libipc.a ./lib/fsl_ipc_channel.o ./lib/fsl_user_dma.o

mem_lib:
	$(CC) -c ./lib/fsl_usmmgr.c $(CFLAGS) -o ./lib/fsl_usmmgr.o
	$(CC) -c ./lib/fsl_ipc_lock.c $(CFLAGS) -o ./lib/fsl_ipc_lock.o
	$(CC) -shared -fPIC ./lib/fsl_usmmgr.o ./lib/fsl_ipc_lock.o -L../fsl_shm -llg_shm  -o libmem.so
#	$(AR) rcs libmem.a ./lib/fsl_usmmgr.o

ipc_test:
ifneq (,$(findstring CONFIG_MULTI_RAT, $(CFLAGS)))
	$(CC) ./test/ipc_test_mrat.c $(CFLAGS) -L. -lipc -lmem -o ipc_test_mrat
	$(CC) ./test/ipc_test67_mrat.c $(CFLAGS) -L. -lipc -lmem -o ipc_test67_mrat
	$(CC) ./test/ipc_test_dsp_stub.c $(CFLAGS) -L. -lipc -lmem -o ipc_test_dsp_stub
else
	$(CC) ./test/ipc_test.c $(CFLAGS) -L. -lipc -lmem -o ipc_test
	$(CC) ./test/ipc_test67.c $(CFLAGS) -L. -lipc -lmem -o ipc_test67
endif
	$(CC) ./test/fsl_dump_memory.c $(CFLAGS) -L. -lipc -lmem -o dump_mem

clean:
	rm -rf dump_mem ipc_test ipc_test67 ipc_test_dsp_stub ipc_test_mrat ./lib/*.o *.a *.so

