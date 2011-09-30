CC=$(CROSS_COMPILE)gcc
AR=$(CROSS_COMPILE)ar
HET_DIR=$(KERNEL_DIR)/drivers/misc
CFLAGS=-lpthread -I ./include -I $(HET_DIR)

all: ipc_lib ipc_test
ipc_lib:
	$(CC) -c ./lib/fsl_ipc_channel.c $(CFLAGS) -o ./lib/fsl_ipc_channel.o
	$(CC) -c ./lib/fsl_ipc_helper.c $(CFLAGS) -o ./lib/fsl_ipc_helper.o
	$(CC) -c ./lib/fsl_user_dma.c $(CFLAGS) -o ./lib/fsl_user_dma.o
	$(AR) rcs libipc.a ./lib/fsl_ipc_channel.o ./lib/fsl_ipc_helper.o ./lib/fsl_user_dma.o

ipc_test:
	$(CC) ./test/ipc_test.c $(CFLAGS) -L. -lipc -o ipc_test

clean:
	rm -rf ipc_test ./lib/*.o *.a

