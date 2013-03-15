KERNEL_DIR ?=
obj-y +=fsl_ipc.o
obj-m +=ipc.o
ipc-objs := fsl_ipc.o
obj-y +=fsl_hetmgr.o
obj-m +=hetmgr.o
hetmgr-objs := fsl_hetmgr.o

ifeq ($(KERNEL_DIR),)
    $(error ERROR: KERNEL_DIR is not defined...!)
endif

ifeq ($(CONFIG_MULTI_RAT), 1)
    ccflags-y += -DCONFIG_MULTI_RAT
endif

all:
	make -C $(KERNEL_DIR) SUBDIRS=$(PWD) modules
clean:
	rm *.o *.ko  *.order *.symvers *.mod.c
