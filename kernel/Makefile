KERNEL_DIR ?=
obj-y +=fsl_hetmgr.o
obj-m +=hetmgr.o
hetmgr-objs := fsl_hetmgr.o
obj-y +=fsl_shm.o
obj-m +=shm.o
shm-objs := fsl_shm.o
obj-y +=fsl_L1_defense.o
obj-m +=l1d.o
l1d-objs := fsl_L1_defense.o

ifeq ($(KERNEL_DIR),)
    $(error ERROR: KERNEL_DIR is not defined...!)
endif

ifeq ($(CONFIG_MULTI_RAT), 1)
    ccflags-y += -DCONFIG_MULTI_RAT
endif

ifeq ($(B913x), 1)
    ccflags-y += -DB913x
endif

ifeq ($(B4860), 1)
    ccflags-y += -DB4860
endif

include $(KBUILD_OUTPUT)/.config

ifneq ($(CONFIG_SYS_SUPPORTS_HUGETLBFS),y)
     fatal := $(error HUGETLB is Disabled in Kernel. fsl_shm depends on it)
endif

all:
	make -C $(KERNEL_DIR) SUBDIRS=$(PWD) modules
clean:
	rm -rf *.o *.ko  *.order *.symvers *.mod.c .*.cmd .*_versions
