PROJECTS = fsl_shm ipc dsp_boot

all:
	for dir in $(PROJECTS); do \
        $(MAKE) -C $$dir;\
	done
clean:
	for dir in $(PROJECTS); do \
        $(MAKE) -C $$dir clean;\
	done


