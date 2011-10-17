PROJECTS = dsp_boot ipc fsl_shm

all:
	for dir in $(PROJECTS); do \
        $(MAKE) -C $$dir;\
	done
clean:
	for dir in $(PROJECTS); do \
        $(MAKE) -C $$dir clean;\
	done


