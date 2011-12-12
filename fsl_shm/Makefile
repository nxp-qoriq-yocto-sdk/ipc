CC=$(CROSS_COMPILE)gcc
AR=$(CROSS_COMPILE)ar
CFLAGS=-I ./lib -I ./include -Wall

all: lg_shm_lib lg_shm_test
lg_shm_lib:
	$(CC) -c ./lib/lg_shm.c $(CFLAGS) -o ./lib/lg_shm.o
	$(AR) rcs liblg_shm.a ./lib/lg_shm.o

lg_shm_test:
	$(CC) ./test/app.c $(CFLAGS) -L. -llg_shm -o app

clean:
	rm -rf app ./lib/*.o *.a

