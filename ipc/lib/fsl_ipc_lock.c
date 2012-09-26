/*
 * Copyright 2011-2012 Freescale Semiconductor, Inc.
 *
 * Author: Naveen Burmi <naveenburmi@freescale.com>
 */
#include <stdio.h>
#include "fsl_ipc_lock.h"

union semun {
	int val;
	struct semid_ds *buf;
	unsigned short  *array;
	struct seminfo  *__buf;
};

int fsl_ipc_sem_lock(int semid)
{
	int rc = 0;
	int nsops = 2;
	struct sembuf sops[2];

	sops[0].sem_num = 0;
	sops[0].sem_op = 0;
	sops[0].sem_flg = SEM_UNDO;

	sops[1].sem_num = 0;
	sops[1].sem_op = 1;
	sops[1].sem_flg = SEM_UNDO | IPC_NOWAIT;

	rc = semop(semid, sops, nsops);
	if (rc < 0) {
		perror("semop: semop failed");
		rc = -1;
	}

	return rc;
}

int fsl_ipc_sem_unlock(int semid)
{
	int rc = 0;
	int nsops = 1;
	struct sembuf sops;

	sops.sem_num = 0;
	sops.sem_op = -1;
	sops.sem_flg = SEM_UNDO | IPC_NOWAIT;

	rc = semop(semid, &sops, nsops);
	if (rc < 0) {
		perror("semop: semop failed");
		rc = -1;
	}

	return rc;
}

int fsl_ipc_sem_destroy(int semid)
{
	union semun arg;
	int rc = 0;

	if (semctl(semid, 0, IPC_RMID, arg) == -1) {
		perror("semctl");
		rc = -1;
	}

	return rc;
}

int fsl_ipc_sem_init(key_t key)
{
	int semid;
	int nsems = 1;
	int semflg = IPC_CREAT | 0666;

	semid = semget(key, nsems, semflg);
	if (semid < 0) {
		perror("semget: semget failed\n");
		printf("for key %x\n", key);
	}

	return semid;
}


#ifndef CONFIG_LOCK
int fsl_ipc_lock(int semid)
{
	return 0;
}

int fsl_ipc_unlock(int semid)
{
	return 0;
}

int fsl_ipc_init_lock(int objId)
{
	return 0;
}

int fsl_ipc_destroy_lock(int semid)
{
	return 0;
}
#else
int fsl_ipc_lock(int semid)
{
	return fsl_ipc_sem_lock(semid);
}

int fsl_ipc_unlock(int semid)
{
	return fsl_ipc_sem_unlock(semid);
}

int fsl_ipc_destroy_lock(int semid)
{
	return fsl_ipc_sem_destroy(semid);
}

int fsl_ipc_init_lock(key_t key)
{
	return fsl_ipc_sem_init(key);
}
#endif
