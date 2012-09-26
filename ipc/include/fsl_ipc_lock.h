/*
 * Copyright 2011-2012 Freescale Semiconductor, Inc.
 *
 * Author: Naveen Burmi <naveenburmi@freescale.com>
 */
#ifndef FSL_IPC_LOCK_H
#define FSL_IPC_LOCK_H

#include <sys/sem.h>

int fsl_ipc_sem_lock(int semid);
int fsl_ipc_sem_unlock(int semid);
int fsl_ipc_sem_init(key_t key);
int fsl_ipc_sem_destroy(int semid);

int fsl_ipc_lock(int semid);
int fsl_ipc_unlock(int semid);
int fsl_ipc_destroy_lock(int semid);
int fsl_ipc_init_lock(key_t key);

#endif
