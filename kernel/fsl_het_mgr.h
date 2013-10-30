/*
 * Copyright 2013 Freescale Semiconductor, Inc.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#ifndef _FSL_HET_MGR_H
#define _FSL_HET_MGR_H

#include <linux/ioctl.h>

#include "fsl_ipc_types.h"

#define HET_MGR_MAGIC	'S'
/* IOCTL to get het sys_map_t */
#define IOCTL_HET_MGR_GET_SYS_MAP	_IOWR(HET_MGR_MAGIC, 1, uint64_t)
/* IOCTL to get virtual to physical addr */
#define IOCTL_HET_MGR_V2P		_IOR(HET_MGR_MAGIC, 2, mem_range_t *)
/* IOCTL to set hardware shemaphore id */
#define IOCTL_HET_MGR_SET_SHMID		_IOW(HET_MGR_MAGIC, 3, uint32_t)
/* IOCTL to get hardware shemaphore id */
#define IOCTL_HET_MGR_GET_SHMID		_IOR(HET_MGR_MAGIC, 4, uint32_t *)
/* IOCTL to get shared memory area size */
#define IOCTL_HET_MGR_SET_SHARED_AREA	_IOW(HET_MGR_MAGIC, 5, uint64_t)
/* IOCTL to set Initialization marker of IPC start area */
#define IOCTL_HET_MGR_SET_INITIALIZED	_IOW(HET_MGR_MAGIC, 6, uint32_t)
/* IOCTL to get hardware shemaphore value */
#define	IOCTL_HW_SEM_GET_VALUE 		_IOR(HET_MGR_MAGIC, 7, hw_sem_t*)
/* IOCTL to set hardware shemaphore value */
#define	IOCTL_HW_SEM_SET_VALUE 		_IOW(HET_MGR_MAGIC, 8, hw_sem_t*)
/* IOCTL to get hardware shemaphore's unique value */
#define IOCTL_HW_SEM_GET_UVALUE 	_IOR(HET_MGR_MAGIC, 9, hw_sem_info_t*)
/* IOCTL to get IPC PARAMS  */
#define IOCTL_HET_MGR_GET_IPC_PARAMS 	_IOR(HET_MGR_MAGIC, 10, uint64_t)
/* IOCTL to get RAT_MODE  */
#define IOCTL_HET_MGR_GET_RAT_MODE 	_IOR(HET_MGR_MAGIC, 11, uint64_t)
/* IOCTL to reset het structure */
#define IOCTL_HET_MGR_RESET_STRUCTURES	_IOW(HET_MGR_MAGIC, 12, uint32_t)

#endif
