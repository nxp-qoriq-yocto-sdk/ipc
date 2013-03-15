/*
 * Copyright 2011-2013 Freescale Semiconductor, Inc.
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

#ifndef IPC_KERN_MOD_H
#define IPC_KERN_MOD_H

#include "fsl_ipc_types.h"
#include <linux/ioctl.h>

#define MAX_SC_PA_CHANNELS	32
#define MAX_IPC_CHANNELS	64
#define DEFAULT_CHANNEL_DEPTH	16
#define DEFAULT_RAT_INST	2
#ifdef CONFIG_MULTI_RAT
#define TOTAL_IPC_CHANNELS	(MAX_IPC_CHANNELS * DEFAULT_RAT_INST)
#else
#define TOTAL_IPC_CHANNELS	MAX_IPC_CHANNELS
#endif

typedef struct {
	uint32_t	max_channels;
	uint32_t	max_depth;
} ipc_bootargs_info_t;

typedef struct {
	uint32_t	channel_id;
	uint32_t	signal;
} ipc_rc_t;

#endif
