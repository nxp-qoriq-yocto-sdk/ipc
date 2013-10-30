/*
 * * Copyright (c) 2013 Freescale Semiconductor, Inc.
 * *
 * * Freescale compact header file
 * *
 * * Author: Ashish kumar <ashish.kumar@freescale.com>
 * *
 */
#ifndef __FSL_KER_COMPACT_H
#define __FSL_KER_COMPACT_H

#ifdef B913x
#define PA_CCSRBAR      0xff700000
#define PA_CCSR_SZ      0x00100000
#define DSP_CCSRBAR     0xff600000
#define DSP_CCSR_SZ     0x00100000
#define DSP_CORE0_M2    0xb0000000
#define DSP_CORE1_M2    0xb1000000
#define DSP_M3          0xc0000000
#define DSP_CORE0_M2_SZ (512*1024)
#define DSP_CORE1_M2_SZ (512*1024)
#define DSP_M3_SZ       (32*1024)
#define HW_SEM_OFFSET   0x17100

#define DSP_SHARED_SZ                   0x1000000
#define SHARED_CTRL_AREA_START_ADDR     0x37000000
#define SHARED_CTRL_AREA_SZ             0x1000000
#define DSP_PVT_START_ADDR              0x38000000
#define DSP_PVT_SZ                      0x8000000
#endif

#ifdef B4860
#define PA_CCSRBAR      0xFFE000000
#define PA_CCSR_SZ      0x1000000
#define DSP_CCSRBAR     0xFFE000000
#define DSP_CCSR_SZ     0x1000000
#define DSP_M3          0xc40000000
#define DSP_M3_SZ       (64*512)
#define HW_SEM_OFFSET   0x8F2108
#define NR_DSP_CORE	6

#define SHARED_CTRL_AREA_START_ADDR     0xFFF00000
/* only 1 mb for shared ctrl area for B4 */
#define SHARED_CTRL_AREA_SZ             0x100000
#define DSP_PVT_START_ADDR              0x80000000
#define DSP_PVT_SZ                      0x7FF00000
#define DSP_SHARED_SZ                   0x1000000
#endif
#endif
