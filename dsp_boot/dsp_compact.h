#ifndef __DSP_COMPACT_H
#define __DSP_COMPACT_H

#ifdef B4860
#define ADDR_SIZE 8
#define MAP_AREA_MASK 0xfffffffffffff000

#define LCC_BSTRH 0x8
#define LCC_BSTRL 0x9
#define LCC_BSTAR 0xA
#define DCFG_BRR  0x38039
#define GCR_CDCER0  0x23C04B
#define GCR_CHMER0  0x23C053
#define SC_BOOT_HW_SEMAPHORE0		0x23C840

#define OS_HET_SC_SEMAPHORE_VAL         0xFE
#endif

#ifdef B913x
#define ADDR_SIZE 4
#define MAP_AREA_MASK 0xfffff000
#endif

#endif
