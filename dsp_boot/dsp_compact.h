/**
 *** @dsp_boot.h
 ***
 *** Copyright 2013 Freescale Semiconductor, Inc.
 ***
 *** Author: Ashish Kumar <ashish.kumar@freescale.com>
 ***/

#ifndef __DSP_COMPACT_H
#define __DSP_COMPACT_H

#ifdef B4860
#define ADDR_SIZE 8
#define MAP_AREA_MASK 0xfffffffffffff000
#define U32_T_MASK 0xFFFFFFFF
#define VIR_ADDR32_MASK 0xfffff000

#define LCC_BSTRH 0x8
#define LCC_BSTRL 0x9
#define LCC_BSTAR 0xA
#define DCFG_BRR  0x38039
#define GCR_CDCER0  0x23C04B
#define GCR_CHMER0  0x23C053
#define SC_BOOT_HW_SEMAPHORE0		0x23C840
#define GIC_VIGR 0x08F2000
#define PCPH15SETR (0xE20B4/4)
#define PCPH15SR (0xE20B0/4)
#define PCPH15PSR (0xE20BC/4)
#define PCPH15CLRR (0xE20B8/4)
#define PIR (0x41090/4)
#define DCFG_CRSTSR	(0xE0410/4)

#define L2_CACHE_2	(0xC60000/4)
#define L2_CACHE_3	(0xCA0000/4)
#define L2_CACHE_4	(0xCE0000/4)
#define OFF_L2_CACHE_X (0x40000/4)
#define OFF_L2_CACHE_X_CSR1 (0x4/4)
/* L2_CACHE_LOCK_FLASH_CLEAR_MASK */
#define L2LFC (1 << 10)
/* L2_CACHE_INVALIDATE_MASK */
#define L2FI (1 << 21)
/* L2_CACHE_FLUSH */
#define L2FL (1 << 11)
/* L2_CACHE_ENABLE */
#define L2E (1 << 31)
/* L2_CACHE_PARITY_ENABLE*/
#define L2PE (1 << 30)

#define NR_DSP_CORE	6
/* VTB flush*/
#define CNPC_PHYSICAL_ADDR          0xF00001000
#define NPC_REGS_SIZE               0x1000
#define DCSR_CNPC_OQCR_OFFSET       0x0C
#define DCSR_CNPC_OQCR_AFA_MASK     0x00000010
/* End vtb flush*/
/* HW WATCHPOINT
 * DCSR base depends upon the corresponding LAW
 */
#define DCSR_BASE 0xF00000000
#define DCSR_CLUSTER0_OFFSET 0x110000
#define DCSR_CLUSTER_OFFSET 0x10000
#define CORE_OFFSET 0x8000
#define DTU_OFFSET 0x4000

#define DHRRR_OFFSET 0xA0
#define DMEER_OFFSET 0x70
#define ARDCR0_OFFSET 0x250
#define DEPCR0_OFFSET 0x254
#define PADRRA0_OFFSET 0x258
#define PADRRB0_OFFSET 0x25C

#define WPT_TYPE_W 0x40001
#define WPT_TYPE_R 0x20001
#define WPT_TYPE_WR 0x60001
#define WPT_REGS_SIZE 0x1000

/* END HW WATCHPOINT */
#define OS_HET_SC_SEMAPHORE_VAL         0xFE

/*
 * DSP safe address is DTU PC_NEXT
 * DCSR base host DTU
 */
#define DSP_SAFE_ADDR 0x80000000
#define PC_NEXT_OFFSET 0x50
#define RCR_OFFSET 0x54
#define SASR_OFFSET 0x120
#define SASR_DM_MASK 0x00000008
#define RCR_VALUE 001
#define RCPM_OFFSET 0x22000
#define CRSMCR0_OFFSET 0x5C
/* END DTU RESUME ADDR */
#endif

#ifdef B913x
#define ADDR_SIZE 4
#define MAP_AREA_MASK 0xfffff000
#define VIR_ADDR32_MASK 0xfffff000
#define GIC_VIGR 0x17000
#define GIC_VIGR_VALUE 0x107
#endif

#endif
