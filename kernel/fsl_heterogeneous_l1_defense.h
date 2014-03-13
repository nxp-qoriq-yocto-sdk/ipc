/**
 **  @File          fsl_heterogeneous_L1_defence.h
 **
 **  @Description   Main header file for data_structures and err codes.
 **
 ** Copyright (c) 2013 Freescale Semiconductor Inc.
 **
 ** Redistribution and use in source and binary forms, with or without
 ** modification, are permitted provided that the following conditions
 ** are met:
 ** 1. Redistributions of source code must retain the above copyright
 **    notice, this list of conditions and the following disclaimer.
 ** 2. Redistributions in binary form must reproduce the above copyright
 **    notice, this list of conditions and the following disclaimer in the
 **    documentation and/or other materials provided with the distribution.
 ** 3. Neither the name of Freescale Semiconductor Inc nor the names of its
 **    contributors may be used to endorse or promote products derived from
 **    this software without specific prior written permission.
 **
 ** THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND
 ** ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 ** IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 ** ARE DISCLAIMED.  IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE
 ** FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 ** DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 ** OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 ** HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 ** LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 ** OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 ** SUCH DAMAGE.
 **
 **     Author: Ashish Kumar <Ashish.Kumar@freescale.com>
 **/
#ifndef __FSL_HETEROGENOUS_L1_DEFENSE_H
#define __FSL_HETEROGENOUS_L1_DEFENSE_H

#ifdef B4860

#define MAX_NUM_OF_DSP_CORES 6

#define HET_START_VALID_VALUE    0x12345678
#define HET_END_VALID_VALUE      0x87654321
/*************************************************************************
@Description warm reset modes
**************************************************************************/
typedef enum {
	NO_WARM_RESET = 0x0,
	MODE_1_ACTIVE = 0x1, /* L1 scenrio mode 1 */
	MODE_2_ACTIVE = 0x2, /* L1 scenrio mode 2 */
	MODE_3_ACTIVE = 0x4  /* L1 scenrio mode 3 */
} os_het_l1d_mode_t;

/*************************************************************************
@Description mode 2 maple reset activation
*************************************************************************/
typedef enum {
	RESET_MAPLE_1 = 0x2, /* reset first maple engine */
	RESET_MAPLE_2 = 0x4, /* reset second maple engine */
	RESET_MAPLE_3 = 0x8  /* reset third maple engine */
} os_het_l1d_reset_maple_t;

/**************************************************************************
 @Description   L1 level status enumeration
 *************************************************************************/
 typedef enum {
	/* data was corrupted */
	OS_HET_ERR_L1D_MEMORY_CORRUPTED     = 0x0037FFBF,
	/* Warm reset mode is invalid or unsupported */
	OS_HET_ERR_L1D_MODE_INVALID         = 0x0037FFED,
	/* invalid L1-defense initiation parameters */
	OS_HET_ERR_L1D_FUNCTION_INVALID     = 0x0037FFEC,
	/* System is already in reset mode */
	OS_HET_ERR_L1D_ALREADY_ACTIVE       = 0x0037FFC3,
	/* Unknown error */
	OS_HET_ERR_L1D_UNKNOWN              = 0x00370002,

	/* NMI received, Indication that core is ready for reset */
	OS_HET_INFO_L1D_READY_FOR_RESET     = 0x1037FFBE,
	/* beginning execution of warm reset os initialization */
	BEGIN_WARM_RESET_OS_INIT            = 0x1037FFBD,
	/* beginning execution of warm reset application initialization */
	BEGIN_WARM_RESET_APP_INIT           = 0x1037FFBC,
	/* warm reset was completed successfully */
	WARM_RESET_SUCCESS                  = 0x1037FFBB

} os_het_l1d_status;

/*************************************************************************
@Description Heterogeneous OS L1 defense control structure
*************************************************************************/
typedef struct {
	/* validation value for checking for corrpution in case of reset */
	uint32_t start_validation_value;
	/* definition of the current L1 defense mode to be used*/
	uint32_t warm_reset_mode;
	/* only relevant if MODE_2 is defined in reset_mode - whether/which
	 * maple engine should be reset */
	uint32_t reset_maple;
	/* status and error codes for each of the DSP cores */
	uint32_t reset_status[MAX_NUM_OF_DSP_CORES];
	/* validation value for checking for corrpution in case of reset */
	uint32_t end_validation_value;
} os_het_l1d_t;

#endif

#endif /*__FSL_HETEROGENEOUS_L1_DEFENSE_H */
