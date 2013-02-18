/**
 ** @bsc913x_dsp_boot.h
 **
 ** Copyright 2013 Freescale Semiconductor, Inc.
 **
 ** Author: Ashish Kumar <ashish.kumar@freescale.com>
 **/
#ifndef __BSC913X_DSP_BOOT_H
#define __BSC913X_DSP_BOOT_H

#ifdef DEBUG_RELOAD
#define reload_print(...)  printf(__VA_ARGS__);
#else
#define reload_print(...)
#endif
typedef void *fsl_ipc_t;

int send_vnmi_func();
int fsl_ipc_reinit(fsl_ipc_t *);
int fsl_load_dsp_image(char *);
int fsl_restart_L1(fsl_ipc_t, char*);
#define DSP_BOOT_SUCCESS 2
#endif
