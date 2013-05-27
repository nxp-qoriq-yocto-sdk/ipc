/*
 *  @ fsl_hetmgr.c
 *
 * Copyright (c) 2011-2013, Freescale Semiconductor Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of Freescale Semiconductor Inc nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 *      Author: Manish Jaggi <manish.jaggi@freescale.com>
 *      Author: Ashish Kumar <ashish.kumar@freescale.com>
 *
 */
#include "fsl_ipc_types.h"
#include "fsl_het_mgr.h"
#include <linux/io.h>
#include <linux/mm.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/of.h>
#include <asm/uaccess.h>
#include "fsl_heterogeneous.h"
#include "fsl_ker_compact.h"


static long dsp_shared_size = DSP_SHARED_SZ;
module_param(dsp_shared_size , long, S_IRUGO);
static long dsp_private_addr = DSP_PVT_START_ADDR;
module_param(dsp_private_addr , long, S_IRUGO);
static long dsp_private_size = DSP_PVT_SZ;
module_param(dsp_private_size , long, S_IRUGO);
static long shared_ctrl_addr = SHARED_CTRL_AREA_START_ADDR;
module_param(shared_ctrl_addr , long, S_IRUGO);
static long shared_ctrl_size = SHARED_CTRL_AREA_SZ;
module_param(shared_ctrl_size , long, S_IRUGO);

static int 			fslhet_devno;
static dev_t 			fslhet_dev;
static struct cdev 		fslhet_cdev;
static uint32_t 		het_mgr_major;
static uint32_t 		het_mgr_minor;
static sys_map_t 		sys_map;

static int 			shmid;
static void __iomem		*sem;
static shared_area_t 		shared_area;
static phys_addr_t 		sh_ctrl_area_mark;
static os_het_control_t	*ctrl;


#ifdef CONFIG_MULTI_RAT
#define MAX_NUM_IPC_REGIONS 2
#endif

static int het_mgr_open(struct inode *inode, struct file *filep);
static int het_mgr_release(struct inode *inode, struct file *filep);
#ifdef HAVE_UNLOCKED_IOCTL
static long het_mgr_ioctl(struct file *filp, unsigned int cmd,
		unsigned long arg);
#else
static int het_mgr_ioctl(struct inode *inode, struct file *filp,
		unsigned int cmd, unsigned long arg);
#endif

#ifdef CONFIG_MULTI_RAT
uint32_t get_hetmgr_rat_instances(void)
{
	return ctrl->num_ipc_regions;
}
EXPORT_SYMBOL(get_hetmgr_rat_instances);
#endif


int init_sh_ctrl_area(void)
{
	int ctr = 0;
	unsigned long tmp;
	os_het_debug_print_t      *het_debug_print_v;
	os_het_debug_print_sc_t   *het_debug_print_sc_array_v;

	pr_info("Initializing Shared control area\n");
	pr_info("Shared Control area start address = %lx\n",
			sys_map.sh_ctrl_area.phys_addr);

	ctrl = ioremap(
		sys_map.sh_ctrl_area.phys_addr, sys_map.sh_ctrl_area.size);
	if (!ctrl) {
		pr_err("ioremap failed with addr=%lx val=%x\n",
		sys_map.sh_ctrl_area.phys_addr, sys_map.sh_ctrl_area.size);
		return -1;
	}

	/* zeroize the structure */
	memset(ctrl, 0, sys_map.sh_ctrl_area.size);

	ctr += sizeof(os_het_control_t);
	ctrl->shared_ctrl_size = 0x4000; /* 16k */
	/* set the physical adress for ipc */
	/* casting is done just to avoid compilation errors */
	ctrl->ipc = (void *)(sys_map.sh_ctrl_area.phys_addr + ctr);
	pr_info("IPC structure start address = %lx\n",
			sys_map.sh_ctrl_area.phys_addr + ctr);
#ifndef CONFIG_MULTI_RAT
	ctr += sizeof(os_het_ipc_t);
#else
	ctrl->num_ipc_regions =  MAX_NUM_IPC_REGIONS;
	ctr += ctrl->num_ipc_regions * sizeof(os_het_ipc_t);
#endif
	tmp = sys_map.sh_ctrl_area.phys_addr + ctr;

	memcpy((void *)&ctrl->smartdsp_debug, &tmp, sizeof(long));
	pr_info("Smart DSP Debug start address = %lx\n", tmp);

	ctr += sizeof(os_het_smartdsp_log_t) * 2;

	/*debug_print initilization start*/

	tmp = sys_map.sh_ctrl_area.phys_addr + ctr;
	ctrl->het_debug_print = (os_het_debug_print_t *)tmp;

	het_debug_print_v =
		(os_het_debug_print_t *) ((unsigned long) ctrl + ctr);

	ctr += sizeof(os_het_debug_print_t);

	het_debug_print_v->sc_array_size = NUM_OF_DBGP_SC_CORES*
						NUM_OF_DBGP_TABLES_PER_SC;

	tmp = sys_map.sh_ctrl_area.phys_addr + ctr;
	het_debug_print_v->sc_debug_print = (void *)tmp;

	het_debug_print_sc_array_v =
	(os_het_debug_print_sc_t *) ((unsigned long) ctrl + ctr);

	memset((void *)het_debug_print_sc_array_v,
			0,
			het_debug_print_v->sc_array_size*
			sizeof(os_het_debug_print_sc_t));

	ctr += (het_debug_print_v->sc_array_size*
	sizeof(os_het_debug_print_sc_t));

	/*debug_print initilization end*/

	sh_ctrl_area_mark = sys_map.sh_ctrl_area.phys_addr + ctr;
	pr_info("Free Area starts from %llx\n",
		(unsigned long long)sh_ctrl_area_mark);

	return 0;
}

int init_hw_sem(void)
{
	sem = ioremap(sys_map.dsp_ccsrbar.phys_addr + HW_SEM_OFFSET, 0x100);
	if (sem == NULL) {
		pr_err("%s: Error in ioremap\n", __func__);
		return -1;
	}

	pr_info("virt addr of sem= %p\n", sem);

	return 0;
}

static int hugev2p(mem_range_t *r)
{
	int ret = 0;
	struct task_struct *tsk;
	struct mm_struct *mm;
	unsigned long start;
	int len;
	int write;
	int force;
	struct page *pages = NULL;
	struct page *pg;
	struct vm_area_struct *vmas = NULL;
	int npages;

	tsk = current;
	mm = current->mm;
	start = (unsigned long)r->vaddr;
	len = 1;
	write = 0;
	force = 0;

	down_read(&current->mm->mmap_sem);
	ret = get_user_pages(tsk, mm, start, len, write, force, &pages, &vmas);
	up_read(&current->mm->mmap_sem);
	if (ret > 0) {
		npages = ret;
		pg = &pages[0];
		pr_info("%llx Addr\n", (unsigned long long)page_to_phys(pg));
		r->phys_addr = page_to_phys(pg);
		put_page(pg);
	}

	return 0;
}

int get_het_sys_map(sys_map_t *sysmap)
{
	memset(sysmap, 0, sizeof(sys_map_t));

	sysmap->dsp_shared_size =  dsp_shared_size;
	if (sysmap->dsp_shared_size < DSP_SHARED_SZ)
		sysmap->dsp_shared_size = DSP_SHARED_SZ;

	pr_info("IPC: dsp_shared_size - %#x\n", sysmap->dsp_shared_size);

	sysmap->pa_ccsrbar.phys_addr = PA_CCSRBAR;
	sysmap->pa_ccsrbar.size = PA_CCSR_SZ;

	sysmap->dsp_ccsrbar.phys_addr = DSP_CCSRBAR;
	sysmap->dsp_ccsrbar.size = DSP_CCSR_SZ;

	sysmap->smart_dsp_os_priv_area.phys_addr = dsp_private_addr;
	if (sysmap->smart_dsp_os_priv_area.phys_addr < DSP_PVT_START_ADDR)
		sysmap->smart_dsp_os_priv_area.phys_addr = DSP_PVT_START_ADDR;

	pr_info("IPC: dsp_private_addr - %#lx\n",
		sysmap->smart_dsp_os_priv_area.phys_addr);

	sysmap->smart_dsp_os_priv_area.size = dsp_private_size;
	if (sysmap->smart_dsp_os_priv_area.size < DSP_PVT_SZ)
		sysmap->smart_dsp_os_priv_area.size = DSP_PVT_SZ;

	pr_info("IPC: dsp_private_size - %#x\n",
		sysmap->smart_dsp_os_priv_area.size);

	sysmap->sh_ctrl_area.phys_addr = shared_ctrl_addr;
	if (sysmap->sh_ctrl_area.phys_addr < SHARED_CTRL_AREA_START_ADDR)
		sysmap->sh_ctrl_area.phys_addr = SHARED_CTRL_AREA_START_ADDR;

	pr_info("IPC: shared_ctrl_addr - %#lx\n",
		sysmap->sh_ctrl_area.phys_addr);

	sysmap->sh_ctrl_area.size = shared_ctrl_size;
	if (sysmap->sh_ctrl_area.size < SHARED_CTRL_AREA_SZ)
		sysmap->sh_ctrl_area.size = SHARED_CTRL_AREA_SZ;

	pr_info("IPC: shared_ctrl_size - %#x\n", sysmap->sh_ctrl_area.size);
#ifdef DSP_CORE0_M2
	sysmap->dsp_core0_m2.phys_addr = DSP_CORE0_M2;
	sysmap->dsp_core0_m2.size = DSP_CORE0_M2_SZ;
#endif
	if ((of_find_compatible_node(NULL, NULL, "fsl,bsc9132qds")) ||
	   (of_find_compatible_node(NULL, NULL, "fsl,B4860QDS"))) {
#ifdef DSP_CORE1_M2
		sysmap->dsp_core1_m2.phys_addr = DSP_CORE1_M2;
		sysmap->dsp_core1_m2.size = DSP_CORE1_M2_SZ;
#endif
		pr_info("IPC: M2 and M3 exist in star core\n");
		sysmap->dsp_m3.phys_addr = DSP_M3;
		sysmap->dsp_m3.size = DSP_M3_SZ;
	}

	return 0;
}

#ifndef CONFIG_MULTI_RAT
int get_hetmgr_ipc_addr(mem_range_t *r)
#else
int get_hetmgr_ipc_addr(mem_range_t *r, int inst_id)
#endif
{
	r->phys_addr = (phys_addr_t)ctrl->ipc;
#ifdef CONFIG_MULTI_RAT
	r->phys_addr += inst_id*sizeof(os_het_ipc_t);
#endif
	r->vaddr = (void *)ctrl + ((unsigned long)ctrl->ipc -
#ifndef CONFIG_MULTI_RAT
	sys_map.sh_ctrl_area.phys_addr);
#else
	sys_map.sh_ctrl_area.phys_addr) +
	sizeof(os_het_ipc_t)*inst_id;
#endif
	r->size = sizeof(os_het_ipc_t);

	return 0;
}
EXPORT_SYMBOL(get_hetmgr_ipc_addr);

int get_hetmgr_mem(mem_range_t *r)
{
	r->phys_addr = sh_ctrl_area_mark;
	r->vaddr = (void *)ctrl + (sh_ctrl_area_mark -
			sys_map.sh_ctrl_area.phys_addr);
	sh_ctrl_area_mark += r->size;
	if (sh_ctrl_area_mark > ctrl->shared_ctrl_size)
		ctrl->shared_ctrl_size *= 4;
	return 0;
}
EXPORT_SYMBOL(get_hetmgr_mem);

static int het_mgr_open(struct inode *inode, struct file *filep)
{
	return 0;
}

static int het_mgr_release(struct inode *inode, struct file *filep)
{
	return 0;
}

#ifdef HAVE_UNLOCKED_IOCTL
static long het_mgr_ioctl(struct file *filp, unsigned int cmd,
		unsigned long arg)
#else
static int het_mgr_ioctl(struct inode *inode, struct file *filp,
		unsigned int cmd, unsigned long arg)
#endif
{
	int ret = 0;
	sys_map_t *tmp_sys_map;
	mem_range_t	*tmp;
	mem_range_t r;
	hw_sem_info_t	hinfo;
	hw_sem_t	hwsem;

	switch (cmd) {

	case IOCTL_HET_MGR_GET_SYS_MAP:
		tmp_sys_map = (sys_map_t *)arg;
		if (copy_to_user(tmp_sys_map, &sys_map, sizeof(sys_map_t)))
			ret = -EFAULT;
		break;

	case IOCTL_HET_MGR_V2P:
		tmp = (mem_range_t *)arg;
		if (copy_from_user(&r, tmp, sizeof(mem_range_t))) {
			ret = -EFAULT;
			break;
		}

		pr_info("Virt Address=%lx Size=%x\n", (unsigned long)r.vaddr,
			r.size);
		ret = hugev2p(&r);
		if (copy_to_user(tmp, &r, sizeof(mem_range_t)))
			ret = -EFAULT;
		break;

	case IOCTL_HET_MGR_SET_SHMID:
		if (copy_from_user(&shmid, (uint32_t *)arg, sizeof(uint32_t)))
			ret = -EFAULT;
		break;

	case IOCTL_HET_MGR_GET_SHMID:
		if (copy_to_user((uint32_t *)arg, &shmid, sizeof(uint32_t)))
			ret = -EFAULT;
		break;

	case IOCTL_HET_MGR_SET_SHARED_AREA:
		if (copy_from_user(&shared_area, (shared_area_t *)arg,
				sizeof(shared_area_t))) {
			ret = -EFAULT;
		break;
		}

		ctrl->pa_shared_mem.start_addr =
			shared_area.pa_ipc_shared.phys_addr;
		ctrl->pa_shared_mem.size = shared_area.pa_ipc_shared.size;
		ctrl->sc_shared_mem.start_addr =
			shared_area.dsp_ipc_shared.phys_addr;
		ctrl->sc_shared_mem.size = shared_area.dsp_ipc_shared.size;
		break;

	case IOCTL_HET_MGR_SET_INITIALIZED:
		#ifdef CONFIG_MULTI_RAT
		ctrl->initialized.pa_initialized = OS_HET_INITIALIZED_MULTIMODE;
		#else
		ctrl->initialized.pa_initialized = OS_HET_INITIALIZED;
		#endif
		ctrl->initialized.sc_initialized = OS_HET_UNINITIALIZED;
		break;

	case IOCTL_HW_SEM_GET_UVALUE:
		hinfo.pa_uniq_val = OS_HET_PA_SEMAPHORE_VAL;
		hinfo.dsp_uniq_val = OS_HET_SC_SEMAPHORE_VAL;
		if (copy_to_user((void *)arg, &hinfo, sizeof(hw_sem_info_t)))
			ret = -EFAULT;
		break;

	case IOCTL_HW_SEM_SET_VALUE:
		if (copy_from_user(&hwsem, (void *)arg, sizeof(hw_sem_t))) {
			ret = -EFAULT;
			break;
		}

		if (hwsem.sem_no  > 7) {
			pr_err("Ilegal value %d of hwsem.sem_no\n",
				hwsem.sem_no);
			break;
		}

		iowrite32be(hwsem.value, (sem + hwsem.sem_no*0x8));
		break;

	case IOCTL_HW_SEM_GET_VALUE:
		if (copy_from_user(&hwsem, (void *)arg, sizeof(hw_sem_t))) {
			ret = -EFAULT;
			break;
		}

		hwsem.value = ioread32be((sem + hwsem.sem_no*0x8));
		if (copy_to_user((void *)arg, &hwsem, sizeof(hw_sem_t)))
			ret = -EFAULT;
		break;

	default:
		return -ENOTTY;
	}

	return ret;
}
/*
file operations data structure
*/
static const struct file_operations het_mgr_fops = {
	.owner	= THIS_MODULE,
	.open = 	het_mgr_open,
	.release =	het_mgr_release,
#ifdef HAVE_UNLOCKED_IOCTL
	.unlocked_ioctl =  	het_mgr_ioctl,
#else
	.ioctl =  	het_mgr_ioctl,
#endif
};

int het_mgr_init(void)
{
	int ret;
	het_mgr_major = 0;
	het_mgr_minor = 0;
	fslhet_dev = 0;
	/*register /dev/het_mgr character driver */
	if (het_mgr_major) {
		fslhet_dev = MKDEV(het_mgr_major, het_mgr_minor);
		ret = register_chrdev_region(fslhet_dev, 1, "/dev/het_mgr");

	} else {
		ret = alloc_chrdev_region(&fslhet_dev, het_mgr_minor,
				1, "/dev/het_mgr");
		het_mgr_major = MAJOR(fslhet_dev);
	}

	if (ret < 0) {
		pr_err("het_mgr_dev: can't get major %d\n", het_mgr_major);
		return ret;
	}

	fslhet_devno = MKDEV(het_mgr_major, het_mgr_minor);
	pr_info("Het Mgr %d %d\n", het_mgr_major, het_mgr_minor);
	cdev_init(&fslhet_cdev, &het_mgr_fops);
	fslhet_cdev.owner = THIS_MODULE;
	fslhet_cdev.ops = &het_mgr_fops;
	ret = cdev_add(&fslhet_cdev, fslhet_devno, 1);

	/* Fail gracefully if need be */
	if (ret) {
		pr_err("Error %d adding Heterogeneous System Manager", ret);
		unregister_chrdev_region(fslhet_dev, 1);
		return ret;
	}

	ret = get_het_sys_map(&sys_map);
	if (ret)
		goto end;

	ret = init_sh_ctrl_area();
	if (ret)
		goto end;

	ret = init_hw_sem();
	if (ret)
		goto end;

	return ret;
end:
	cdev_del(&fslhet_cdev);
	unregister_chrdev_region(fslhet_dev, 1);
	return ret;
}

void het_mgr_exit(void)
{
	cdev_del(&fslhet_cdev);
	unregister_chrdev_region(fslhet_dev, 1);
}

MODULE_AUTHOR("Ashish.Kumar@freescale.com");
MODULE_DESCRIPTION("Heterogeneous System Manager driver");
MODULE_LICENSE("GPL");

module_init(het_mgr_init);
module_exit(het_mgr_exit);
