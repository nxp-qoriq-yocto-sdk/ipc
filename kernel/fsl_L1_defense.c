/*
 *  @ fsl_L1_defense.c
 *
 * Copyright (c) 2013, Freescale Semiconductor Inc.
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
 *      Author: Ashish Kumar <Ashish.Kumar@freescale.com>
 *
 */
#include <linux/io.h>
#include <linux/mm.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/of.h>
#include <asm/uaccess.h>
#include "fsl_heterogeneous.h"
#include "fsl_ker_compact.h"
#include "fsl_ipc_kmod.h"

#include <linux/irq.h>
#include <linux/smp.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/wait.h>
#include <linux/sched.h>

#include <asm/io.h>
#include <asm/irq.h>
#include <asm/mpic.h>
#include "fsl_L1_defense.h"
static uint32_t fsl_l1d_map_wsrsr_reg(void);
static void fsl_l1d_clear_wsrsr_reg(uint32_t);

static int		fsl_l1d_devno;
static dev_t		fsl_l1d_dev;
static struct cdev 	fsl_l1d_cdev;
static uint32_t 	fsl_l1d_major;
static uint32_t 	fsl_l1d_minor;
static void __iomem	*vaddr_wsrsr;
static spinlock_t sp;
static wait_queue_head_t wq;
static uint32_t wsrsr_bitmask;
static uint32_t 	virq;

#define MPIC_WSRSR 0xFFE043A00

static int fsl_l1d_open(struct inode *inode, struct file *filep);
static int fsl_l1d_release(struct inode *inode, struct file *filep);
#ifdef HAVE_UNLOCKED_IOCTL
static long fsl_l1d_ioctl(struct file *filp, unsigned int cmd,
		unsigned long arg);
#else
static int fsl_l1d_ioctl(struct inode *inode, struct file *filp,
		unsigned int cmd, unsigned long arg);
#endif


static int fsl_l1d_open(struct inode *inode, struct file *filep)
{
	return 0;
}

static int fsl_l1d_release(struct inode *inode, struct file *filep)
{
	return 0;
}

static uint32_t fsl_l1d_map_wsrsr_reg()
{
	vaddr_wsrsr = ioremap(MPIC_WSRSR, sizeof(uint64_t));
	if (vaddr_wsrsr == NULL) {
		pr_info("ioremap failed\n");
		return 0xFFFFFFFF;
	}
	return ioread32be(vaddr_wsrsr);
}

static void fsl_l1d_clear_wsrsr_reg(uint32_t wsrsr_clear_mask)
{
	iowrite32be(wsrsr_clear_mask, vaddr_wsrsr);
	return;
}

#ifdef HAVE_UNLOCKED_IOCTL
static long fsl_l1d_ioctl(struct file *filp, unsigned int cmd,
		unsigned long arg)
#else
static int fsl_l1d_ioctl(struct inode *inode, struct file *filp,
		unsigned int cmd, unsigned long arg)
#endif
{
	int ret = 0;
	uint32_t *tmp;
	unsigned long flags;
	switch (cmd) {

	case IOCTL_FSL_L1D_GET_WSRSR_BITMASK:
		pr_debug("IOCTL REACHED\n");
		spin_lock_irqsave(&sp, flags);
		if (wsrsr_bitmask) {
			pr_debug("Enter func %s wsrsr= %#x\n",
			  __func__, wsrsr_bitmask);
			tmp = (uint32_t *)arg;
			if (copy_to_user(tmp, &wsrsr_bitmask,
			    sizeof(uint32_t))) {
				printk(KERN_ERR"Error in copy to user\n");
				ret = -EFAULT;
			} else
				wsrsr_bitmask = 0;
			spin_unlock_irqrestore(&sp, flags);
		} else {
			spin_unlock_irqrestore(&sp, flags);
			wait_event_interruptible(wq, wsrsr_bitmask != 0);
		}
		break;
	default:
		return -ENOTTY;
	}

	return ret;
}

static int fsl_l1d_mmap(struct file *filp, struct vm_area_struct *vma)
{
	int ret;
	long length = vma->vm_end - vma->vm_start;

	/*
	 Protection here is cacheable and coherent
	 */
	vma->vm_page_prot |= _PAGE_WRITETHRU;
	pr_debug("CHACHED WRITETHRU vm_page_prot %lx\n",
			pgprot_val(vma->vm_page_prot));

	ret = remap_pfn_range(vma, vma->vm_start,
			      vma->vm_pgoff, length,
			      vma->vm_page_prot);
	if (ret < 0)
		return ret;

	return 0;
}

/*
file operations data structure
*/
static const struct file_operations fsl_l1d_fops = {
owner:	THIS_MODULE,
open :	fsl_l1d_open,
release :	fsl_l1d_release,
#ifdef HAVE_UNLOCKED_IOCTL
unlocked_ioctl :	fsl_l1d_ioctl,
#else
ioctl :  	fsl_l1d_ioctl,
#endif

#ifdef CONFIG_COMPAT
compat_ioctl :	fsl_l1d_ioctl,
#endif
mmap :	fsl_l1d_mmap
};

static irqreturn_t fsl_l1d_interrupt_handler(int irq, void *data)
{
	spin_lock(&sp);
	wsrsr_bitmask |= ioread32be(vaddr_wsrsr);
	spin_unlock(&sp);
	pr_debug("Enter func %s wsrsr= %#x\n", __func__, wsrsr_bitmask);
	fsl_l1d_clear_wsrsr_reg(wsrsr_bitmask);
	wake_up_interruptible(&wq);

	return IRQ_HANDLED;
}

static int mpic_watchdog_init(irq_hw_number_t irqnum)
{
	int ret = 0;

	pr_debug("Enter func %s\n", __func__);
	virq = irq_create_mapping(NULL, irqnum);
	if (virq == NO_IRQ) {
		pr_err("Error interrupt setup failed\n");
		return -1;
	}

	ret = request_irq(virq, fsl_l1d_interrupt_handler, 0,
			  "mpic-watchdog-int", NULL);
	if (ret) {
		pr_err("Error %d in fsl_l1d_interrupt_handler", ret);
		return -1;
	} else
		return 0;
}

static int fsl_l1d_init(void)
{
	int ret;
	fsl_l1d_major = 0;
	fsl_l1d_minor = 0;
	fsl_l1d_dev = 0;

	/*register /dev/fsl_l1d character driver */
	if (fsl_l1d_major) {
		fsl_l1d_dev = MKDEV(fsl_l1d_major, fsl_l1d_minor);
		ret = register_chrdev_region(fsl_l1d_dev, 1,
				"/dev/fsl_l1d");

	} else {
		ret = alloc_chrdev_region(&fsl_l1d_dev, fsl_l1d_minor,
				1, "/dev/fsl_l1d");
		fsl_l1d_major = MAJOR(fsl_l1d_dev);
	}

	if (ret < 0) {
		pr_err("fsl_l1d_dev: can't get major %d\n", fsl_l1d_major);
		return ret;
	}

	fsl_l1d_devno = MKDEV(fsl_l1d_major, fsl_l1d_minor);
	pr_info("fsl_l1d Major=%d Minor=%d\n", fsl_l1d_major, fsl_l1d_minor);
	cdev_init(&fsl_l1d_cdev, &fsl_l1d_fops);
	fsl_l1d_cdev.owner = THIS_MODULE;
	fsl_l1d_cdev.ops = &fsl_l1d_fops;
	ret = cdev_add(&fsl_l1d_cdev, fsl_l1d_devno, 1);

	/* Fail gracefully if need be */
	if (ret) {
		pr_err("Error %d in adding IPC WATCHDOG Manager", ret);
		goto end_l1d;
	}

	/* This is required only if the spinlock_t varibale sp *
	*  is not initailized *
	*/
	spin_lock_init(&sp);
	wsrsr_bitmask = fsl_l1d_map_wsrsr_reg();
	if (wsrsr_bitmask == 0xFFFFFFFF) {
		pr_err("Error in map_wsrsr_reg\n");
		ret = -1;
		goto end_l1d;
	}

	pr_debug("Wsrsr = %x\n", wsrsr_bitmask);
	init_waitqueue_head(&wq);

	ret = mpic_watchdog_init(FSL_MPIC_WATCHDOG_IRQ);
	if (ret < 0) {
		pr_err("Error in mpic_watchdog_init\n");
		goto end_l1d;
	} else
		return ret;

end_l1d:
	unregister_chrdev_region(fsl_l1d_dev, 1);
	return ret;
}

static void fsl_l1d_exit(void)
{
	free_irq(virq, NULL);
	cdev_del(&fsl_l1d_cdev);
	unregister_chrdev_region(fsl_l1d_dev, 1);
}

MODULE_AUTHOR("Ashish.Kumar@freescale.com");
MODULE_DESCRIPTION("IPC WATCHDOG Manager driver");
MODULE_LICENSE("GPL");

module_init(fsl_l1d_init);
module_exit(fsl_l1d_exit);
