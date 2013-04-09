/*
 * Copyright (C) 2013 Freescale Semiconductor, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN
 * NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
 * TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *	Author: Naveen Burmi <naveenburmi@freescale.com>
 */

#include <linux/init.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <asm/uaccess.h>

#include "fsl_shm.h"

#define MIN_FREE_SIZE_LIST		0x1000
#define NUMBER_OF_LISTS			8
#define MAX_FREE_SIZE_LIST		(MIN_FREE_SIZE_LIST * NUMBER_OF_LISTS)
#define MASK				(~(MIN_FREE_SIZE_LIST - 1))
#define LIST_INDEX_FROM_SIZE(size)	((size & MASK)/MIN_FREE_SIZE_LIST)
#define SPLIT_THREASHOLD		0
#define OFFSET_OF(node)			(offsetof(node, g_entry))

#define MIN_ADDRESS_LIST		0x1000
#define MAX_ADDRESS_LIST		(MIN_ADDRESS_LIST * NUMBER_OF_LISTS)
#define ADDRESS_MASK			(~(MIN_FREE_SIZE_LIST - 1))
#define LIST_INDEX_FROM_ADDRESS(addr)	((addr & ADDRESS_MASK)/MIN_ADDRESS_LIST)

static int fsl_shm_major;
static spinlock_t mm_lock;
static void *psa;
static unsigned long shm_size;

struct global_node {
	struct global_node *g_next;
	struct global_node *g_prev;
};

typedef struct node {
	unsigned char tag;
	unsigned long address;
	unsigned int size;
	struct node *next;
	struct node *prev;
	struct global_node g_entry;
} node;

typedef node free_node;
typedef node allocated_node;

struct global_node global_list_head;

free_node free_area_lists[NUMBER_OF_LISTS + 1];
allocated_node allocated_area_lists[NUMBER_OF_LISTS + 1];

static allocated_node *get_allocated_list_from_address(unsigned long address)
{
	int index = -1;

	if (address > MAX_ADDRESS_LIST)
		return &allocated_area_lists[NUMBER_OF_LISTS];

	index = LIST_INDEX_FROM_ADDRESS(address);

	return &allocated_area_lists[index];
}

static free_node *get_list(unsigned int size)
{
	int index = -1;

	if (size > MAX_FREE_SIZE_LIST)
		return &free_area_lists[NUMBER_OF_LISTS];

	index = LIST_INDEX_FROM_SIZE(size);

	return &free_area_lists[index];
}

static free_node *get_free_area_list(unsigned int size)
{
	int index = -1;

	if (size > MAX_FREE_SIZE_LIST)
		return &free_area_lists[NUMBER_OF_LISTS];

	index = LIST_INDEX_FROM_SIZE(size);
	do {
		if (free_area_lists[index].size > size) {
			/* chech whether this list contains an entry
			 * which is bigger then OR equal to the required
			 * size. This can be done by verifying the size of
			 * entry present at tail.
			 */
			if (free_area_lists[index].prev->size >= size)
				return &free_area_lists[index];
		}

		index++;
	} while (index < (NUMBER_OF_LISTS + 1));

	return NULL;
}

/* list add/remove operations on the List*/

static void list_add_allocated_area(allocated_node *entry)
{
	allocated_node *head;

	head = get_allocated_list_from_address(entry->address);
	head->size = head->size + entry->size;

	if (head->prev->size <= entry->size) {
		/* add allocated area at tail */
		entry->next = head;
		entry->prev = head->prev;
		head->prev->next = entry;
		head->prev = entry;
	} else if (head->next->size > entry->size) {
		/* add allocated area at head */
		entry->next = head->next;
		entry->prev = head;
		head->next->prev = entry;
		head->next = entry;
	} else {
		/* add allocated area after 1st entry at head */
		entry->next = head->next->next;
		entry->prev = head->next;
		entry->next->prev = entry;
		entry->prev->next = entry;
	}
}

static void list_remove_allocated_area(allocated_node *entry)
{
	allocated_node *head;

	head = get_allocated_list_from_address(entry->address);
	head->size -= entry->size;

	entry->prev->next = entry->next;
	entry->next->prev = entry->prev;
	entry->next = NULL;
	entry->prev = NULL;
}

static void list_add_free_area(free_node *entry)
{
	free_node *head;

	head = get_list(entry->size);
	head->size = head->size + entry->size;

	if (head->prev->size <= entry->size) {
		/* add free area at tail */
		entry->next = head;
		entry->prev = head->prev;
		head->prev->next = entry;
		head->prev = entry;
	} else if (head->next->size > entry->size) {
		/* add free area at head */
		entry->next = head->next;
		entry->prev = head;
		head->next->prev = entry;
		head->next = entry;
	} else {
		/* add free area after 1st entry at head */
		entry->next = head->next->next;
		entry->prev = head->next;
		entry->next->prev = entry;
		entry->prev->next = entry;
	}
}

static void list_remove_free_area(free_node *entry)
{
	free_node *head;

	head = get_list(entry->size);
	head->size -= entry->size;

	entry->prev->next = entry->next;
	entry->next->prev = entry->prev;
	entry->next = NULL;
	entry->prev = NULL;
}

static void list_add_global_before(struct global_node *entry,
				   struct global_node *list)
{
	entry->g_next = list;
	entry->g_prev = list->g_prev;
	entry->g_next->g_prev = entry;
	entry->g_prev->g_next = entry;
}

static void list_add_global_after(struct global_node *entry,
				  struct global_node *list)
{
	entry->g_prev = list;
	entry->g_next = list->g_next;
	entry->g_next->g_prev = entry;
	entry->g_prev->g_next = entry;
}

static void list_remove_global(struct global_node *entry)
{
	entry->g_prev->g_next = entry->g_next;
	entry->g_next->g_prev = entry->g_prev;
	entry->g_next = NULL;
	entry->g_prev = NULL;
}

/*****************************************************************/

static void *allocate_free_area(free_node * nptr, unsigned int size)
{
	allocated_node *anptr = NULL;
	int free_area_available;

	free_area_available = nptr->size;

	if (nptr->size == size) {
		anptr = (allocated_node *)nptr;
		anptr->tag = 1;
		list_remove_free_area(nptr);
		list_add_allocated_area(anptr);

		return anptr;
	}

	if (free_area_available > size) {
		anptr = (allocated_node *)nptr;

		if ((free_area_available - size) < SPLIT_THREASHOLD) {
			/* do not split the free area */
			anptr->tag = 1;
			list_remove_free_area(nptr);
			list_add_allocated_area(anptr);
		} else {
			anptr = kmalloc(sizeof(allocated_node), GFP_KERNEL);
			if (!anptr) {
				printk(KERN_ERR "%s %d: kmalloc failed\n",
				       __func__, __LINE__);
				return NULL;
			}

			list_remove_free_area(nptr);

			anptr->address = nptr->address;
			anptr->size = size;
			anptr->tag = 1;
			nptr->address = nptr->address + size;
			nptr->size = free_area_available - size;
			nptr->tag = 0;

			list_add_free_area(nptr);
			list_add_allocated_area(anptr);
			list_add_global_before(&anptr->g_entry, &nptr->g_entry);
		}
	}

	return anptr;
}

void *do_align_valloc(free_node * ptr, unsigned long align, unsigned long size)
{
	allocated_node *anptr = NULL;
	free_node *fnptr = NULL;
	void *align_addr, *addr, *end_addr;

	if (ptr->size < size)
		return NULL;

	addr = (void *)(ptr->address);
	align_addr = (void *)((unsigned long)addr & (~(align - 1)));

	if (!addr || (align_addr && (align_addr == addr))) {
		anptr = allocate_free_area(ptr, size);
	} else {
		align_addr =
		    (void *)(((unsigned long)addr + align) & (~(align - 1)));
		end_addr = (void *)ptr->address + ptr->size;
		if ((align_addr < end_addr) &&
			(align_addr + size <= end_addr)) {
			anptr = kmalloc(sizeof(allocated_node), GFP_KERNEL);
			if (!anptr) {
				printk(KERN_ERR "%s %d: kmalloc failed\n",
				       __func__, __LINE__);
				return NULL;
			}

			anptr->address = (unsigned long)align_addr;
			anptr->size = size;
			anptr->tag = 1;

			/* chech if there is lower free area */
			if (align_addr + size != end_addr) {
				fnptr = kmalloc(sizeof(free_node), GFP_KERNEL);
				if (!fnptr) {
					kfree(anptr);
					printk(KERN_ERR "kmalloc failed\n");
					return NULL;
				}

				fnptr->address =
				    (unsigned long)align_addr + size;
				fnptr->size = end_addr - (align_addr + size);
				fnptr->tag = 0;
			}

			list_remove_free_area(ptr);

			ptr->size = ptr->size - anptr->size;
			if (fnptr)
				ptr->size = ptr->size - fnptr->size;

			list_add_free_area(ptr);
			list_add_allocated_area(anptr);
			list_add_global_after(&anptr->g_entry, &ptr->g_entry);

			if (fnptr) {
				list_add_free_area(fnptr);
				list_add_global_after(&fnptr->g_entry,
						      &anptr->g_entry);
			}
		}
	}

	return anptr;
}

static int try_merge_upward(struct node *curr_node)
{
	struct node *prev_node;
	int ret = 0;

	if (curr_node->g_entry.g_prev != &global_list_head) {
		prev_node =
		    (void *)(curr_node->g_entry.g_prev) - OFFSET_OF(node);

		if (prev_node->tag == 0) {
			ret = 1;
			list_remove_free_area(prev_node);
			prev_node->size += curr_node->size;
			list_add_free_area(prev_node);
		}
	}

	return ret;
}

static void try_merge_downward(struct node *curr_node)
{
	struct node *next_node;

	if (curr_node->g_entry.g_next != &global_list_head) {
		next_node =
		    (void *)(curr_node->g_entry.g_next) - OFFSET_OF(node);
		if (next_node->tag == 0) {
			curr_node->size += next_node->size;
			list_remove_free_area(next_node);
			list_remove_global(&next_node->g_entry);
			kfree(next_node);
		}
	}
}

static void initialize_free_area_lists(void *start_address)
{
	int i;

	free_node *nptr = kmalloc(sizeof(free_node), GFP_KERNEL);

	for (i = 0; i <= NUMBER_OF_LISTS; i++) {
		free_area_lists[i].address = 0;
		free_area_lists[i].size = 0;
		free_area_lists[i].next = &free_area_lists[i];
		free_area_lists[i].prev = &free_area_lists[i];
	}

	nptr->address = (unsigned long)start_address;
	nptr->size = shm_size;
	nptr->tag = 0;

	list_add_free_area(nptr);
	list_add_global_before(&nptr->g_entry, &global_list_head);
}

static void initialize_address_lists(void)
{
	int i;

	for (i = 0; i <= NUMBER_OF_LISTS; i++) {
		allocated_area_lists[i].address = 0;
		allocated_area_lists[i].size = 0;
		allocated_area_lists[i].next = &allocated_area_lists[i];
		allocated_area_lists[i].prev = &allocated_area_lists[i];
	}
}

static void initialize_global_list(struct global_node *global_list_head)
{
	global_list_head->g_next = global_list_head;
	global_list_head->g_prev = global_list_head;
}

static void initialize(unsigned long paddr, unsigned long size)
{
	psa = (void *)paddr;
	shm_size = size;

	initialize_global_list(&global_list_head);
	initialize_free_area_lists(psa);
	initialize_address_lists();
}

static void *__alloc(unsigned long size)
{
	allocated_node *allocated_node = NULL;
	free_node *ptr, *tptr = NULL;
	free_node *head;

	if (!size) {
		printk(KERN_WARNING
		       "Allocation size must be greater then zero\n");
		return NULL;
	}

	head = get_free_area_list(size);

	if (head) {
		ptr = head->next;
		tptr = head;

		do {
			if (ptr->size >= size) {
				allocated_node = allocate_free_area(ptr, size);
				break;
			} else {
				if (tptr == head)
					tptr = head->next;
				else if (tptr->size < ptr->size)
					tptr = ptr;

				ptr = ptr->next;
			}
		} while (ptr != head);
	}

	if (allocated_node) {
		if (tptr != head) {
			list_remove_free_area(tptr);
			list_add_free_area(tptr);
		}
		return (void *)(allocated_node->address);
	}

	return NULL;
}

static void *__memalign(unsigned long size, unsigned long align)
{
	allocated_node *allocated_node = NULL;
	free_node *ptr, *tptr = NULL;
	free_node *head;
	int index = -1;

	if (!size) {
		printk("Allocation size must be greater then zero\n");
		return NULL;
	}

	if (!align)
		return __alloc(size);

	head = get_free_area_list(size);

	if (head) {
		do {
			ptr = head->next;
			tptr = head;
			do {
				allocated_node =
				     do_align_valloc(ptr, align, size);
				if (allocated_node)
					break;
				else {
					if (tptr == head)
						tptr = head->next;
					else if (tptr->size < ptr->size)
						tptr = ptr;

					ptr = ptr->next;
				}
			} while (ptr != head);

			if (allocated_node) {
				if (tptr != head) {
					list_remove_free_area(tptr);
					list_add_free_area(tptr);
				}

				return (void *)(allocated_node->address);
			}

			head = head + 1;
			index =
			    ((unsigned long)head -
			     (unsigned long)&free_area_lists) /
			    sizeof(free_node);
		} while (index < NUMBER_OF_LISTS + 1);
	}

	return NULL;
}

static void __free(void *ptr)
{
	allocated_node *list_head;
	allocated_node *anptr;
	int merged = 0;

	list_head = get_allocated_list_from_address((unsigned long)ptr);

	anptr = list_head->next;

	while (anptr != list_head) {
		if (anptr->address == (unsigned long)ptr)
			break;

		anptr = anptr->next;
	}

	if (anptr->address == (unsigned long)ptr) {
		list_remove_allocated_area(anptr);
		anptr->tag = 0;
		try_merge_downward(anptr);
		merged = try_merge_upward(anptr);

		if (merged) {
			list_remove_global(&anptr->g_entry);
			kfree(anptr);
		} else
			list_add_free_area((free_node *) anptr);
	} else
		printk(KERN_WARNING "%s: Trying to free %p invalid address\n",
		       __func__, ptr);
}

static void *fsl_shm_alloc(unsigned long size)
{
	void *ptr;

	spin_lock(&mm_lock);
	ptr = __alloc(size);
	spin_unlock(&mm_lock);

	return ptr;
}

static void *fsl_shm_memalign(unsigned long size, unsigned long align)
{
	void *ptr;

	spin_lock(&mm_lock);
	ptr = __memalign(size, align);
	spin_unlock(&mm_lock);

	return ptr;
}

static void fsl_shm_free(void *ptr)
{
	spin_lock(&mm_lock);
	if (ptr)
		__free(ptr);
	spin_unlock(&mm_lock);
}

static int fsl_shm_get_paddr(shm_seg_t *mem)
{
	int ret = 0;
	struct page *pg, *pages = NULL;
	struct vm_area_struct *vmas = NULL;
	int npages;

	down_read(&current->mm->mmap_sem);
	ret =
	    get_user_pages(current, current->mm, (unsigned long)mem->vaddr, 1,
			   0, 0, &pages, &vmas);
	up_read(&current->mm->mmap_sem);
	if (ret > 0) {
		npages = ret;
		pg = &pages[0];
		mem->paddr = (void *)page_to_phys(pg);
		put_page(pg);
	}

	return 0;
}

#ifdef HAVE_UNLOCKED_IOCTL
static long fsl_shm_ioctl(struct file *file, unsigned int cmd,
		unsigned long arg)
#else
static int fsl_shm_ioctl(struct inode *inode, struct file *file,
		unsigned int cmd, unsigned long arg)
#endif
{
	shm_seg_t seg;
	alloc_req_t mem;
	memalign_req_t memalign;
	unsigned long addr;
	static int initialized = -1;
	int ret = 1;

	switch (cmd) {
	case IOCTL_FSL_SHM_INIT:
		if (copy_from_user(&seg, (shm_seg_t *) arg,
		    sizeof(shm_seg_t))) {
			ret = -EFAULT;
			break;
		}

		fsl_shm_get_paddr(&seg);

		spin_lock(&mm_lock);
		if (initialized) {
			initialized++;
			initialize((unsigned long)seg.paddr, seg.size);
		}
		spin_unlock(&mm_lock);

		if (copy_to_user((shm_seg_t *) arg, &seg, sizeof(shm_seg_t)))
			ret = -EFAULT;
		break;

	case IOCTL_FSL_SHM_ALLOC:
		if (copy_from_user(&mem, (alloc_req_t *) arg,
		    sizeof(alloc_req_t))) {
			ret = -EFAULT;
			break;
		}

		mem.paddr = fsl_shm_alloc(mem.size);
		if (copy_to_user((alloc_req_t *) arg, &mem,
		    sizeof(alloc_req_t)))
			ret = -EFAULT;
		break;

	case IOCTL_FSL_SHM_MEMALIGN:
		if (copy_from_user(&memalign, (memalign_req_t *) arg,
		    sizeof(memalign_req_t))) {
			ret = -EFAULT;
			break;
		}

		memalign.paddr =
		    fsl_shm_memalign(memalign.size, memalign.align);
		if (copy_to_user((memalign_req_t *) arg, &memalign,
		    sizeof(memalign_req_t)))
			ret = -EFAULT;
		break;

	case IOCTL_FSL_SHM_FREE:
		if (copy_from_user(&addr, (void *)arg, sizeof(addr))) {
			ret = -EFAULT;
			break;
		}

		fsl_shm_free((void *)addr);
		break;

	default:
		break;
	}

	return ret;
}

static const struct file_operations fsl_shm_fops = {
	.owner = THIS_MODULE,
#ifdef HAVE_UNLOCKED_IOCTL
	.unlocked_ioctl = fsl_shm_ioctl,
#else
	.ioctl = fsl_shm_ioctl,
#endif
};

static __init int fsl_shm_init(void)
{
	int ret;

	fsl_shm_major = 0;
	ret = register_chrdev(fsl_shm_major, "fsl_shm", &fsl_shm_fops);
	if (ret < 0) {
		printk(KERN_ERR "%s: Can't register character device\n",
			__func__);
		return ret;
	}

	fsl_shm_major = ret;
	spin_lock_init(&mm_lock);

	printk(KERN_INFO "fsl_shm module installed successfully with"
	       "major num : %d\n", fsl_shm_major);

	return 0;
}

static void __exit fsl_shm_exit(void)
{
	unregister_chrdev(fsl_shm_major, "fsl_shm");
}

module_init(fsl_shm_init);
module_exit(fsl_shm_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("FSL Shared Memory Manager");
MODULE_AUTHOR("naveenburmi@freescale.com");
