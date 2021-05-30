/*
 * Copyright (C) 2019-2021 Corellium LLC
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/version.h>
#include <linux/kallsyms.h>

#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/init.h>
#include <linux/printk.h>
#include <linux/mm.h>
#include <linux/io.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/interrupt.h>

#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/poll.h>

#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/acpi.h>

#include <asm/io.h>
#include <asm/cacheflush.h>
#include <asm/uaccess.h>

#define D0MAP_HVCID             0xDE19
#define D0MAP_HVC_PROBE         0       /*                         -> X0:instnum */
#define D0MAP_HVC_GET_BASE      1       /* X1:inst                 -> X0:base */
#define D0MAP_HVC_GET_SIZE      2       /* X1:inst                 -> X0:size */
#define D0MAP_HVC_ALLOC         3       /* X1:inst,X2:size,X3:puid -> X0:offset */
#define D0MAP_HVC_NOTIFY        4       /* X1:inst,X2:size         -> X0:available */
#define D0MAP_HVC_FREE          5       /* X1:inst,X2:offset       -> - */

#define D0MAP_IOC_WAIT          _IO('0', 0x01)

/* global state (should not have >1 instance of D0map per system) */
struct d0map_cdev_priv;
struct d0map_state {
    unsigned num_insts;                 /* number of instances */
    struct d0map_state_inst {
        int irq;                        /* IRQ number for this instance */
        struct device *char_dev;        /* character device for each instance */
        struct list_head waitcli;       /* list of waiting clients */
        spinlock_t waitcli_lock;
        uint64_t wait_size;             /* size for last wait notification request */
        uint64_t wait_ack;              /* size for last wait notification */
    } *inst;
    unsigned irqs_requested;

    /* character device related stuff */
    struct cdev cdev;                   /* cdev filled with our fops */
    int cdev_added;                     /* flag indicating cdev_add succeeded */
    dev_t dev_start;                    /* major & first minor allocated for us */
    struct class *dev_class;            /* device class */
} d0map;

struct d0map_wait_item {
    struct list_head item;
    uint64_t size;
    wait_queue_head_t queue;
    unsigned wake;
};

struct d0map_cdev_priv {
    unsigned inst;
    struct d0map_wait_item wait_item;
};

#define XSTR(s) STR(s)
#define STR(s) #s
u64 d0map_hvc(u64 x0, u64 x1, u64 x2, u64 x3);
__asm__(".global d0map_hvc\n"
        "d0map_hvc:\n"
        "    hvc " XSTR(D0MAP_HVCID) "\n"
        "    ret");

static void d0map_update_waiters(unsigned inst)
{
    struct d0map_wait_item *item, *temp;
    uint64_t min_size, res;

    while(1) {
        min_size = -1ul;
        list_for_each_entry_safe(item, temp, &d0map.inst[inst].waitcli, item) {
            if(item->size <= d0map.inst[inst].wait_ack) {
                list_del(&item->item);
                item->wake = 1;
                wake_up_interruptible(&item->queue);
            } else
                if(item->size < min_size)
                    min_size = item->size;
        }
        d0map.inst[inst].wait_ack = 0;

        d0map.inst[inst].wait_size = min_size;
        res = d0map_hvc(D0MAP_HVC_NOTIFY, inst, min_size, 0);
        if(!res)
            break;
        d0map.inst[inst].wait_ack = min_size;
    }
}

static int d0map_cdev_open(struct inode *in, struct file *f)
{
    int inst = in->i_rdev - d0map.dev_start;
    struct d0map_cdev_priv *priv;

    if(inst < 0 || inst >= d0map.num_insts)
        return -EINVAL;

    priv = kmalloc(sizeof(struct d0map_cdev_priv), GFP_KERNEL);
    if(!priv)
        return -ENOMEM;

    priv->inst = inst;
    priv->wait_item.wake = 1;
    priv->wait_item.size = -1ul;
    init_waitqueue_head(&priv->wait_item.queue);

    f->private_data = priv;
    return 0;
}

static int d0map_cdev_release(struct inode *in, struct file *f)
{
    struct d0map_cdev_priv *priv = f->private_data;
    unsigned long flags;

    spin_lock_irqsave(&d0map.inst[priv->inst].waitcli_lock, flags);
    smp_mb();
    if(!priv->wait_item.wake)
        list_del(&priv->wait_item.item);
    smp_mb();
    spin_unlock_irqrestore(&d0map.inst[priv->inst].waitcli_lock, flags);

    kfree(priv);
    return 0;
}

static ssize_t d0map_cdev_write(struct file *f, const char __user *buf, size_t count, loff_t *ppos)
{
    struct d0map_cdev_priv *priv = f->private_data;
    unsigned long flags;
    size_t done = 0;
    u64 size = -1ul;

    if(count % sizeof(u64))
        return -EINVAL;

    while(done < count) {
        if(get_user(size, (u64 __user *)(buf + done)))
            return -EFAULT;
        done += sizeof(u64);
    }

    spin_lock_irqsave(&d0map.inst[priv->inst].waitcli_lock, flags);
    smp_mb();
    priv->wait_item.size = size;
    if(priv->wait_item.wake) {
        priv->wait_item.wake = 0;
        list_add_tail(&priv->wait_item.item, &d0map.inst[priv->inst].waitcli);
    }
    d0map_update_waiters(priv->inst);
    smp_mb();
    spin_unlock_irqrestore(&d0map.inst[priv->inst].waitcli_lock, flags);

    return done;
}

static ssize_t d0map_cdev_read(struct file *f, char __user *buf, size_t count, loff_t *ppos)
{
    struct d0map_cdev_priv *priv = f->private_data;
    u64 result;

    if(count < sizeof(result))
        return -EINVAL;

    while(1) {
        if((volatile unsigned)priv->wait_item.wake)
            break;

        if(f->f_flags & O_NONBLOCK)
            return -EAGAIN;

        if(wait_event_interruptible(priv->wait_item.queue, priv->wait_item.wake))
            return -ERESTARTSYS;
    }

    result = priv->wait_item.size;
    if(put_user((u64)result, (u64 __user *)buf))
        return -EFAULT;
    return sizeof(result);
}

static unsigned d0map_cdev_poll(struct file *f, poll_table *wait)
{
    struct d0map_cdev_priv *priv = f->private_data;

    poll_wait(f, &priv->wait_item.queue, wait);
    if((volatile unsigned)priv->wait_item.wake)
        return POLLOUT | POLLWRNORM | POLLIN | POLLRDNORM;

    return POLLOUT | POLLWRNORM;
}

static int d0map_cdev_mmap(struct file *f, struct vm_area_struct *vma)
{
    struct d0map_cdev_priv *priv = f->private_data;
    u64 offset = vma->vm_pgoff << PAGE_SHIFT;
    u64 size = vma->vm_end - vma->vm_start;
    u64 map_ipa, map_size;

    map_ipa = d0map_hvc(D0MAP_HVC_GET_BASE, priv->inst, 0, 0);
    map_size = d0map_hvc(D0MAP_HVC_GET_SIZE, priv->inst, 0, 0);

    if(size > map_size || offset >= map_size || offset + size > map_size)
        return -EINVAL;
    return remap_pfn_range(vma, vma->vm_start, (map_ipa + offset) >> PAGE_SHIFT, size, vma->vm_page_prot);
}

static long d0map_cdev_ioctl(struct file *f, unsigned int cmd, unsigned long arg)
{
    struct d0map_cdev_priv *priv = f->private_data;
    struct d0map_wait_item wait_item;
    unsigned long flags;

    if(cmd != D0MAP_IOC_WAIT)
        return -EINVAL;

    spin_lock_irqsave(&d0map.inst[priv->inst].waitcli_lock, flags);
    smp_mb();
    init_waitqueue_head(&wait_item.queue);
    wait_item.size = arg;
    wait_item.wake = 0;
    list_add_tail(&wait_item.item, &d0map.inst[priv->inst].waitcli);
    d0map_update_waiters(priv->inst);
    smp_mb();
    spin_unlock_irqrestore(&d0map.inst[priv->inst].waitcli_lock, flags);

    while(1) {
        if((volatile unsigned)wait_item.wake)
            break;

        if(wait_event_interruptible(wait_item.queue, wait_item.wake)) {
            spin_lock_irqsave(&d0map.inst[priv->inst].waitcli_lock, flags);
            smp_mb();
            if(!wait_item.wake)
                list_del(&wait_item.item);
            smp_mb();
            spin_unlock_irqrestore(&d0map.inst[priv->inst].waitcli_lock, flags);
            return -ERESTARTSYS;
        }
    }

    return 0;
}

static const struct file_operations d0map_cdev_fileops = {
    .owner = THIS_MODULE,
    .open = d0map_cdev_open,
    .release = d0map_cdev_release,
    .read = d0map_cdev_read,
    .write = d0map_cdev_write,
    .poll = d0map_cdev_poll,
    .mmap = d0map_cdev_mmap,
    .unlocked_ioctl = d0map_cdev_ioctl,
    .compat_ioctl = d0map_cdev_ioctl,
};

/* interrupt handling */

static irqreturn_t d0map_irq_handler(int irq, void *dev_id)
{
    unsigned inst = (struct d0map_state_inst *)dev_id - d0map.inst;
    unsigned long flags;

    if(inst >= d0map.num_insts)
        return IRQ_NONE;

    spin_lock_irqsave(&d0map.inst[inst].waitcli_lock, flags);
    smp_mb();
    d0map.inst[inst].wait_ack = d0map.inst[inst].wait_size;
    d0map_update_waiters(inst);
    smp_mb();
    spin_unlock_irqrestore(&d0map.inst[inst].waitcli_lock, flags);

    return IRQ_HANDLED;
}

/* probe/remove of device */

static int d0map_remove(struct platform_device *pdev);

static int d0map_probe(struct platform_device *pdev)
{
    struct device *dev = &pdev->dev;
    unsigned i;
    int ret = 0;

    d0map.num_insts = d0map_hvc(D0MAP_HVC_PROBE, 0, 0, 0);

    /* show some statistics */
    dev_info(dev, "D0map: %u instance%s, IOC_WAIT supported.\n", d0map.num_insts, d0map.num_insts != 1 ? "s" : "");

    /* create subdevice array */
    d0map.inst = kmalloc(sizeof(d0map.inst[0]) * d0map.num_insts, GFP_KERNEL);
    if(!d0map.inst) {
        ret = -ENOMEM;
        goto fail;
    }
    memset(d0map.inst, 0, sizeof(d0map.inst[0]) * d0map.num_insts);

    /* map IRQs */
    for(i=0; i<d0map.num_insts; i++) {
        d0map.inst[i].irq = platform_get_irq(pdev, i);
        if((ret = request_irq(d0map.inst[i].irq, d0map_irq_handler, IRQF_SHARED, dev_name(dev), d0map.inst + i))) {
            dev_err(dev, "Failed to request IRQ for instance %d.\n", i);
            goto fail;
        }
        d0map.irqs_requested = i + 1;
        INIT_LIST_HEAD(&d0map.inst[i].waitcli);
        spin_lock_init(&d0map.inst[i].waitcli_lock);
    }

    /* create character devices */
    if(!(d0map.dev_class = class_create(THIS_MODULE, "d0map"))) {
        dev_err(dev, "Could not create device class\n");
        goto fail;
    }

    if((ret = alloc_chrdev_region(&d0map.dev_start, 0, d0map.num_insts, "d0map"))) {
        dev_err(dev, "Failed to allocate %d character devices.\n", d0map.num_insts);
        goto fail;
    }

    cdev_init(&d0map.cdev, &d0map_cdev_fileops);
    d0map.cdev.owner = THIS_MODULE;
    if((ret = cdev_add(&d0map.cdev, d0map.dev_start, d0map.num_insts))) {
        dev_err(dev, "Failed to add cdev structure.\n");
        goto fail;
    }
    d0map.cdev_added = 1;

    for(i=0; i<d0map.num_insts; i++) {
        d0map.inst[i].char_dev = device_create(d0map.dev_class, dev, d0map.dev_start + i, d0map.inst + i, "d0map%d", i);
        if(!d0map.inst[i].char_dev) {
            dev_err(dev, "Failed to create character device for instance %d.\n", i);
            ret = -ENOMEM;
            goto fail;
        }
    }

    return 0;

fail:
    d0map_remove(pdev);
    return ret;
}

static int d0map_remove(struct platform_device *pdev)
{
    unsigned i;

    if(d0map.inst) {
        /* release individual character devices */
        for(i=0; i<d0map.num_insts; i++)
            if(d0map.inst[i].char_dev) {
                device_destroy(d0map.dev_class, d0map.dev_start + i);
                d0map.inst[i].char_dev = NULL;
            }

        /* release IRQs */
        for(i=0; i<d0map.irqs_requested; i++)
            free_irq(d0map.inst[i].irq, d0map.inst + i);
        d0map.irqs_requested = 0;

        kfree(d0map.inst);
        d0map.inst = NULL;
    }

    /* free shared character device structures */
    if(d0map.dev_class) {
        class_destroy(d0map.dev_class);
        d0map.dev_class = NULL;
    }
    if(d0map.dev_start) {
        unregister_chrdev_region(d0map.dev_start, d0map.num_insts);
        d0map.dev_start = 0;
    }
    if(d0map.cdev_added) {
        cdev_del(&d0map.cdev);
        d0map.cdev_added = 0;
    }

    return 0;
}

/* structures that bind us to the device tree entry inserted by CHARM */

static const struct of_device_id d0map_of_device_id_table[] = {
    { .compatible = "corellium,d0map-1", },
    { },
};
MODULE_DEVICE_TABLE(of, d0map_of_device_id_table);

static const struct acpi_device_id d0map_acpi_device_id_table[] = {
    { "CRLMD001", 0 },
    {},
};
MODULE_DEVICE_TABLE(acpi, d0map_acpi_device_id_table);

static struct platform_driver d0map_platform_driver = {
    .driver = {
        .name = "corellium-d0map",
        .owner = THIS_MODULE,
        .of_match_table = d0map_of_device_id_table,
        .acpi_match_table = ACPI_PTR(d0map_acpi_device_id_table),
    },
    .probe = d0map_probe,
    .remove = d0map_remove,
};
module_platform_driver(d0map_platform_driver);

MODULE_DESCRIPTION("Corellium Dom0 Mapper");
MODULE_AUTHOR("Stan Skowronek, Corellium");
MODULE_LICENSE("GPL v2");
