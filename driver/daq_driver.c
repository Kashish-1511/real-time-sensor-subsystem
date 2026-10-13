#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/wait.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>

#include "daq_driver.h"

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Kashish Ambaliya");
MODULE_DESCRIPTION("Industrial Real-Time Data Acquisition Character Device Driver");
MODULE_VERSION("1.0");

#define DAQ_BUFFER_CAPACITY 256

/*
 * Device Context Structure: Holds all private state for this driver instance.
 */
struct daq_device {
    dev_t dev_num;                      /* Combined Major + Minor device numbers */
    struct cdev cdev;                   /* VFS character device object */
    struct class *dev_class;            /* /sys/class/daq entry */
    struct device *dev_device;          /* /dev/rt_daq device node */

    /* Ring Buffer Storage */
    struct daq_frame buffer[DAQ_BUFFER_CAPACITY];
    uint32_t head;                      /* Write index (producer: timer callback) */
    uint32_t tail;                      /* Read index (consumer: user read call) */
    uint32_t count;                     /* Number of active items in buffer */
    spinlock_t lock;                    /* Protects buffer against concurrent access */
    wait_queue_head_t read_wait;        /* Puts reading processes to sleep until data arrives */

    /* Simulated Hardware Pulse (High-Resolution Timer) */
    struct hrtimer timer;               /* Hardware timer structure */
    ktime_t sample_period;              /* Interval between samples */
    uint32_t sample_interval_us;        /* Interval in microseconds (default: 1000us = 1kHz) */
    uint32_t sequence_counter;          /* Incrementing packet counter */
    bool streaming_active;              /* Stream run/stop state */

    /* Diagnostics Telemetry */
    struct daq_stats stats;
};

/* Single global instance for our device */
static struct daq_device daq_dev;

/*
 * TIMER CALLBACK: Simulated Hardware Interrupt
 * Runs in Atomic / Interrupt Context! No sleeping or memory allocations allowed here.
 */
static enum hrtimer_restart daq_timer_callback(struct hrtimer *timer)
{
    struct daq_device *dev = container_of(timer, struct daq_device, timer);
    unsigned long flags;
    struct daq_frame frame;

    if (!dev->streaming_active)
        return HRTIMER_NORESTART;

    /* 1. Generate the simulated sensor frame */
    frame.timestamp_ns = ktime_get_real_ns();
    frame.sequence_id  = dev->sequence_counter++;
    frame.channel_id   = 0;
    frame.status_flags = 0x00;

    /* Generate a realistic sine-wave pattern or incremental sensor signal */
    frame.raw_value = (int32_t)((frame.sequence_id % 100) * 10);
    frame.checksum  = (uint32_t)(frame.raw_value ^ frame.sequence_id);

    /* 2. Acquire spinlock and disable local CPU interrupts */
    spin_lock_irqsave(&dev->lock, flags);

    dev->stats.total_samples_generated++;

    if (dev->count < DAQ_BUFFER_CAPACITY) {
        /* Buffer has space: push frame */
        dev->buffer[dev->head] = frame;
        dev->head = (dev->head + 1) % DAQ_BUFFER_CAPACITY;
        dev->count++;
    } else {
        /* Buffer overflow: overwrite oldest and increment dropped counter */
        dev->stats.samples_dropped++;
        frame.status_flags |= 0x01; /* Flag: Buffer Overrun */
        dev->buffer[dev->head] = frame;
        dev->head = (dev->head + 1) % DAQ_BUFFER_CAPACITY;
        dev->tail = (dev->tail + 1) % DAQ_BUFFER_CAPACITY;
    }

    dev->stats.current_buffer_fill = dev->count;
    spin_unlock_irqrestore(&dev->lock, flags);

    /* 3. Wake up any user-space thread waiting inside read() */
    wake_up_interruptible(&dev->read_wait);

    /* 4. Forward timer to the next periodic deadline */
    hrtimer_forward_now(timer, dev->sample_period);
    return HRTIMER_RESTART;
}

/*
 * VFS: open()
 */
static int daq_open(struct inode *inodep, struct file *filep)
{
    /* Associate device context with file pointer */
    filep->private_data = &daq_dev;
    return 0;
}

/*
 * VFS: release() / close()
 */
static int daq_release(struct inode *inodep, struct file *filep)
{
    return 0;
}

/*
 * VFS: read() - Blocking read of one daq_frame
 */
static ssize_t daq_read(struct file *filep, char __user *user_buf, size_t len, loff_t *offset)
{
    struct daq_device *dev = (struct daq_device *)filep->private_data;
    struct daq_frame frame;
    unsigned long flags;
    int ret;

    /* Caller must request at least one complete struct daq_frame */
    if (len < sizeof(struct daq_frame))
        return -EINVAL;

    /*
     * 1. Sleep until data is available in the buffer.
     * If O_NONBLOCK is set on open, return immediately if empty.
     */
    if (filep->f_flags & O_NONBLOCK) {
        spin_lock_irqsave(&dev->lock, flags);
        if (dev->count == 0) {
            spin_unlock_irqrestore(&dev->lock, flags);
            return -EAGAIN;
        }
        spin_unlock_irqrestore(&dev->lock, flags);
    } else {
        ret = wait_event_interruptible(dev->read_wait, (dev->count > 0));
        if (ret != 0)
            return -ERESTARTSYS; /* Interrupted by signal (e.g. Ctrl+C) */
    }

    /* 2. Pop one frame from the ring buffer */
    spin_lock_irqsave(&dev->lock, flags);
    if (dev->count == 0) {
        spin_unlock_irqrestore(&dev->lock, flags);
        return -EAGAIN;
    }

    frame = dev->buffer[dev->tail];
    dev->tail = (dev->tail + 1) % DAQ_BUFFER_CAPACITY;
    dev->count--;
    dev->stats.samples_read_by_user++;
    dev->stats.current_buffer_fill = dev->count;

    spin_unlock_irqrestore(&dev->lock, flags);

    /* 3. Safely transfer frame from kernel memory to user-space pointer */
    if (copy_to_user(user_buf, &frame, sizeof(struct daq_frame)))
        return -EFAULT;

    return sizeof(struct daq_frame);
}

/*
 * VFS: unlocked_ioctl() - Hardware command interface
 */
static long daq_ioctl(struct file *filep, unsigned int cmd, unsigned long arg)
{
    struct daq_device *dev = (struct daq_device *)filep->private_data;
    unsigned long flags;
    uint32_t val;
    struct daq_stats stats_copy;

    switch (cmd) {
    case DAQ_IOCTL_RESET_BUFFER:
        spin_lock_irqsave(&dev->lock, flags);
        dev->head = 0;
        dev->tail = 0;
        dev->count = 0;
        dev->stats.current_buffer_fill = 0;
        spin_unlock_irqrestore(&dev->lock, flags);
        pr_info("rt_daq: Buffer reset successfully\n");
        break;

    case DAQ_IOCTL_SET_SAMPLE_US:
        if (copy_from_user(&val, (uint32_t __user *)arg, sizeof(uint32_t)))
            return -EFAULT;
        if (val < 100 || val > 1000000) /* Limit: 100us (10kHz) to 1s (1Hz) */
            return -EINVAL;

        dev->sample_interval_us = val;
        dev->sample_period = ktime_set(0, val * 1000);
        pr_info("rt_daq: Sample interval set to %u us\n", val);
        break;

    case DAQ_IOCTL_STREAM_CONTROL:
        if (copy_from_user(&val, (uint32_t __user *)arg, sizeof(uint32_t)))
            return -EFAULT;

        if (val == 1 && !dev->streaming_active) {
            dev->streaming_active = true;
            hrtimer_start(&dev->timer, dev->sample_period, HRTIMER_MODE_REL);
            pr_info("rt_daq: Data stream started\n");
        } else if (val == 0 && dev->streaming_active) {
            dev->streaming_active = false;
            hrtimer_cancel(&dev->timer);
            pr_info("rt_daq: Data stream stopped\n");
        }
        break;

    case DAQ_IOCTL_GET_STATS:
        spin_lock_irqsave(&dev->lock, flags);
        stats_copy = dev->stats;
        spin_unlock_irqrestore(&dev->lock, flags);

        if (copy_to_user((struct daq_stats __user *)arg, &stats_copy, sizeof(struct daq_stats)))
            return -EFAULT;
        break;

    default:
        return -ENOTTY; /* Unknown command */
    }

    return 0;
}

/*
 * File Operations Dispatch Table
 */
static const struct file_operations daq_fops = {
    .owner          = THIS_MODULE,
    .open           = daq_open,
    .release        = daq_release,
    .read           = daq_read,
    .unlocked_ioctl = daq_ioctl,
};

/*
 * MODULE INIT: Runs when insmod is called
 */
static int __init daq_init(void)
{
    int ret;

    /* 1. Allocate Major & Minor numbers dynamically */
    ret = alloc_chrdev_region(&daq_dev.dev_num, 0, 1, DAQ_DEVICE_NAME);
    if (ret < 0) {
        pr_err("rt_daq: Failed to allocate major number\n");
        return ret;
    }

    /* 2. Initialize VFS character device object */
    cdev_init(&daq_dev.cdev, &daq_fops);
    daq_dev.cdev.owner = THIS_MODULE;
    ret = cdev_add(&daq_dev.cdev, daq_dev.dev_num, 1);
    if (ret < 0) {
        pr_err("rt_daq: Failed to add cdev\n");
        goto unregister_chrdev;
    }

    /* 3. Create sysfs class (/sys/class/daq) */
    daq_dev.dev_class = class_create(DAQ_CLASS_NAME);
    if (IS_ERR(daq_dev.dev_class)) {
        pr_err("rt_daq: Failed to create device class\n");
        ret = PTR_ERR(daq_dev.dev_class);
        goto delete_cdev;
    }

    /* 4. Create device node (/dev/rt_daq) with udev */
    daq_dev.dev_device = device_create(daq_dev.dev_class, NULL, daq_dev.dev_num, NULL, DAQ_DEVICE_NAME);
    if (IS_ERR(daq_dev.dev_device)) {
        pr_err("rt_daq: Failed to create device node\n");
        ret = PTR_ERR(daq_dev.dev_device);
        goto destroy_class;
    }

    /* 5. Initialize synchronization primitives */
    spin_lock_init(&daq_dev.lock);
    init_waitqueue_head(&daq_dev.read_wait);

    /* 6. Setup default state and timer */
    daq_dev.head = 0;
    daq_dev.tail = 0;
    daq_dev.count = 0;
    daq_dev.sequence_counter = 0;
    daq_dev.sample_interval_us = 1000; /* 1000us = 1ms = 1kHz default */
    daq_dev.sample_period = ktime_set(0, 1000 * 1000);
    daq_dev.streaming_active = false;

    daq_dev.stats.total_samples_generated = 0;
    daq_dev.stats.samples_read_by_user = 0;
    daq_dev.stats.samples_dropped = 0;
    daq_dev.stats.current_buffer_fill = 0;
    daq_dev.stats.max_buffer_capacity = DAQ_BUFFER_CAPACITY;

    hrtimer_init(&daq_dev.timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    daq_dev.timer.function = daq_timer_callback;

    pr_info("rt_daq: Module loaded successfully. Created /dev/%s\n", DAQ_DEVICE_NAME);
    return 0;

destroy_class:
    class_destroy(daq_dev.dev_class);
delete_cdev:
    cdev_del(&daq_dev.cdev);
unregister_chrdev:
    unregister_chrdev_region(daq_dev.dev_num, 1);
    return ret;
}

/*
 * MODULE EXIT: Runs when rmmod is called (Reverse Order Cleanup)
 */
static void __exit daq_exit(void)
{
    /* 1. Stop timer if still running */
    if (daq_dev.streaming_active) {
        daq_dev.streaming_active = false;
        hrtimer_cancel(&daq_dev.timer);
    }

    /* 2. Destroy device node and class */
    device_destroy(daq_dev.dev_class, daq_dev.dev_num);
    class_destroy(daq_dev.dev_class);

    /* 3. Remove cdev and free device numbers */
    cdev_del(&daq_dev.cdev);
    unregister_chrdev_region(daq_dev.dev_num, 1);

    pr_info("rt_daq: Module unloaded cleanly\n");
}

module_init(daq_init);
module_exit(daq_exit);
