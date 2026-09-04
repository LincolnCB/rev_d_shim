// SPDX-License-Identifier: GPL-2.0
//
// pl-irq-shim -- non-root notification when a rev_d_shim PL interrupt fires.
// Project-specific copy of the ex04 pl-irq driver: same mechanism (bind a
// hand-written interrupt node by a private compatible, publish it as
// /dev/<Vivado-label> at mode 0666, deliver via poll()/read() with write() to
// re-arm), wired to this project's hw_manager interrupt.
//
// The interrupt sibling of pl-reg. It lets userspace wait for a PL interrupt
// without root: open the device, block in poll()/read() until the interrupt
// fires, then write() to re-arm it -- the small slice of UIO that programs
// actually use.
//
// Binding: the kernel already ships uio_pdrv_genirq for this, but it only binds
// to a node matching its `of_id` module parameter, which means adding
// uio_pdrv_genirq.of_id="generic-uio" to the kernel command line -- a setting
// that lives apart from the block design and is easy to lose when the bootargs
// are regenerated. pl-irq instead carries its own of_match_table and binds by a
// private compatible ("zynq-toolbox,pl-irq-shim") straight from the project's
// device tree, with no command-line change. ex04 documents the generic-uio
// route as an alternative. Each /dev entry is named after the node's Vivado
// label (hw_manager_irq -> /dev/hw_manager_irq), same as pl-reg.
//
// Non-root: the /dev entry is a miscdevice created world-accessible (0666), so
// an ordinary user can open it with no root and no udev rule -- unlike the
// in-tree generic-uio, whose /dev/uioN is root-owned 0600 and would need a
// boot-time chmod.
//
// Semantics match uio_pdrv_genirq: the handler masks the IRQ on each fire (so a
// level line won't re-fire), read() returns the running event count and blocks
// until it changes, and write(1)/write(0) re-enable/disable the line. The line
// only signals that an interrupt fired, not what caused it, so userspace still
// clears the originating device's own interrupt status before re-arming. The
// userspace side is the shim control programs' interrupt monitor
// (software/*/src/sys/sys_sts.c).

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/poll.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/string.h>
#include <linux/uaccess.h>
#include <linux/wait.h>

#define DRIVER_NAME "pl-irq-shim"
#define DEV_MODE    0666               // world-accessible: no root, no udev

// One of these per bound node. `count` is the running interrupt total (the value
// read() returns); `disabled` tracks whether the IRQ is currently masked so
// enable/disable stay balanced across the handler and write().
struct pl_irq_dev {
	int                irq;
	atomic_t           count;
	wait_queue_head_t  wait;
	spinlock_t         lock;
	bool               disabled;
	struct miscdevice  misc;
	char               name[48];
};

// Per-open cursor: remembers the last count this fd saw, so each reader blocks
// until a NEW interrupt arrives (matching UIO's per-open event tracking).
struct pl_irq_listener {
	struct pl_irq_dev *pi;
	s32                last;
};

// Top-half handler: mask the IRQ so a level line will not re-fire until userspace
// re-arms, bump the count, and wake any blocked readers.
static irqreturn_t pl_irq_handler(int irq, void *data)
{
	struct pl_irq_dev *pi = data;
	unsigned long flags;

	spin_lock_irqsave(&pi->lock, flags);
	if (!pi->disabled) {
		pi->disabled = true;
		disable_irq_nosync(irq);
	}
	spin_unlock_irqrestore(&pi->lock, flags);

	atomic_inc(&pi->count);
	wake_up_interruptible(&pi->wait);
	return IRQ_HANDLED;
}

// Start each fd's cursor at the current count, so it only sees future interrupts
static int pl_irq_open(struct inode *inode, struct file *file)
{
	struct miscdevice *misc = file->private_data;
	struct pl_irq_dev *pi = container_of(misc, struct pl_irq_dev, misc);
	struct pl_irq_listener *l = kzalloc(sizeof(*l), GFP_KERNEL);

	if (!l)
		return -ENOMEM;
	l->pi = pi;
	l->last = atomic_read(&pi->count);
	file->private_data = l;
	return 0;
}

static int pl_irq_release(struct inode *inode, struct file *file)
{
	kfree(file->private_data);
	return 0;
}

// Return the running interrupt count (4 bytes). Blocks until the count changes
// since this fd last read it, unless O_NONBLOCK.
static ssize_t pl_irq_read(struct file *file, char __user *buf, size_t count,
			   loff_t *ppos)
{
	struct pl_irq_listener *l = file->private_data;
	struct pl_irq_dev *pi = l->pi;
	s32 cur;
	int ret;

	if (count < sizeof(s32))
		return -EINVAL;

	if (file->f_flags & O_NONBLOCK) {
		cur = atomic_read(&pi->count);
		if (cur == l->last)
			return -EAGAIN;
	} else {
		ret = wait_event_interruptible(pi->wait,
					       atomic_read(&pi->count) != l->last);
		if (ret)
			return ret;
		cur = atomic_read(&pi->count);
	}

	l->last = cur;
	if (copy_to_user(buf, &cur, sizeof(cur)))
		return -EFAULT;
	return sizeof(cur);
}

// Readable once a new interrupt has arrived since this fd last read
static __poll_t pl_irq_poll(struct file *file, poll_table *wait)
{
	struct pl_irq_listener *l = file->private_data;
	struct pl_irq_dev *pi = l->pi;

	poll_wait(file, &pi->wait, wait);
	if (atomic_read(&pi->count) != l->last)
		return EPOLLIN | EPOLLRDNORM;
	return 0;
}

// Arm (write 1) or mask (write 0) the IRQ, keeping enable/disable balanced
static ssize_t pl_irq_write(struct file *file, const char __user *buf,
			    size_t count, loff_t *ppos)
{
	struct pl_irq_listener *l = file->private_data;
	struct pl_irq_dev *pi = l->pi;
	unsigned long flags;
	s32 val;

	if (count < sizeof(s32))
		return -EINVAL;
	if (copy_from_user(&val, buf, sizeof(val)))
		return -EFAULT;

	spin_lock_irqsave(&pi->lock, flags);
	if (val == 1 && pi->disabled) {
		pi->disabled = false;
		enable_irq(pi->irq);
	} else if (val == 0 && !pi->disabled) {
		pi->disabled = true;
		disable_irq_nosync(pi->irq);
	}
	spin_unlock_irqrestore(&pi->lock, flags);
	return sizeof(val);
}

static const struct file_operations pl_irq_fops = {
	.owner   = THIS_MODULE,
	.open    = pl_irq_open,
	.release = pl_irq_release,
	.read    = pl_irq_read,
	.poll    = pl_irq_poll,
	.write   = pl_irq_write,
};

// Same DT-label lookup as pl-reg: in a name -> node-path directory (/aliases or
// /__symbols__), find the entry pointing back at `np` and return its name.
static const char *pl_irq_lookup_name(const char *dir_path, struct device_node *np)
{
	struct device_node *dir;
	struct property *pp;
	const char *result = NULL;

	dir = of_find_node_by_path(dir_path);
	if (!dir)
		return NULL;

	for_each_property_of_node(dir, pp) {
		struct device_node *target;

		if (!strcmp(pp->name, "name") || !strcmp(pp->name, "phandle"))
			continue;

		target = of_find_node_by_path(pp->value);
		if (target == np)
			result = pp->name;
		of_node_put(target);
		if (result)
			break;
	}

	of_node_put(dir);
	return result;
}

// Recover this node's Vivado label (e.g. "mcdma_irq") to name /dev after it
static const char *pl_irq_instance_name(struct device_node *np)
{
	const char *name;

	name = pl_irq_lookup_name("/aliases", np);
	if (!name)
		name = pl_irq_lookup_name("/__symbols__", np);
	return name;
}

// One bound node -> one /dev/<instance> interrupt line
static int pl_irq_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct pl_irq_dev *pi;
	const char *inst, *compat = NULL;
	int ret;

	pi = devm_kzalloc(dev, sizeof(*pi), GFP_KERNEL);
	if (!pi)
		return -ENOMEM;

	init_waitqueue_head(&pi->wait);
	spin_lock_init(&pi->lock);
	atomic_set(&pi->count, 0);
	pi->disabled = false;

	pi->irq = platform_get_irq(pdev, 0);
	if (pi->irq < 0)
		return pi->irq;

	// request_irq leaves the line enabled; the DT `interrupts` cell selects the
	// trigger type (level-high here), so pass 0 for flags
	ret = devm_request_irq(dev, pi->irq, pl_irq_handler, 0, DRIVER_NAME, pi);
	if (ret) {
		dev_err(dev, "request_irq %d failed: %d\n", pi->irq, ret);
		return ret;
	}

	inst = pl_irq_instance_name(dev->of_node);
	strscpy(pi->name, inst ? inst : dev->of_node->name, sizeof(pi->name));

	pi->misc.minor = MISC_DYNAMIC_MINOR;
	pi->misc.name  = pi->name;
	pi->misc.fops  = &pl_irq_fops;
	pi->misc.mode  = DEV_MODE;

	ret = misc_register(&pi->misc);
	if (ret) {
		dev_err(dev, "misc_register failed: %d\n", ret);
		return ret;
	}

	platform_set_drvdata(pdev, pi);
	of_property_read_string(dev->of_node, "compatible", &compat);
	dev_info(dev, "/dev/%s ready (mode %#o): irq %d, compatible \"%s\"%s\n",
		 pi->name, DEV_MODE, pi->irq, compat ? compat : "?",
		 inst ? "" : " (no DT label; used node name)");
	return 0;
}

static int pl_irq_remove(struct platform_device *pdev)
{
	struct pl_irq_dev *pi = platform_get_drvdata(pdev);

	misc_deregister(&pi->misc);
	return 0;
}

// Bind by a private compatible so no in-kernel driver competes and no kernel
// command-line parameter is needed. The project's device_tree.dtsi gives the
// interrupt node this compatible.
static const struct of_device_id pl_irq_of_match[] = {
	{ .compatible = "zynq-toolbox,pl-irq-shim" },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, pl_irq_of_match);

static struct platform_driver pl_irq_driver = {
	.driver = {
		.name           = DRIVER_NAME,
		.of_match_table = pl_irq_of_match,
	},
	.probe  = pl_irq_probe,
	.remove = pl_irq_remove,
};

module_platform_driver(pl_irq_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Zynq Toolbox");
MODULE_DESCRIPTION("Non-root wait-for-interrupt on a PL line, bound by DT compatible (no bootargs)");
