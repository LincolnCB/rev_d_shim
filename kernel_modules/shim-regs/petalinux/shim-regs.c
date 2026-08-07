// SPDX-License-Identifier: GPL-2.0
/*
 * shim-regs -- non-root, mmap-only access to all Rev. D Shim AXI register
 *              windows, bound AUTOMATICALLY to the device-tree nodes
 *              PetaLinux generates from the Vivado address map.
 *
 * This driver replaces the /dev/mem + mmap strategy used by the shim software
 * (see software/<prog>/src/sys/map_memory.c). It exposes each AXI window as a
 * named /dev entry that userspace mmaps directly -- same speed, no root, no
 * physical addresses in userspace.
 *
 * ---------------------------------------------------------------------------
 * The register windows exposed
 * ---------------------------------------------------------------------------
 * There are four compatible types, covering 20 devices total:
 *
 *  Compatible                       Instance(s)                  Address(es)
 *  -------------------------------  ---------------------------  -----------
 *  xlnx,axi-sys-ctrl-1.0           axi_sys_ctrl                 0x40000000
 *  xlnx,axi-sts-register-1.0       status_reg                   0x40100000
 *  xlnx,axi-clock-timing-snoop-1.0 spi_clk_snoop                0x40200000
 *  xlnx,axi-fifo-bridge-1.0        dac_fifo_{0..7}_axi_bridge   0x800{0..7}0000
 *                                   adc_fifo_{0..7}_axi_bridge   0x800{0..7}1000
 *                                   trig_fifo_axi_bridge         0x80100000
 *
 * Each becomes a /dev/<instance> device, e.g.:
 *
 *   /dev/axi_sys_ctrl           -- system configuration registers
 *   /dev/status_reg             -- system status registers
 *   /dev/spi_clk_snoop          -- SPI clock reconfiguration interface
 *   /dev/dac_fifo_0_axi_bridge  -- DAC command/data FIFO for board 0
 *   /dev/adc_fifo_0_axi_bridge  -- ADC command/data FIFO for board 0
 *   ... (and so on for boards 1-7)
 *   /dev/trig_fifo_axi_bridge   -- trigger command/data FIFO
 *
 * ---------------------------------------------------------------------------
 * How the binding works
 * ---------------------------------------------------------------------------
 * PetaLinux's device-tree generator (DTG) automatically emits one node for
 * every IP in the Vivado address map. The compatible string it assigns is
 * derived from each core's VLNV: "vendor:library:name:version" becomes
 * "xlnx,<name-with-dashes>-<version>" (vendor is always rewritten to "xlnx",
 * so the string encodes core NAME + VERSION only).
 *
 * This driver lists those auto-generated compatibles in its of_match_table.
 * Because each core instance is a separate device-tree node, the kernel calls
 * probe() once per node, so we get one device (and one /dev entry) per
 * register window. There is no hand-written device_tree.dtsi for any of
 * these cores.
 *
 * The DTG preserves each core's Vivado instance name as the DTS label on its
 * node, and the device-tree compiler (run with -@) records every label in a
 * /__symbols__ node as a label -> path map. probe() reverse-looks up the label
 * for each bound node (shim_regs_instance_name) and names the misc device
 * after it, giving /dev/axi_sys_ctrl, /dev/dac_fifo_0_axi_bridge, etc.
 * Userspace opens a register window by name, not address.
 *
 * Fail-loud property: if a core's VLNV name or version changes, its
 * auto-generated compatible changes too, this driver stops matching, probe()
 * never runs, and /dev/<name> is never created -- so userspace open() fails
 * with ENOENT instead of silently accessing the wrong registers.
 *
 * ---------------------------------------------------------------------------
 * How userspace uses it
 * ---------------------------------------------------------------------------
 * Before (map_memory.c, requires root via /dev/mem):
 *
 *     fd  = open("/dev/mem", O_RDWR);
 *     ptr = mmap(NULL, size, PROT_READ|PROT_WRITE, MAP_SHARED, fd, 0x40000000);
 *
 * After (this driver, no root required):
 *
 *     fd  = open("/dev/axi_sys_ctrl", O_RDWR);
 *     ptr = mmap(NULL, size, PROT_READ|PROT_WRITE, MAP_SHARED, fd, 0);
 *
 * Two things change and only two:
 *
 *   1. You open a named device, not /dev/mem. The node is created 0666 (see
 *      DEV_MODE below) so an ordinary user can open it. /dev/mem can never be
 *      handed out this way because it exposes all of physical memory.
 *
 *      Why 0666 and not 0660: the misc core creates the node owned root:root
 *      and can only set its *mode*, not its *group*. Assigning a friendlier
 *      group is a udev job, and this rootfs has no udev. With 0660 the node
 *      stays root:root and a non-root user is denied. 0666 makes it world-
 *      accessible, which is the only udev-free way to reach it without root.
 *      Scope is still limited to just these specific register windows.
 *
 *   2. There are no physical addresses in userspace, and each device maps a
 *      single window at mmap offset 0. If a block moves in the hardware
 *      design, the auto-generated node moves with it and userspace is
 *      unchanged -- it still opens the same /dev/<name>.
 *
 * Why it is exactly as fast as /dev/mem: mmap() installs page-table entries
 * that point straight at the register's physical pages (io_remap_pfn_range,
 * non-cached). After that, every access is a single load/store instruction --
 * no syscall per access. It is the same mechanism /dev/mem uses.
 *
 * This driver is intentionally mmap-only. There is no read()/write()/ioctl()
 * path, because those would be a syscall per access -- the slow thing we are
 * trying to avoid. Keeping it to just mmap keeps the driver small and the
 * "how do I port my map_memory.c code" story a one-line change.
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/string.h>

#define DRIVER_NAME "shim-regs"
#define DEV_MODE    0666               /* world-accessible: no root, no udev */

/* One of these per bound device-tree node. Each node carries a single AXI
 * register window, so we track a single window here. The /dev name is copied
 * in because misc.name must outlive probe(). */
struct shim_regs_dev {
	phys_addr_t       phys;
	resource_size_t   size;
	struct miscdevice misc;
	char              name[48];
};

static int shim_regs_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct miscdevice *misc = file->private_data;
	struct shim_regs_dev *sr = container_of(misc, struct shim_regs_dev, misc);
	unsigned long len = vma->vm_end - vma->vm_start;

	/* Each device exposes exactly one window, mapped at offset 0. */
	if (vma->vm_pgoff != 0)
		return -EINVAL;

	/* Never let a mapping run past the end of the window. */
	if (len > PAGE_ALIGN(sr->size))
		return -EINVAL;

	/* Registers are MMIO: the mapping must be non-cached. */
	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);

	/* Map the window's physical pages straight into the process. After this
	 * returns, userspace accesses the registers with no further syscalls. */
	return io_remap_pfn_range(vma, vma->vm_start,
				  sr->phys >> PAGE_SHIFT,
				  len, vma->vm_page_prot);
}

static const struct file_operations shim_regs_fops = {
	.owner = THIS_MODULE,
	.mmap  = shim_regs_mmap,
};

/* Scan the /aliases- or /__symbols__-style directory at `dir_path` for the
 * property whose value (a node path) resolves back to `np`, and return that
 * property's name. Returns NULL if none matches, in which case the caller
 * falls back to the bare node name. */
static const char *shim_regs_lookup_name(const char *dir_path,
					 struct device_node *np)
{
	struct device_node *dir;
	struct property *pp;
	const char *result = NULL;

	dir = of_find_node_by_path(dir_path);
	if (!dir)
		return NULL;

	/* Every property here maps a name -> a node path. The property whose
	 * path resolves back to our node gives us that node's instance name. */
	for_each_property_of_node(dir, pp) {
		struct device_node *target;

		/* Skip the housekeeping properties the kernel adds. */
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

/* Recover this node's Vivado instance name (e.g. "axi_sys_ctrl") so the /dev
 * entry can be named after it rather than after the shared core name
 * ("axi_fifo_bridge", which all 17 FIFO bridge instances would share).
 *
 * Where the instance name actually lives (verified against a real 2024.2 DTB):
 * PetaLinux does NOT put PL cores in /aliases -- that node only carries the
 * standard serialN / spiN entries. Instead the instance name is emitted as the
 * DTS *label* on the node ("axi_sys_ctrl: axi_sys_ctrl@40000000"), and the
 * device-tree compiler (run with -@) preserves every label in a /__symbols__
 * node as a label -> path map. So we check /aliases first (the conventional
 * place, in case a core is ever given a real alias) and then /__symbols__,
 * where the Vivado instance labels actually are. */
static const char *shim_regs_instance_name(struct device_node *np)
{
	const char *name;

	name = shim_regs_lookup_name("/aliases", np);
	if (!name)
		name = shim_regs_lookup_name("/__symbols__", np);
	return name;
}

static int shim_regs_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct shim_regs_dev *sr;
	struct resource *res;
	const char *inst, *compat = NULL;
	int ret;

	sr = devm_kzalloc(dev, sizeof(*sr), GFP_KERNEL);
	if (!sr)
		return -ENOMEM;

	/* Auto-generated nodes carry a single reg range and NO reg-names, so we
	 * fetch the window by index (0) rather than by name. */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(dev, "no MEM resource in node\n");
		return -ENODEV;
	}

	/* Claim the window. This is the thing /dev/mem cannot do: the driver now
	 * *owns* these registers, so two drivers can't fight over them. (It does
	 * not lock out /dev/mem unless the kernel has CONFIG_IO_STRICT_DEVMEM.) */
	if (!devm_request_mem_region(dev, res->start, resource_size(res),
				     DRIVER_NAME)) {
		dev_err(dev, "region %pa already in use\n", &res->start);
		return -EBUSY;
	}

	sr->phys = res->start;
	sr->size = resource_size(res);

	/* Name the /dev node after the Vivado instance name (recovered from the
	 * DT symbols/aliases), so userspace opens /dev/axi_sys_ctrl and never
	 * touches an address. Fall back to the bare node name if this node
	 * somehow has no label (e.g. a non-standard DTB). */
	inst = shim_regs_instance_name(dev->of_node);
	strscpy(sr->name, inst ? inst : dev->of_node->name, sizeof(sr->name));

	sr->misc.minor = MISC_DYNAMIC_MINOR;
	sr->misc.name  = sr->name;
	sr->misc.fops  = &shim_regs_fops;
	sr->misc.mode  = DEV_MODE;   /* world-rw node: non-root access, no udev */

	ret = misc_register(&sr->misc);
	if (ret) {
		dev_err(dev, "misc_register failed: %d\n", ret);
		return ret;
	}

	platform_set_drvdata(pdev, sr);
	of_property_read_string(dev->of_node, "compatible", &compat);
	dev_info(dev,
		 "/dev/%s ready (mode %#o): %pa size 0x%llx, compatible \"%s\"%s\n",
		 sr->name, DEV_MODE, &sr->phys,
		 (unsigned long long)sr->size, compat ? compat : "?",
		 inst ? "" : " (no DT label; used node name)");
	return 0;
}

static int shim_regs_remove(struct platform_device *pdev)
{
	struct shim_regs_dev *sr = platform_get_drvdata(pdev);

	misc_deregister(&sr->misc);
	return 0;
}

/* Bind to the compatibles PetaLinux's DTG auto-generates for the shim's AXI
 * cores. These are "xlnx,<vlnv-name-dashed>-<version>" -- the vendor part of
 * the VLNV is always rewritten to "xlnx" by the generator.
 *
 * axi-fifo-bridge covers 17 instances (8 DAC + 8 ADC + 1 trigger); the others
 * each cover a single instance. Instance names are recovered at probe time
 * from /__symbols__ so each gets its own uniquely named /dev entry. */
static const struct of_device_id shim_regs_of_match[] = {
	{ .compatible = "xlnx,axi-sys-ctrl-1.0" },
	{ .compatible = "xlnx,axi-sts-register-1.0" },
	{ .compatible = "xlnx,axi-clock-timing-snoop-1.0" },
	{ .compatible = "xlnx,axi-fifo-bridge-1.0" },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, shim_regs_of_match);

static struct platform_driver shim_regs_driver = {
	.driver = {
		.name           = DRIVER_NAME,
		.of_match_table = shim_regs_of_match,
	},
	.probe  = shim_regs_probe,
	.remove = shim_regs_remove,
};

module_platform_driver(shim_regs_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Rev. D Shim Amplifier Project");
MODULE_DESCRIPTION("Non-root, mmap-only access to shim AXI register windows, bound to PetaLinux auto-nodes");
