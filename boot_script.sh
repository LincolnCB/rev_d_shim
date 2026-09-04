#!/bin/sh
# Runs once at boot (installed to /etc/init.d by the boot-script recipe; see
# scripts/petalinux/boot_script.sh).
#
# Relax the u-dma-buf interfaces so the DMA datapath runs without root. This
# rootfs has no udev, and u-dma-buf creates each /dev node root-owned 0600 and
# its sysfs cache-sync controls root-owned 0664 -- neither has a mode knob
# (unlike pl-reg-shim / pl-irq-shim, which set misc.mode = 0666 themselves). A
# boot-time chmod is the udev-free way to hand them to a non-root user; a chmod
# on a sysfs attribute persists for the device's lifetime. The PL register
# windows and the hw_manager interrupt are already non-root via pl-reg-shim and
# pl-irq-shim.
#
# Both regions are covered: udmabuf0 (the data region -- the prebuffered DAC
# sequence and ADC capture) and udmabuf1 (the SG descriptor rings).

# The buffer nodes themselves (the mmap targets).
chmod 0666 /dev/udmabuf* 2>/dev/null || true

# The sysfs controls the software writes: sync_for_device / sync_for_cpu drive the
# single-flush coherency path, and the offset/size/direction/mode attrs may be set
# alongside them. Without these, mmap succeeds but the first sync fails EACCES.
for f in sync_for_device sync_for_cpu sync_offset sync_size sync_direction sync_mode; do
  chmod 0666 /sys/class/u-dma-buf/*/"$f" 2>/dev/null || true
done
