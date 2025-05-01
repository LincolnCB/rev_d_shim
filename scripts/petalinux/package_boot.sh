#!/bin/bash
# Package the PetaLinux project boot image
# Arguments: <board_name> <board_version> <project_name>
if [ $# -ne 3 ]; then
    echo "[PTLNX BOOT PKG] ERROR:"
    echo "Usage: $0 <board_name> <board_version> <project_name>"
    exit 1
fi

# Store the positional parameters in named variables and clear them
BRD=${1}
VER=${2}
PRJ=${3}
PBV="project \"${PRJ}\" and board \"${BRD}\" v${VER}"
set --

# If any subsequent command fails, exit immediately
set -e

# Source the PetaLinux settings script
if [ -z "$PETALINUX_PATH" ]; then
    echo "[PTLNX BOOT PKG] ERROR: PETALINUX_PATH environment variable is not set."
    exit 1
fi
if [ ! -f "${PETALINUX_PATH}/settings.sh" ]; then
    echo "[PTLNX BOOT PKG] ERROR: PetaLinux settings script not found at ${PETALINUX_PATH}/settings.sh"
    exit 1
fi
source $PETALINUX_PATH/settings.sh

# Enter the project
# Check that the necessary PetaLinux project exists
if [ ! -d "tmp/${BRD}/${VER}/${PRJ}/petalinux" ]; then
    echo "[PTLNX BOOT PKG] ERROR:"
    echo "Missing PetaLinux project directory for ${PBV}"
    echo " Path: tmp/${BRD}/${VER}/${PRJ}/petalinux."
    echo "First run the following command:"
    echo
    echo " make BOARD=${BRD} BOARD_VER=${VER} PROJECT=${PRJ} petalinux"
    echo
    exit 1
fi
cd tmp/${BRD}/${VER}/${PRJ}/petalinux

# Package the boot image using the year-specific command line arguments
if [ "$PETALINUX_YEAR" -lt 2024 ]; then
    echo "[PTLNX BOOT PKG] Using legacy PetaLinux project creation command for year ${PETALINUX_YEAR}"
    petalinux-package --boot \
    --format BIN \
    --fsbl \
    --fpga \
    --kernel \
    --boot-device sd \
    --force
else
    echo "[PTLNX BOOT PKG] Using PetaLinux project creation python arguments (for year ${PETALINUX_YEAR})"
    petalinux-package boot \
    --format BIN \
    --fsbl \
    --fpga \
    --boot-device sd \
    --force
fi

# Compress the necessary files together
tar -czf images/linux/BOOT.tar.gz \
    -C images/linux \
    BOOT.BIN \
    image.ub \
    boot.scr \
    system.dtb
