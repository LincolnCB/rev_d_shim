#!/bin/bash
# Create a patch file for the PetaLinux filesystem configuration
# Arguments: <board_name> <board_version> <project_name> ['update' (optional)]
if [ $# -ne 3 ] && [ $# -ne 4 ]; then
    echo "Usage: $0 <board_name> <board_version> <project_name> ['update' (optional)]"
    exit 1
fi

# Exit if "update" is not the fourth argument
if [ $# -eq 4 ] && [ "${4}" != "update" ]; then
    echo "Usage: $0 <board_name> <board_version> <project_name> ['update' (optional)]"
    exit 1
fi

# Update is the fourth argument or there is none
if [ $# -eq 4 ] ; then
    UPDATE=1
else
    UPDATE=0
fi

# Check if terminal width is at least 80 columns
if [ $(tput cols) -lt 80 ] || [ $(tput lines) -lt 19 ]; then
    echo "Terminal must be at least 80 columns wide and 19 lines tall to use the PetaLinux configuration menu."
    exit 1
fi

# Store the positional parameters in named variables and clear them
# (Petalinux settings script requires no positional parameters)
CMD=${0}
BRD=${1}
VER=${2}
PRJ=${3}
PBV="project \"${PRJ}\" and board \"${BRD}\" v${VER}"
set --

# Check the PetaLinux config and dependencies
echo "[PTLNX ROOTFS SCRIPT] Checking PetaLinux config and dependencies for ${PBV}"
./scripts/check/petalinux_cfg.sh ${BRD} ${VER} ${PRJ} || exit 1

# Check that the PetaLinux root filesystem configuration patch does not already exist if not updating
if [ -f "projects/${PRJ}/cfg/${BRD}/${VER}/petalinux/rootfs_config.patch" ] && [ $UPDATE -ne 1 ]; then
    echo "[PTLNX ROOTFS SCRIPT] ERROR:"
    echo "PetaLinux root filesystem configuration patch already exists for ${PBV}"
    echo "  projects/${PRJ}/cfg/${BRD}/${VER}/petalinux/rootfs_config.patch"
    echo "If you want to use that patch as the start point, use the following command:"
    echo
    echo "  ${CMD} ${BRD} ${VER} ${PRJ} update"
    exit 1
fi

# Check that the PetaLinux root filesystem configuration patch DOES exist if updating
if [ ! -f "projects/${PRJ}/cfg/${BRD}/${VER}/petalinux/rootfs_config.patch" ] && [ $UPDATE -eq 1 ]; then
    echo "[PTLNX ROOTFS SCRIPT] ERROR:"
    echo "Missing PetaLinux root filesystem configuration patch for ${PBV}"
    echo "  projects/${PRJ}/cfg/${BRD}/${VER}/petalinux/rootfs_config.patch"
    echo "If you want to create a new patch, copy one in or use the following command:"
    echo
    echo "  ${CMD} ${BRD} ${VER} ${PRJ}"
    exit 1
fi

# Source the PetaLinux settings script (make sure to clear positional parameters first)
source ${PETALINUX_PATH}/settings.sh

# Create a new template project
cd tmp
if [ -d "petalinux_template" ]; then
    rm -rf petalinux_template
fi
mkdir petalinux_template
cd petalinux_template
petalinux-create -t project --template zynq --name petalinux
cd petalinux

# Patch the project configuration
echo "[PTLNX ROOTFS SCRIPT] Initializing default PetaLinux project configuration"
petalinux-config --get-hw-description ../../${BRD}/${VER}/${PRJ}/hw_def.xsa --silentconfig
echo "[PTLNX ROOTFS SCRIPT] Patching and configuring PetaLinux project"
patch project-spec/configs/config ../../../projects/${PRJ}/cfg/${BRD}/${VER}/petalinux/config.patch
petalinux-config --silentconfig

# Initialize the default root filesystem configuration
echo "[PTLNX ROOTFS SCRIPT] Initializing default PetaLinux root filesystem configuration"
petalinux-config -c rootfs --silentconfig

# Copy the default root filesystem configuration
echo "[PTLNX ROOTFS SCRIPT] Saving default PetaLinux root filesystem configuration"
cp project-spec/configs/rootfs_config project-spec/configs/rootfs_config.default

# If updating, apply the existing patch
if [ $UPDATE -eq 1 ]; then
    echo "[PTLNX ROOTFS SCRIPT] Applying existing PetaLinux root filesystem configuration patch"
    patch project-spec/configs/rootfs_config ../../../projects/${PRJ}/cfg/${BRD}/${VER}/petalinux/rootfs_config.patch
fi

# Manually configure the root filesystem
echo "[PTLNX ROOTFS SCRIPT] Manually configuring PetaLinux root filesystem"
petalinux-config -c rootfs

# Create a patch for the root filesystem configuration
echo "[PTLNX ROOTFS SCRIPT] Creating PetaLinux root filesystem configuration patch"
diff -u project-spec/configs/rootfs_config.default project-spec/configs/rootfs_config | \
    tail -n +3 > \
    ../../../projects/${PRJ}/cfg/${BRD}/${VER}/petalinux/rootfs_config.patch

# Replace the root filesystem configuration with the default for the template project
echo "[PTLNX ROOTFS SCRIPT] Restoring default PetaLinux root filesystem configuration for template project"
