#!/bin/bash
# Build a PetaLinux project for the given board and project
# Arguments: <board_name> <board_version> <project_name>
if [ $# -ne 3 ]; then
    echo "[PTLNX BUILD SCRIPT] ERROR:"
    echo "Usage: $0 <board_name> <board_version> <project_name>"
    exit 1
fi

# Store the positional parameters in named variables and clear them
BRD=${1}
VER=${2}
PRJ=${3}
PBV="project \"${PRJ}\" and board \"${BRD}\" v${VER}"
set --

# Check that the project exists in "projects"
if [ ! -d "projects/${PRJ}" ]; then
    echo "[PTLNX BUILD SCRIPT] ERROR:"
    echo "Repository project directory not found for project \"${PRJ}\""
    echo "  projects/${PRJ}"
    exit 1
fi

# Check that the necessary XSA exists
if [ ! -f "tmp/${BRD}/${VER}/${PRJ}/hw_def.xsa" ]; then
    echo "[PTLNX BUILD SCRIPT] ERROR:"
    echo "Missing Vivado-generated XSA hardware definition file for ${PBV}"
    echo "  tmp/${BRD}/${VER}/${PRJ}/hw_def.xsa."
    echo "First run the following command:"
    echo
    echo "  make BOARD=${BRD} BOARD_VER=${VER} PROJECT=${PRJ} xsa"
    echo
    exit 1
fi

# Check that the necessary PetaLinux config files exist
if [ ! -f "projects/${PRJ}/cfg/${BRD}/${VER}/petalinux/config.patch" ]; then
    echo "[PTLNX BUILD SCRIPT] ERROR:"
    echo "Missing PetaLinux project configuration patch file for ${PBV}"
    echo "  projects/${PRJ}/cfg/${BRD}/${VER}/petalinux/config.patch"
    echo "You can create this file by running the following command:"
    echo
    echo "  scripts/petalinux/config_project.sh ${BRD} ${VER} ${PRJ}"
    echo
    exit 1
fi
if [ ! -f "projects/${PRJ}/cfg/${BRD}/${VER}/petalinux/rootfs_config.patch" ]; then
    echo "[PTLNX BUILD SCRIPT] ERROR:"
    echo "Missing PetaLinux filesystem configuration patch file for ${PBV}"
    echo "  projects/${PRJ}/cfg/${BRD}/${VER}/petalinux/rootfs_config.patch"
    echo "You can create this file by running the following command:"
    echo
    echo "  scripts/petalinux/config_rootfs.sh ${BRD} ${VER} ${PRJ}"
    echo
    exit 1
fi

# Create and enter the project
cd tmp/${BRD}/${VER}/${PRJ}
petalinux-create -t project --template zynq --name petalinux --force
cd petalinux

# Patch the project configuration
echo "[PTLNX BUILD SCRIPT] Initializing default PetaLinux project configuration"
petalinux-config --get-hw-description ../hw_def.xsa --silentconfig
echo "[PTLNX BUILD SCRIPT] Patching and configuring PetaLinux project"
patch project-spec/configs/config ../../../../../projects/${PRJ}/cfg/${BRD}/${VER}/petalinux/config.patch
petalinux-config --silentconfig

# Patch the root filesystem configuration
echo "[PTLNX BUILD SCRIPT] Initializing default PetaLinux root filesystem configuration"
petalinux-config -c rootfs --silentconfig
echo "[PTLNX BUILD SCRIPT] Patching and configuring PetaLinux root filesystem"
patch project-spec/configs/rootfs_config ../../../../../projects/${PRJ}/cfg/${BRD}/${VER}/petalinux/rootfs_config.patch
petalinux-config -c rootfs --silentconfig

# Build the project
echo "[PTLNX BUILD SCRIPT] Building the PetaLinux project"
petalinux-build
