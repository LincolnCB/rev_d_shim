#!/bin/bash
# Check the PetaLinux root filesystem configuration for a project, board, and version
# Arguments: <board_name> <board_version> <project_name> [--full]
# Full check: PetaLinux system config and prerequisites
# Minimum check: PetaLinux system config (includes project source, PetaLinux environment, and PetaLinux config directory)

# Parse arguments
if [ $# -lt 3 ] || [ $# -gt 4 ] || ( [ $# -eq 4 ] && [ "$4" != "--full" ] ); then
  echo "[CHECK PTLNX ROOTFS CFG] ERROR:"
  echo "Usage: $0 <board_name> <board_version> <project_name> [--full]"
  exit 1
fi
FULL_CHECK=false
if [[ "$4" == "--full" ]]; then
  FULL_CHECK=true
fi

# Store the positional parameters in named variables and clear them
BRD=${1}
VER=${2}
PRJ=${3}
PBV="project \"${PRJ}\" and board \"${BRD}\" v${VER}"
set --

# If any subsequent command fails, exit immediately
set -e

# Check prerequisites. If full, check all prerequisites. Otherwise, just the immediate necessary ones.
if $FULL_CHECK; then
  # Full check: PetaLinux system config and prerequisites
  ./scripts/check/petalinux_sys_cfg_file.sh ${BRD} ${VER} ${PRJ} --full
else
  # Minimum check: PetaLinux system config (includes project source and PetaLinux environment)
  ./scripts/check/petalinux_sys_cfg_file.sh ${BRD} ${VER} ${PRJ}
fi

# Check that the necessary PetaLinux rootfs config file exists
if [ ! -f "projects/${PRJ}/cfg/${BRD}/${VER}/petalinux/${PETALINUX_VERSION}/rootfs_config.patch" ]; then
  echo "[CHECK PTLNX ROOTFS CFG] ERROR:"
  echo "Missing PetaLinux filesystem configuration patch file for ${PBV}"
  echo " Path: projects/${PRJ}/cfg/${BRD}/${VER}/petalinux/${PETALINUX_VERSION}/rootfs_config.patch"
  echo "You can create this file by running the following command:"
  echo
  echo " scripts/petalinux/config_rootfs.sh ${BRD} ${VER} ${PRJ}"
  echo
  exit 1
fi
