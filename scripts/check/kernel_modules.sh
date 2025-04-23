#!/bin/bash
# Check if the XSA for a project exists
# Arguments: <board_name> <board_version> <project_name>
if [ $# -ne 3 ]; then
    echo "[CHECK PTLNX CFG] ERROR:"
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

# Check up to PetaLinux RootFS config file
./scripts/check/petalinux_rootfs_cfg.sh ${BRD} ${VER} ${PRJ}

# Check for the kernel_modules file
KERNEL_MODULES_FILE="projects/${PRJ}/cfg/${BRD}/${VER}/petalinux/${PETALINUX_VERSION}/kernel_modules"
if [ -f "${KERNEL_MODULES_FILE}" ]; then
  while IFS= read -r module || [ -n "$module" ]; do
    # Ensure each line is a single word
    if [[ ! "$module" =~ ^[a-zA-Z0-9_-]+$ ]]; then
      echo "[CHECK KERNEL MODULES] ERROR:"
      echo "Invalid module name '${module}' in ${KERNEL_MODULES_FILE}"
      echo "Each line must contain only one valid word (alphanumeric, underscores, or hyphens)."
      exit 1
    fi

    # Check if the directory for the module exists
    if [ ! -d "kernel_modules/${module}" ]; then
      echo "[CHECK KERNEL MODULES] ERROR:"
      echo "Kernel module directory not found for '${module}'"
      echo "Expected path: kernel_modules/${module}"
      exit 1
    fi
  done < "${KERNEL_MODULES_FILE}"
else
  echo "[CHECK KERNEL MODULES] INFO: No kernel_modules file found. Skipping kernel module checks."
fi
