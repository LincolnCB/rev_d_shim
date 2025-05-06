#!/bin/bash
# Check the PetaLinux config directory for a project, board, and version
# Arguments: <board_name> <board_version> <project_name> [--full]
# Full check: PetaLinux environment and prerequisites
# Minimum check: Project source and PetaLinux environment

# Parse arguments
if [ $# -lt 3 ] || [ $# -gt 4 ] || ( [ $# -eq 4 ] && [ "$4" != "--full" ] ); then
  echo "[CHECK PTLNX CFG DIR] ERROR:"
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
  # Full check: PetaLinux environment and prerequisites
  ./scripts/check/petalinux_env.sh ${BRD} ${VER} ${PRJ} --full
else
  # Minimum check: Project directory and PetaLinux environment
  ./scripts/check/project_dir.sh ${BRD} ${VER} ${PRJ}
  ./scripts/check/petalinux_env.sh ${BRD} ${VER} ${PRJ}
fi

# Check that the board cfg directory (and petalinux directory) exists
if [ ! -d "projects/${PRJ}/cfg/${BRD}/${VER}/petalinux" ]; then
  echo "[CHECK PTLNX CFG DIR] ERROR:"
  echo "Board PetaLinux configuration directory not found for ${PBV}"
  echo "  Path: projects/${PRJ}/cfg/${BRD}/${VER}/petalinux"
  exit 1
fi

# Check that the PetaLinux config directory exists for the PetaLinux version
if [ ! -d "projects/${PRJ}/cfg/${BRD}/${VER}/petalinux/${PETALINUX_VERSION}" ]; then
  echo "[CHECK PTLNX CFG DIR] ERROR:"
  echo "PetaLinux version ${PETALINUX_VERSION} directory not found for ${PBV}"
  echo "  Path: projects/${PRJ}/cfg/${BRD}/${VER}/petalinux/${PETALINUX_VERSION}"
  exit 1
fi
