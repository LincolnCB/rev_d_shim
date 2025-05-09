#!/bin/bash
# Check the PetaLinux project for a board and version
# Arguments: <board_name> <board_version> <project_name> [--full]
# Full check: PetaLinux environment and prerequisites
# Minimum check: PetaLinux environment

# Parse arguments
if [ $# -lt 3 ] || [ $# -gt 4 ] || ( [ $# -eq 4 ] && [ "$4" != "--full" ] ); then
  echo "[CHECK PTLNX PROJECT] ERROR:"
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
  # Minimum check: PetaLinux environment
  ./scripts/check/petalinux_env.sh ${BRD} ${VER} ${PRJ}
fi

# Check that the necessary PetaLinux project exists
if [ ! -d "tmp/${BRD}/${VER}/${PRJ}/petalinux" ]; then
  echo "[CHECK PTLNX PROJECT] ERROR:"
  echo "Missing PetaLinux project directory for ${PBV}"
  echo " Path: tmp/${BRD}/${VER}/${PRJ}/petalinux"
  echo "First run the following command:"
  echo
  echo " make BOARD=${BRD} BOARD_VER=${VER} PROJECT=${PRJ} petalinux"
  echo
  exit 1
fi
