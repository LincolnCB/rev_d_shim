#!/bin/bash
# Check tif the XSA file for a project, board, and version exists
# Arguments: <board_name> <board_version> <project_name> [--full]
# Full check: Project source and prerequisites
# Minimum check: None

# Parse arguments
if [ $# -lt 3 ] || [ $# -gt 4 ] || ( [ $# -eq 4 ] && [ "$4" != "--full" ] ); then
  echo "[CHECK XSA FILE] ERROR:"
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
  # Full check: Project source and prerequisites
  ./scripts/check/project_src.sh ${BRD} ${VER} ${PRJ} --full
fi # Minimum check: None

# Check that the necessary XSA exists
if [ ! -f "tmp/${BRD}/${VER}/${PRJ}/hw_def.xsa" ]; then
  echo "[CHECK XSA FILE] ERROR:"
  echo "Missing Vivado-generated XSA hardware definition file for ${PBV}"
  echo " Path: tmp/${BRD}/${VER}/${PRJ}/hw_def.xsa"
  echo "First run the following command:"
  echo
  echo " make BOARD=${BRD} BOARD_VER=${VER} PROJECT=${PRJ} xsa"
  echo
  exit 1
fi
