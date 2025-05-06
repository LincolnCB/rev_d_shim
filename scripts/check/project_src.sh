#!/bin/bash
# Check the project source scripts for a given board and version
# Arguments: <board_name> <board_version> <project_name> [--full]
# Full check: Board files
# Minimum check: None

# Parse arguments
if [ $# -lt 3 ] || [ $# -gt 4 ] || ( [ $# -eq 4 ] && [ "$4" != "--full" ] ); then
  echo "[CHECK PROJECT SRC] ERROR:"
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
  # Full check: Project directory and prerequisites
  ./scripts/check/project_dir.sh ${BRD} ${VER} ${PRJ} --full
else
  # Minimum check: Project directory
  ./scripts/check/project_dir.sh ${BRD} ${VER} ${PRJ}
fi

BD_FILE=projects/${PRJ}/block_design.tcl

# Use get_cores_from_tcl.sh to extract core paths
CORE_NAMES=$(./scripts/make/get_cores_from_tcl.sh "${BD_FILE}")
if [ $? -ne 0 ]; then
  echo "[CHECK PROJECT SRC] ERROR: Failed to extract cores from block design file."
  exit 1
fi

# Verify that each extracted core path exists
while IFS= read -r CORE_NAME; do
  # Convert a core name (like lcb/hw_manager) to a core path (like custom_cores/lcb/cores/hw_manager)
  CORE_PATH="custom_cores/${CORE_NAME//\//\/cores/}"
  if [ ! -d "$CORE_PATH" ]; then
    echo "[CHECK CORES] ERROR: Core path does not exist: $CORE_PATH"
    exit 1
  fi
  # Check if the core path contains a .v or .sv file of the same name
  CORE_FILE="${CORE_PATH}/$(basename "$CORE_NAME").v"
  if [ ! -f "$CORE_FILE" ]; then
    echo "[CHECK CORES] ERROR: Core top file does not exist: $CORE_FILE"
    exit 1
  fi
done <<< "$CORE_NAMES"
