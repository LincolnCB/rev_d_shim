#!/bin/bash
# Check the project source scripts for a given board and version
# Arguments: <board_name> <board_version> <project_name> [--full]
# Full check: Board files
# Minimum check: None

# Parse arguments
FULL_CHECK=false

# Loop through arguments to find --full and assign positional parameters
ARGS=()
for arg in "$@"; do
  if [[ "$arg" == "--full" ]]; then
    FULL_CHECK=true
  else
    ARGS+=("$arg")
  fi
done

if [ ${#ARGS[@]} -ne 3 ]; then
  echo "[CHECK PROJECT SRC] ERROR:"
  echo "Usage: $0 <board_name> <board_version> <project_name> [--full]"
  exit 1
fi

BRD=${ARGS[0]}
VER=${ARGS[1]}
PRJ=${ARGS[2]}
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

# Use get_cores_from_tcl.sh to extract custom core paths
CORE_NAMES=$(./scripts/make/get_cores_from_tcl.sh "${BD_FILE}")
if [ $? -ne 0 ]; then
  echo "[CHECK PROJECT SRC] ERROR: Failed to extract cores from block design file."
  exit 1
fi

# If no custom cores, exit
[ -z "$CORE_NAMES" ] && exit 0

# Verify that each extracted core path exists
while IFS= read -r CORE_NAME; do
  # Convert a core name (like base/axi_fifo_bridge) to a core path (like project/PROJECT/cores/base/axi_fifo_bridge)
  CORE_PATH="projects/${PRJ}/cores/${CORE_NAME}"
  if [ ! -d "$CORE_PATH" ]; then
    echo "[CHECK PROJECT SRC] ERROR: Core path does not exist: $CORE_PATH"
    exit 1
  fi
  # Check if the core path contains a .v or .sv file of the same name
  CORE_FILE="${CORE_PATH}/$(basename "$CORE_NAME").v"
  if [ ! -f "$CORE_FILE" ]; then
    echo "[CHECK PROJECT SRC] ERROR: Core top file does not exist: $CORE_FILE"
    exit 1
  fi
done <<< "$CORE_NAMES"
