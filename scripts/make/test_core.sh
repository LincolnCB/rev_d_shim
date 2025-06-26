#!/bin/bash
# Tests a custom core with cocotb. Uses the shared Makefile in scripts/make/cocotb.mk.
# Arguments: <vendor> <core>
# Usage: test_core.sh <vendor> <core>
# Example:
#   ./scripts/make/test_core.sh lcb shim_hw_manager

if [ $# -ne 2 ]; then
  echo "[CORE TESTS] ERROR:"
  echo "Usage: $0 <vendor> <core>"
  exit 1
fi

# Store the positional parameters in named variables and clear them
VENDOR=${1}
CORE=${2}
set --

TEST_DIR="custom_cores/${VENDOR}/cores/${CORE}/tests"

# Verify that the tests directory exists
if [ ! -d "${TEST_DIR}" ]; then
  echo "[CORE TESTS] ERROR: Directory not found: ${TEST_DIR}"
  exit 1
fi

STS_FILE="${TEST_DIR}/test_status"

# Remove any old results.xml for a clean start
rm -f "${TEST_DIR}/results/results.xml"

# Enter the src directory where the Makefile is expected
if [ ! -d "${TEST_DIR}/src" ]; then
  echo "[CORE TESTS]: src directory not found in ${TEST_DIR} -- assuming no tests to run."

  # Write a skip on the status file line (overwriting old one)
  echo "NO TESTS (no src directory) as of $(date +"%Y/%m/%d at %H:%M %Z") " > "${STS_FILE}"
  echo "[CORE TESTS] ${CORE}: Tests SKIPPED (see ${TEST_DIR}/${STS_FILE} details)"

  exit 0
fi

echo "[CORE TESTS] Running tests for ${CORE} in ${TEST_DIR} using scripts/make/cocotb.mk"

# Run “make test_custom_core”
# Makefile inside tests/src defines a target "test_custom_core"
mkdir -p "${TEST_DIR}/results"  # Ensure results directory exists

if ! make --directory="${TEST_DIR}/src" --file="$(realpath scripts/make/cocotb.mk)" "test_custom_core" > "${TEST_DIR}/results/log.txt"; then
  # Makefile itself failed (e.g. Verilator compile error). Mark as failure.
  echo "[CORE TESTS] ERROR: Makefile failed for ${CORE} tests."
  echo "See log.txt for details: ${TEST_DIR}/results/log.txt"
  exit 1
else
  # Make succeeded. Now look for results.xml in the results directory.
  if [ -f "${TEST_DIR}/results/results.xml" ]; then
    # If there's any tag starting with "<failure" in results.xml, mark as failure
    if grep -q "<failure" "${TEST_DIR}/results/results.xml"; then
      echo "[CORE TESTS] ERROR: Found failure tags in results.xml for ${CORE} tests."
      STATUS="FAILED tests"
    else
      STATUS="PASSED tests"
    fi
  else
    # Failure if results.xml is not found
    echo "[CORE TESTS] ERROR: No results.xml found in ${TEST_DIR}/results" 
    STATUS="FAILED tests"
  fi
fi

# Write the status file line (overwriting old one)
echo "${STATUS} on $(date +"%Y/%m/%d at %H:%M %Z")" > "${STS_FILE}"

# Put extra info if failed
if [ "${STATUS}" == "FAILED tests" ]; then
  echo "See log.txt for details: ${TEST_DIR}/results/log.txt" >> "${STS_FILE}"
fi

echo "[CORE TESTS] ${CORE}: ${STATUS} (see ${TEST_DIR}/${STS_FILE} and ${TEST_DIR}/results/log.txt for details)"
exit 0
