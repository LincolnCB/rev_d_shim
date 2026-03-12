#############################################
## Variables
#############################################
## Required environment variable inputs to the Makefile
#############################################

# You need to set PROJECT, BOARD, and BOARD_VER to the project and board you want to build
# These can be set on the command line:
# - e.g. 'make PROJECT=ex02_axi_interface BOARD=snickerdoodle_black BOARD_VER=1.0'

#   --------------------------------------------------------------------
#   ------>                                                      <------
#   ------> To set your personal defaults, edit make_defaults.mk <------
#   ------>   which you can copy from make_defaults.mk.example   <------
#   ------>                                                      <------
#   --------------------------------------------------------------------

# Include the default variables file
include make_defaults.mk

#############################################



#############################################
## Initialization
#############################################
## Some scripts to initialize the environment and check for necessary tools/src
#############################################

# Check for the REV D environment variable
ifneq ($(shell pwd),$(ZYNQ_TOOLBOX))
$(warning Environment variable ZYNQ_TOOLBOX does not match the current directory)
$(warning - Current directory: $(shell pwd))
$(error - ZYNQ_TOOLBOX: $(ZYNQ_TOOLBOX))
endif

# Check if the Vivado settings64.sh file exists
ifeq ($(wildcard $(VIVADO_PATH)/settings64.sh),)
$(error Vivado path environment variable VIVADO_PATH is not/incorrectly set - "$(VIVADO_PATH)". VIVADO_PATH/settings64.sh must exist)
endif

# Check if the PetaLinux settings.sh file exists
ifeq ($(wildcard $(PETALINUX_PATH)/settings.sh),)
$(error PetaLinux path environment variable PETALINUX_PATH is not/incorrectly set - "$(PETALINUX_PATH)". PETALINUX_PATH/settings.sh must exist)
endif

# Check that the PetaLinux version environment variable is set
ifeq ($(PETALINUX_VERSION),)
$(error PetaLinux version environment variable PETALINUX_VERSION is not set)
endif

# Check if the project and board matter for the make targets
#   For instance, if the user runs 'make clean' or 'make help', the project and board do not matter
#   and don't need to be checked
PROJECT_MATTERS = true
# If no targets, then the project and board do matter
ifneq ($(),$(MAKECMDGOALS))
# If some targets are specified, check if none of them require the project and board
ifeq ($(),$(filter-out help clean_sd clean_build clean_tests clean_test_results clean_all,$(MAKECMDGOALS)))
PROJECT_MATTERS = false
endif
endif

$(info --------------------------)
ifeq ($(),$(MAKECMDGOALS))
$(info ---- Making "all")
else
$(info ---- Making "$(MAKECMDGOALS)")
endif

# Run some checks and setup if the project and board matter for the make targets
ifeq (true, $(PROJECT_MATTERS)) # Check if the project and board matter
$(info ----  for project "$(PROJECT)" and board "$(BOARD)" version $(BOARD_VER))

# Check the board, board version, and project
ifneq ($(), $(shell ./scripts/check/project_src.sh $(BOARD) $(BOARD_VER) $(PROJECT) --full))
$(info --------------------------)
$(info ----  Project check failed)
$(info ----  $(shell ./scripts/check/project_src.sh $(BOARD) $(BOARD_VER) $(PROJECT) --full))
$(info --------------------------)
$(error Missing sources)
endif

# Extract the part from board file
export PART=$(shell ./scripts/make/get_part.sh $(BOARD) $(BOARD_VER))
$(info ----  Part: $(PART))

# Get the list of necessary cores from the project file to avoid building unnecessary cores
PROJECT_CORES = $(shell ./scripts/make/get_cores_from_tcl.sh projects/$(PROJECT)/block_design.tcl)
BOARD_XDC = $(wildcard projects/$(PROJECT)/cfg/$(BOARD)/$(BOARD_VER)/xdc/*.xdc)
$(info --------------------------)
$(info ---- Project cores found using `scripts/make/get_cores_from_tcl.sh projects/$(PROJECT)/block_design.tcl`:)
$(info ----   $(PROJECT_CORES))
$(info --------------------------)
$(info ---- XDC files found in `projects/$(PROJECT)/cfg/$(BOARD)/$(BOARD_VER)/xdc/`:)
$(info ----   $(BOARD_XDC))



endif # Clean check
$(info --------------------------)

# Set up commands
VIVADO = vivado -nolog -nojournal -mode batch
XSCT = xsct
RM = rm -rf

#############################################



#############################################
## Make-specific targets (.PHONY, etc.)
#############################################

# Files not to delete on half-completion (GNU Make 4.9)
.PRECIOUS: tmp/cores/% tmp/%.xpr tmp/%.bit

# Targets that aren't real files (GNU Make 4.9)
.PHONY: all help tests write_sd petalinux_cfg petalinux_rootfs_cfg petalinux_kernel_cfg clean_sd clean_project clean_build clean_tests clean_test_results clean_all bit sd rootfs boot cores xpr xsa petalinux petalinux_build

# Enable secondary expansion (GNU Make 3.9) to allow for more complex pattern matching (see cores target)
.SECONDEXPANSION:

# Default target is the first listed (GNU Make 2.3)
all: sd


#############################################
## Script targets (tests, sd management, clean, etc. targets)
#############################################

# Print all the available targets
help:
	@echo "Makefile for building Zynq-based projects"
	@echo "Optionally set the variables PROJECT, BOARD, BOARD_VER, OFFLINE, and MOUNT_DIR"
	@echo "  - e.g. 'make PROJECT=ex02_axi_interface BOARD=snickerdoodle_black BOARD_VER=1.0 OFFLINE=true'"
	@echo ""
	@echo "Available targets:"
	@echo "  all                    - Build the SD card image for the project"
	@echo "  tests                  - Run all the tests for the custom cores necessary for the project"
	@echo "  write_sd               - Write the SD card image to the mount point (will clean first)"
	@echo "                           (set custom MOUNT_DIR to the mount point of the SD card if needed)"
	@echo "  petalinux_cfg          - Write or update the PetaLinux system configuration file"
	@echo "  petalinux_rootfs_cfg   - Write or update the PetaLinux root filesystem configuration file"
	@echo "  petalinux_kernel_cfg   - Write or update the PetaLinux kernel configuration file"
	@echo "  clean_sd               - Clean the SD card at the mount point"
	@echo "                           (set custom MOUNT_DIR to the mount point of the SD card if needed)"
	@echo "  clean_project          - Remove a single project's intermediate and temporary files, including cores"
	@echo "  clean_build            - Remove all the intermediate and temporary files"
	@echo "  clean_tests            - Remove all the result files from the tests for PROJECT, leaving the test status"
	@echo "  clean_all_tests        - Remove all the result files from the tests for all projects, leaving the test status"
	@echo "  clean_test_results     - Clean all test status and summary files from custom cores in PROJECT"
	@echo "  clean_all_test_results - Clean all test status and summary files from custom cores in all projects"
	@echo "  clean_all              - Remove all the output files too"
	@echo "  bit                    - Build the bitstream file (system.bit) to the 'out' directory"
	@echo "  sd                     - Build all the files necessary for a bootable SD card to the 'out' directory"
	@echo "  rootfs                 - Build the compressed root filesystem to the 'out' directory"
	@echo "  boot                   - Build the compressed boot files to the 'out' directory"
	@echo "  cores                  - Build all the cores necessary for the project in the 'tmp' directory"
	@echo "  xpr                    - Build the Xilinx project file (project.xpr) in the 'tmp' directory"
	@echo "  xsa                    - Build the hardware definition file (hw_def.xsa) in the 'tmp' directory"
	@echo "  petalinux              - Create the PetaLinux project without building it in the 'tmp' directory"
	@echo "  petalinux_build        - Build the PetaLinux project in the 'tmp' directory"

# Test summary for all the custom cores necessary for the project
tests: projects/${PROJECT}/tests/core_tests_summary

# Write the SD card image to the mount point
write_sd: sd
	@./scripts/make/status.sh "WRITING SD CARD IMAGE"
	./scripts/make/write_sd.sh $(BOARD) $(BOARD_VER) $(PROJECT) $(MOUNT_DIR) --clean

# Write or update the PetaLinux system configuration file
petalinux_cfg: xsa
	@./scripts/make/status.sh "CONFIGURING PETALINUX PROJECT"
	./scripts/petalinux/config_system.sh $(BOARD) $(BOARD_VER) $(PROJECT)

# Write or update the PetaLinux root filesystem configuration file
petalinux_rootfs_cfg: xsa
	@./scripts/make/status.sh "CONFIGURING PETALINUX ROOTFS"
	./scripts/petalinux/config_rootfs.sh $(BOARD) $(BOARD_VER) $(PROJECT)

# Write or update the PetaLinux kernel configuration file
petalinux_kernel_cfg: xsa
	@./scripts/make/status.sh "CONFIGURING PETALINUX KERNEL"
	./scripts/petalinux/config_kernel.sh $(BOARD) $(BOARD_VER) $(PROJECT) $(OFFLINE)

# Clean the SD card image at the mount point
clean_sd:
	@./scripts/make/status.sh "CLEANING SD CARD IMAGE"
	./scripts/make/clean_sd.sh $(MOUNT_DIR)

# Remove a single project's intermediate and temporary files, including cores
clean_project:
	@./scripts/make/status.sh "CLEANING PROJECT: $(BOARD)/$(BOARD_VER)/$(PROJECT)"
	$(RM) tmp/$(BOARD)/$(BOARD_VER)/$(PROJECT)

# Remove all the intermediate and temporary files
clean_build:
	@./scripts/make/status.sh "CLEANING"
	$(RM) .Xil
	$(RM) tmp
	$(RM) tmp_reports

# Remove all the result files from the tests for a single project, leaving the test status
clean_tests:
	@./scripts/make/status.sh "CLEANING TESTS"
	$(RM) projects/$(PROJECT)/cores/*/*/tests/results

# Remove all the result files from the tests for all projects, leaving the test status
clean_all_tests: clean_tests
	@./scripts/make/status.sh "CLEANING ALL TESTS"
	$(RM) projects/*/cores/*/*/tests/results

# Clean all test status and summary files from custom cores and project folder
clean_test_results: clean_tests
	@./scripts/make/status.sh "CLEANING TEST RESULTS"
	$(RM) projects/$(PROJECT)/cores/*/*/tests/test_status
	$(RM) projects/$(PROJECT)/tests/core_tests_summary

# Clean all test status and summary files from custom cores and all project folders
clean_all_test_results: clean_all_tests
	@./scripts/make/status.sh "CLEANING ALL TEST RESULTS"
	$(RM) projects/*/cores/*/*/tests/test_status
	$(RM) projects/*/tests/core_tests_summary

# Remove all the output files too
clean_all: clean_build clean_test_results
	@./scripts/make/status.sh "FULL CLEAN"
	$(RM) out

#############################################


#############################################
## Custom output targets (the important output stages)
#############################################

# All the files necessary for a bootable SD card. See PetaLinux UG1144 for info
# https://docs.amd.com/r/en-US/ug1144-petalinux-tools-reference-guide/Preparing-the-SD-Card
sd: boot rootfs

# The bitstream file (system.bit)
# Built from the Vivado project (project.xpr)
bit: out/$(BOARD)/$(BOARD_VER)/$(PROJECT)/system.bit

# The compressed root filesystem
# Made in the petalinux build
rootfs: out/$(BOARD)/$(BOARD_VER)/$(PROJECT)/rootfs.tar.gz

# The compressed boot files
# Requires the petalinux build (which will make the rootfs)
boot: out/$(BOARD)/$(BOARD_VER)/$(PROJECT)/BOOT.tar.gz

#############################################


#############################################
## Custom intermediate targets (could be used directly for testing)
#############################################

# All the cores necessary for the project
# Separated in `tmp/cores` by vendor
# The necessary cores for the specific project are extracted
# 	from `block_design.tcl` (recursively by sub-modules)
#		by `scripts/make/get_cores_from_tcl.sh`
cores: $(addprefix tmp/$(BOARD)/$(BOARD_VER)/$(PROJECT)/cores/, $(PROJECT_CORES))

# The Xilinx project file
# This file can be edited in Vivado to test Tcl commands and changes
xpr: tmp/$(BOARD)/$(BOARD_VER)/$(PROJECT)/project.xpr

# The hardware definition file
# This file is used by petalinux to build the linux system
xsa: tmp/$(BOARD)/$(BOARD_VER)/$(PROJECT)/hw_def.xsa

# The PetaLinux project
# This project is used to build the linux system
petalinux: tmp/$(BOARD)/$(BOARD_VER)/$(PROJECT)/petalinux/project-spec

# Build the PetaLinux project
petalinux_build: tmp/$(BOARD)/$(BOARD_VER)/$(PROJECT)/petalinux/images/linux/BOOT.tar.gz

#############################################



#############################################
## Specific targets (don't recommend using these directly)
#############################################

# Test status file for a custom core
# This is used to test the core in Vivado
# The test status file is generated by the `scripts/make/test_core.sh` script
# This make target uses "pattern-specific variables" (GNU Make 6.12) to set the vendor and core
#  as well as "secondary expansion" (GNU Make 3.9) to allow for their use in the prerequisite
projects/${PROJECT}/cores/%/tests/test_status: VENDOR = $(word 1,$(subst /, ,$*))
projects/${PROJECT}/cores/%/tests/test_status: CORE = $(word 2,$(subst /, ,$*))
projects/${PROJECT}/cores/%/tests/test_status: projects/${PROJECT}/cores/$$(VENDOR)/$$(CORE)/$$(CORE).v $$(wildcard projects/${PROJECT}/cores/$$(VENDOR)/$$(CORE)/tests/src/*) $$(wildcard projects/${PROJECT}/cores/$$(VENDOR)/$$(CORE)/submodules/*.v) scripts/make/test_core.sh scripts/make/cocotb.mk
	@./scripts/make/status.sh "MAKING TEST STATUS FILE FOR CORE: '$(CORE)' by '$(VENDOR)' in '$(PROJECT)'"
	mkdir -p $(@D)
	scripts/make/test_core.sh $(PROJECT) $(VENDOR) $(CORE)

# Test summary for all the custom cores necessary for the project
# The necessary cores for the specific project are extracted
# 	from `block_design.tcl` (recursively by sub-modules)
#		by `scripts/make/get_cores_from_tcl.sh`
projects/${PROJECT}/tests/core_tests_summary: $(addprefix projects/${PROJECT}/cores/, $(addsuffix /tests/test_status, $(PROJECT_CORES))) scripts/make/test_core.sh scripts/make/cocotb.mk
	@./scripts/make/status.sh "MAKING TEST SUMMARY FOR PROJECT: $(PROJECT)"
	mkdir -p $(@D)
	echo "Test summary of custom cores for project $(PROJECT) on $$(date +"%Y/%m/%d at %H:%M %Z"):" > $@
	@echo "" >> $@
	@for core in $(PROJECT_CORES); do \
		VENDOR=$$(echo $$core | cut -d'/' -f1); \
		CORE=$$(echo $$core | cut -d'/' -f2); \
		TEST_CERT=projects/${PROJECT}/cores/$$VENDOR/$$CORE/tests/test_status; \
		echo "$$VENDOR/cores/$$CORE:" >> $@; \
		echo "  - $$(cat $$TEST_CERT)" >> $@; \
	done

# Core RTL needs to be packaged to be used in the block design flow
# Cores are packaged using the `scripts/vivado/package_core.tcl` script
# This make target uses "pattern-specific variables" (GNU Make 6.12) to set the vendor and core
#  as well as "secondary expansion" (GNU Make 3.9) to allow for their use in the prerequisite
tmp/$(BOARD)/$(BOARD_VER)/$(PROJECT)/cores/%: VENDOR = $(word 1,$(subst /, ,$*))
tmp/$(BOARD)/$(BOARD_VER)/$(PROJECT)/cores/%: CORE = $(word 2,$(subst /, ,$*))
tmp/$(BOARD)/$(BOARD_VER)/$(PROJECT)/cores/%: projects/${PROJECT}/cores/$$(VENDOR)/$$(CORE)/$$(CORE).v $$(wildcard projects/${PROJECT}/cores/$$(VENDOR)/$$(CORE)/submodules/*.v) scripts/vivado/package_core.tcl
	@./scripts/make/status.sh "MAKING USER CORE: '$(CORE)' by '$(VENDOR)'"
	mkdir -p $(@D)
	$(VIVADO) -source scripts/vivado/package_core.tcl -tclargs $(BOARD) $(BOARD_VER) $(PROJECT) $(VENDOR) $(CORE) $(PART)

# The project file (.xpr)
# Requires all the cores
# Built using the `scripts/vivado/project.tcl` script, which uses
# 	the block design and ports files from the project
tmp/$(BOARD)/$(BOARD_VER)/$(PROJECT)/project.xpr: scripts/vivado/project.tcl projects/$(PROJECT)/block_design.tcl $(BOARD_XDC) $(addprefix tmp/$(BOARD)/$(BOARD_VER)/$(PROJECT)/cores/, $(PROJECT_CORES)) $(wildcard projects/$(PROJECT)/modules/*.tcl) scripts/vivado/project.tcl
	@./scripts/make/status.sh "MAKING PROJECT: $(BOARD)/$(BOARD_VER)/$(PROJECT)/project.xpr"
	mkdir -p $(@D)
	$(VIVADO) -source scripts/vivado/project.tcl -tclargs $(BOARD) $(BOARD_VER) $(PROJECT)

# The bitstream file (.bit)
# Requires the project file
# Built using the `scripts/vivado/bitstream.tcl` script, with bitstream compression set to false
tmp/$(BOARD)/$(BOARD_VER)/$(PROJECT)/bitstream.bit: tmp/$(BOARD)/$(BOARD_VER)/$(PROJECT)/project.xpr scripts/vivado/bitstream.tcl
	@./scripts/make/status.sh "MAKING BITSTREAM: $(BOARD)/$(BOARD_VER)/$(PROJECT)/bitstream.bit"
	$(VIVADO) -source scripts/vivado/bitstream.tcl -tclargs $(BOARD)/$(BOARD_VER)/$(PROJECT) false

# The hardware definition file
# Requires the project file
# Built using the scripts/vivado/hw_def.tcl script
# This target uses a `-` before the first VIVADO command to ignore errors,
#   then tracks if there was actually an error. This allows the second script to run,
#   logging the utilization, but still exit with an error at the end if the first command failed.
#   Note the `;` and `\` that make these steps a single command line, as make runs each line separately.
tmp/$(BOARD)/$(BOARD_VER)/$(PROJECT)/hw_def.xsa: tmp/$(BOARD)/$(BOARD_VER)/$(PROJECT)/project.xpr scripts/vivado/hw_def.tcl scripts/vivado/utilization.tcl
	@./scripts/make/status.sh "MAKING HW DEF: $(BOARD)/$(BOARD_VER)/$(PROJECT)/hw_def.xsa"
	$(VIVADO) -source scripts/vivado/hw_def.tcl -tclargs $(BOARD)/$(BOARD_VER)/$(PROJECT); \
		RESULT=$$?; \
		./scripts/make/status.sh "WRITING UTILIZATION: $(BOARD)/$(BOARD_VER)/$(PROJECT)/hw_def.xsa"; \
		$(VIVADO) -source scripts/vivado/utilization.tcl -tclargs $(BOARD)/$(BOARD_VER)/$(PROJECT); \
		if [ $$RESULT -ne 0 ]; then \
			echo "Error: Vivado hw_def.tcl failed with exit code $$RESULT"; \
			exit $$RESULT; \
		fi

# The PetaLinux project specification directory
# Requires the hardware definition file
# Built using the scripts/petalinux/project.sh script
tmp/$(BOARD)/$(BOARD_VER)/$(PROJECT)/petalinux/project-spec: tmp/$(BOARD)/$(BOARD_VER)/$(PROJECT)/hw_def.xsa $(shell find projects/$(PROJECT)/cfg/$(BOARD)/$(BOARD_VER)/petalinux/$(PETALINUX_VERSION) -type f) $(shell find projects/$(PROJECT)/software -type f) $(shell find scripts/petalinux -type f)
	@./scripts/make/status.sh "MAKING CONFIGURED PETALINUX PROJECT: $(BOARD)/$(BOARD_VER)/$(PROJECT)/petalinux"
	scripts/petalinux/project.sh $(BOARD) $(BOARD_VER) $(PROJECT) $(OFFLINE)
	scripts/petalinux/software.sh $(BOARD) $(BOARD_VER) $(PROJECT)
	scripts/petalinux/kernel_modules.sh $(BOARD) $(BOARD_VER) $(PROJECT)
	scripts/petalinux/device_tree.sh $(BOARD) $(BOARD_VER) $(PROJECT)

# The compressed root filesystem
# Requires the PetaLinux project specification directory
tmp/$(BOARD)/$(BOARD_VER)/$(PROJECT)/petalinux/images/linux/rootfs.tar.gz: tmp/$(BOARD)/$(BOARD_VER)/$(PROJECT)/petalinux/project-spec scripts/petalinux/package_rootfs_files.sh $(wildcard projects/$(PROJECT)/rootfs_include/*)
	@./scripts/make/status.sh "MAKING LINUX SYSTEM FOR: $(BOARD)/$(BOARD_VER)/$(PROJECT)/petalinux"
	cd tmp/$(BOARD)/$(BOARD_VER)/$(PROJECT)/petalinux && \
		source $(PETALINUX_PATH)/settings.sh && \
		petalinux-build
	@./scripts/make/status.sh "PACKAGING ADDITIONAL ROOTFS FILES FOR: $(BOARD)/$(BOARD_VER)/$(PROJECT)/petalinux"
	scripts/petalinux/package_rootfs_files.sh $(BOARD) $(BOARD_VER) $(PROJECT)

# The compressed boot files
# Requires the root filesystem
# Built using the petalinux package boot command
tmp/$(BOARD)/$(BOARD_VER)/$(PROJECT)/petalinux/images/linux/BOOT.tar.gz: tmp/$(BOARD)/$(BOARD_VER)/$(PROJECT)/petalinux/images/linux/rootfs.tar.gz scripts/petalinux/package_boot.sh
	@./scripts/make/status.sh "PACKAGING BOOT FILES FOR: $(BOARD)/$(BOARD_VER)/$(PROJECT)/petalinux"
	scripts/petalinux/package_boot.sh $(BOARD) $(BOARD_VER) $(PROJECT)

# The bitstream file copied to the output directory
out/$(BOARD)/$(BOARD_VER)/$(PROJECT)/system.bit: tmp/$(BOARD)/$(BOARD_VER)/$(PROJECT)/bitstream.bit
	mkdir -p $(@D)
	cp tmp/$(BOARD)/$(BOARD_VER)/$(PROJECT)/bitstream.bit $@

# The compressed boot files copied to the output directory
out/$(BOARD)/$(BOARD_VER)/$(PROJECT)/BOOT.tar.gz: tmp/$(BOARD)/$(BOARD_VER)/$(PROJECT)/petalinux/images/linux/BOOT.tar.gz
	mkdir -p $(@D)
	cp tmp/$(BOARD)/$(BOARD_VER)/$(PROJECT)/petalinux/images/linux/BOOT.tar.gz $@

# The compressed root filesystem copied to the output directory
out/$(BOARD)/$(BOARD_VER)/$(PROJECT)/rootfs.tar.gz: tmp/$(BOARD)/$(BOARD_VER)/$(PROJECT)/petalinux/images/linux/rootfs.tar.gz
	mkdir -p $(@D)
	cp tmp/$(BOARD)/$(BOARD_VER)/$(PROJECT)/petalinux/images/linux/rootfs.tar.gz $@
