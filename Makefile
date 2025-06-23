#############################################
## Variables
#############################################
## Required environment variable inputs to the Makefile
#############################################

# You need to set PROJECT, BOARD, and BOARD_VERto the project and board you want to build
# These can be set on the command line:
# - e.g. 'make PROJECT=ex02_axi_interface BOARD=snickerdoodle_black BOARD_VER=1.0'

# Default values for PROJECT and BOARD
PROJECT ?= rev_d_shim
BOARD ?= snickerdoodle_black
BOARD_VER ?= 1.0
OFFLINE ?= false

#############################################



#############################################
## Initialization
#############################################
## Some scripts to initialize the environment and check for necessary tools/src
#############################################

# Check for the REV D environment variable
ifneq ($(shell pwd),$(REV_D_DIR))
$(warning Environment variable REV_D_DIR does not match the current directory)
$(warning - Current directory: $(shell pwd))
$(error - REV_D_DIR: $(REV_D_DIR))
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

# Check if the target is only clean or clean_all (to avoid unnecessary checks)
CLEAN_ONLY = false
ifneq ($(),$(MAKECMDGOALS))
ifeq ($(),$(filter-out clean clean_all,$(MAKECMDGOALS)))
CLEAN_ONLY = true
endif
endif

$(info --------------------------)
ifeq ($(),$(MAKECMDGOALS))
$(info ---- Making "all")
else
$(info ---- Making "$(MAKECMDGOALS)")
endif

# Run some checks and setup, but only if there are targets other than clean or clean_all
ifneq (true, $(CLEAN_ONLY)) # Clean check
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
## Make-specific targets (clean, all, .PHONY, etc.)
#############################################

# Files not to delete on half-completion (GNU Make 4.9)
.PRECIOUS: tmp/cores/% tmp/%.xpr tmp/%.bit

# Targets that aren't real files (GNU Make 4.9)
.PHONY: all clean clean_project clean_all bit sd rootfs boot tests cores xpr xsa petalinux petalinux_build

# Enable secondary expansion (GNU Make 3.9) to allow for more complex pattern matching (see cores target)
.SECONDEXPANSION:

# Default target is the first listed (GNU Make 2.3)
all: sd

# Remove a single project's intermediate and temporary files, including cores
clean_project:
	@./scripts/make/status.sh "CLEANING PROJECT: $(BOARD)/$(BOARD_VER)/$(PROJECT)"
	$(RM) tmp/$(BOARD)/$(BOARD_VER)/$(PROJECT)
	$(RM) -r $(addsuffix *, $(addprefix tmp/custom_cores/, $(PROJECT_CORES)))

# Remove all the intermediate and temporary files
clean:
	@./scripts/make/status.sh "CLEANING"
	$(RM) .Xil
	$(RM) tmp
	$(RM) tmp_reports

# Remove all the output files too
clean_all: clean
	@./scripts/make/status.sh "CLEANING OUTPUT FILES"
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
bit: tmp/$(BOARD)/$(BOARD_VER)/$(PROJECT)/bitstream.bit
	mkdir -p out/$(BOARD)/$(BOARD_VER)/$(PROJECT)
	cp tmp/$(BOARD)/$(BOARD_VER)/$(PROJECT)/bitstream.bit out/$(BOARD)/$(BOARD_VER)/$(PROJECT)/system.bit

# The compressed root filesystem
# Made in the petalinux build
rootfs: tmp/$(BOARD)/$(BOARD_VER)/$(PROJECT)/petalinux/images/linux/rootfs.tar.gz
	mkdir -p out/$(BOARD)/$(BOARD_VER)/$(PROJECT)
	cp tmp/$(BOARD)/$(BOARD_VER)/$(PROJECT)/petalinux/images/linux/rootfs.tar.gz out/$(BOARD)/$(BOARD_VER)/$(PROJECT)/rootfs.tar.gz

# The compressed boot files
# Requires the petalinux build (which will make the rootfs)
boot: tmp/$(BOARD)/$(BOARD_VER)/$(PROJECT)/petalinux/images/linux/BOOT.tar.gz
	mkdir -p out/$(BOARD)/$(BOARD_VER)/$(PROJECT)
	cp tmp/$(BOARD)/$(BOARD_VER)/$(PROJECT)/petalinux/images/linux/BOOT.tar.gz out/$(BOARD)/$(BOARD_VER)/$(PROJECT)/BOOT.tar.gz


#############################################


#############################################
## Custom intermediate targets (could be used directly for testing)
#############################################

# Test summary for all the custom cores necessary for the project
tests: projects/${PROJECT}/tests/core_tests_summary

# All the cores necessary for the project
# Separated in `tmp/cores` by vendor
# The necessary cores for the specific project are extracted
# 	from `block_design.tcl` (recursively by sub-modules)
#		by `scripts/make/get_cores_from_tcl.sh`
cores: $(addprefix tmp/custom_cores/, $(PROJECT_CORES))

# The Xilinx project file
# This file can be edited in Vivado to test TCL commands and changes
xpr: tmp/$(BOARD)/$(BOARD_VER)/$(PROJECT)/project.xpr

# The hardware definition file
# This file is used by petalinux to build the linux system
xsa: tmp/$(BOARD)/$(BOARD_VER)/$(PROJECT)/hw_def.xsa

# The PetaLinux project
# This project is used to build the linux system
petalinux: tmp/$(BOARD)/$(BOARD_VER)/$(PROJECT)/petalinux

# Build the PetaLinux project
petalinux_build: petalinux
	@./scripts/make/status.sh "MAKING LINUX SYSTEM FOR: $(BOARD)/$(BOARD_VER)/$(PROJECT)/petalinux"
	cd tmp/$(BOARD)/$(BOARD_VER)/$(PROJECT)/petalinux && \
		source $(PETALINUX_PATH)/settings.sh && \
		petalinux-build

#############################################



#############################################
## Specific targets (don't recommend using these directly)
#############################################

# Test certificate for a custom core
# This is used to test the core in Vivado
# The test certificate is generated by the `scripts/make/test_core.sh` script
# This make target uses "pattern-specific variables" (GNU Make 6.12) to set the vendor and core
#  as well as "secondary expansion" (GNU Make 3.9) to allow for their use in the prerequisite
custom_cores/%/tests/test_status: VENDOR = $(word 1,$(subst /, ,$*))
custom_cores/%/tests/test_status: CORE = $(word 3,$(subst /, ,$*))
custom_cores/%/tests/test_status: custom_cores/$$(VENDOR)/cores/$$(CORE)/$$(CORE).v $$(wildcard custom_cores/$$(VENDOR)/cores/$$(CORE)/tests/src/*) $$(wildcard custom_cores/$$(VENDOR)/cores/$$(CORE)/submodules/*.v) scripts/make/test_core.sh scripts/make/cocotb.mk
	@./scripts/make/status.sh "MAKING TEST CERTIFICATE FOR CORE: '$(CORE)' by '$(VENDOR)'"
	mkdir -p $(@D)
	scripts/make/test_core.sh $(VENDOR) $(CORE)

# Test summary for all the custom cores necessary for the project
# The necessary cores for the specific project are extracted
# 	from `block_design.tcl` (recursively by sub-modules)
#		by `scripts/make/get_cores_from_tcl.sh`
projects/${PROJECT}/tests/core_tests_summary: $(addprefix custom_cores/, $(addsuffix /tests/test_status, $(foreach core,$(PROJECT_CORES),$(subst /,/cores/,$(core))))) scripts/make/test_core.sh scripts/make/cocotb.mk
	@./scripts/make/status.sh "MAKING TEST SUMMARY FOR PROJECT: $(PROJECT)"
	mkdir -p $(@D)
	echo "Test summary of custom cores for project $(PROJECT) on $$(date +"%Y/%m/%d at %H:%M %Z"):" > $@
	echo "" >> $@
	for core in $(PROJECT_CORES); do \
		VENDOR=$$(echo $$core | cut -d'/' -f1); \
		CORE=$$(echo $$core | cut -d'/' -f2); \
		TEST_CERT=custom_cores/$$VENDOR/cores/$$CORE/tests/test_status; \
		echo "$$VENDOR/cores/$$CORE:" >> $@; \
		echo "  - $$(cat $$TEST_CERT)" >> $@; \
	done

# Core RTL needs to be packaged to be used in the block design flow
# Cores are packaged using the `scripts/vivado/package_core.tcl` script
# This make target uses "pattern-specific variables" (GNU Make 6.12) to set the vendor and core
#  as well as "secondary expansion" (GNU Make 3.9) to allow for their use in the prerequisite
tmp/custom_cores/%: VENDOR = $(word 1,$(subst /, ,$*))
tmp/custom_cores/%: CORE = $(word 2,$(subst /, ,$*))
tmp/custom_cores/%: custom_cores/$$(VENDOR)/cores/$$(CORE)/$$(CORE).v $$(wildcard custom_cores/$$(VENDOR)/cores/$$(CORE)/submodules/*.v) scripts/vivado/package_core.tcl
	@./scripts/make/status.sh "MAKING USER CORE: '$(CORE)' by '$(VENDOR)'"
	mkdir -p $(@D)
	$(VIVADO) -source scripts/vivado/package_core.tcl -tclargs $(VENDOR) $(CORE) $(PART)

# The project file (.xpr)
# Requires all the cores
# Built using the `scripts/vivado/project.tcl` script, which uses
# 	the block design and ports files from the project
tmp/$(BOARD)/$(BOARD_VER)/$(PROJECT)/project.xpr: scripts/vivado/project.tcl projects/$(PROJECT)/block_design.tcl projects/$(PROJECT)/ports.tcl $(BOARD_XDC) $(addprefix tmp/custom_cores/, $(PROJECT_CORES)) $(wildcard projects/$(PROJECT)/modules/*.tcl) scripts/vivado/project.tcl
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
	-$(VIVADO) -source scripts/vivado/hw_def.tcl -tclargs $(BOARD)/$(BOARD_VER)/$(PROJECT); \
		RESULT=$$?; \
		./scripts/make/status.sh "WRITING UTILIZATION: $(BOARD)/$(BOARD_VER)/$(PROJECT)/hw_def.xsa"; \
		$(VIVADO) -source scripts/vivado/utilization.tcl -tclargs $(BOARD)/$(BOARD_VER)/$(PROJECT); \
		if [ $$RESULT -ne 0 ]; then exit $$RESULT; fi

# The PetaLinux project
# Requires the hardware definition file
# Built using the scripts/petalinux/project.sh script
tmp/$(BOARD)/$(BOARD_VER)/$(PROJECT)/petalinux: tmp/$(BOARD)/$(BOARD_VER)/$(PROJECT)/hw_def.xsa projects/$(PROJECT)/cfg/$(BOARD)/$(BOARD_VER)/petalinux/$(PETALINUX_VERSION)/config.patch projects/$(PROJECT)/cfg/$(BOARD)/$(BOARD_VER)/petalinux/$(PETALINUX_VERSION)/rootfs_config.patch scripts/petalinux/project.sh scripts/petalinux/software.sh scripts/petalinux/kernel_modules.sh scripts/petalinux/make_offline.sh
	@./scripts/make/status.sh "MAKING CONFIGURED PETALINUX PROJECT: $(BOARD)/$(BOARD_VER)/$(PROJECT)/petalinux"
	@if [ $(OFFLINE) = "true" ]; then scripts/make/status.sh "PetaLinux OFFLINE build"; fi
	scripts/petalinux/project.sh $(BOARD) $(BOARD_VER) $(PROJECT)
	scripts/petalinux/software.sh $(BOARD) $(BOARD_VER) $(PROJECT)
	scripts/petalinux/kernel_modules.sh $(BOARD) $(BOARD_VER) $(PROJECT)
	@if [ $(OFFLINE) = "true" ]; then scripts/petalinux/make_offline.sh $(BOARD) $(BOARD_VER) $(PROJECT); fi
	

# The compressed root filesystem
# Requires the PetaLinux project
tmp/$(BOARD)/$(BOARD_VER)/$(PROJECT)/petalinux/images/linux/rootfs.tar.gz: tmp/$(BOARD)/$(BOARD_VER)/$(PROJECT)/petalinux scripts/petalinux/package_rootfs_files.sh
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
