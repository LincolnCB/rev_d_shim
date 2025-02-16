#############################################
## Variables
#############################################

# You need to set PROJECT and BOARD to the project and board you want to build
# These can be set on the command line:
# - e.g. 'make PROJECT=example_axi_hub_regs BOARD=snickerdoodle_black'

# Default values for PROJECT and BOARD
PROJECT ?= example_axi_hub
BOARD ?= snickerdoodle_black


#############################################
## Initialization
#############################################

# Check for the REV D environment variable
ifneq ($(shell pwd),$(REV_D_DIR))
$(error Environment variable REV_D_DIR does not match the current directory)
$(error - Current directory: $(shell pwd))
$(error - REV_D_DIR: $(REV_D_DIR))
endif

# Check if the target is only clean or cleanall (to avoid unnecessary checks)
CLEAN_ONLY = false
ifneq ($(),$(MAKECMDGOALS))
ifeq ($(),$(filter-out clean cleanall,$(MAKECMDGOALS)))
CLEAN_ONLY = true
endif
endif

# Run some checks and setup, but only if there are targets other than clean or cleanall
ifneq (true, $(CLEAN_ONLY)) # Clean check

# Check that the project and board exist, and that the necessary files are present
ifeq ($(),$(wildcard boards/$(BOARD)/board_config.json))
$(error Board "$(BOARD)" or the corresponding board configuration file "boards/$(BOARD)/board_config.json" does not exist)
endif
ifeq ($(),$(wildcard projects/$(PROJECT)))
$(error Project "$(PROJECT)" does not exist -- missing folder "projects/$(PROJECT)")
endif
ifeq ($(),$(wildcard projects/$(PROJECT)/block_design.tcl))
$(error Project "$(PROJECT)" does not have a block design file "projects/$(PROJECT)/block_design.tcl")
endif
ifeq ($(),$(wildcard projects/$(PROJECT)/ports.tcl))
$(error Project "$(PROJECT)" does not have a ports file "projects/$(PROJECT)/ports.tcl")
endif
ifeq ($(),$(wildcard projects/$(PROJECT)/cfg/$(BOARD)/))
$(error No support for board "$(BOARD)" in project "$(PROJECT)" -- configuration folder "projects/$(PROJECT)/cfg/$(BOARD)/" does not exist or is empty)
endif
ifeq ($(),$(wildcard projects/$(PROJECT)/cfg/$(BOARD)/xdc/))
$(error No support for board "$(BOARD)" in project "$(PROJECT)" -- design constraints folder "projects/$(PROJECT)/cfg/$(BOARD)/xdc/" does not exist or is empty)
endif

# Extract the part and processor from the board configuration file
export PART=$(shell jq -r '.part' boards/$(BOARD)/board_config.json)
export PROC=$(shell jq -r '.proc' boards/$(BOARD)/board_config.json)

# Get the list of necessary cores from the project file to avoid building unnecessary cores
PROJECT_CORES = $(shell ./scripts/make/get_cores_from_tcl.sh projects/$(PROJECT)/block_design.tcl)
$(info Project cores: $(PROJECT_CORES))

endif # Clean check

# Set up commands
VIVADO = vivado -nolog -nojournal -mode batch
XSCT = xsct
RM = rm -rf

# Files not to delete on half-completion (.PRECIOUS is a special target that tells make not to delete these files)
.PRECIOUS: tmp/cores/% tmp/%.xpr tmp/%.bit

# Targets that aren't real files
.PHONY: all clean cleanall bit sd rootfs boot cores xpr xsa

#############################################



#############################################
## Generic and clean targets
#############################################
# Default target
all: sd

# Remove the intermediate and temporary files
clean:
	@./scripts/make/status.sh "CLEANING"
	$(RM) .Xil
	$(RM) tmp

# Remove the output files too
cleanall: clean
	@./scripts/make/status.sh "CLEANING OUTPUT FILES"
	$(RM) out

#############################################


#############################################
## Output targets
#############################################

# The bitstream file
# Built from the Vivado project
bit: tmp/$(BOARD)/$(PROJECT)/bitstream.bit
	mkdir -p out/$(BOARD)/$(PROJECT)
	cp tmp/$(BOARD)/$(PROJECT)/bitstream.bit out/$(BOARD)/$(PROJECT)/system.bit

# All the files necessary for a bootable SD card
sd: boot rootfs

# The compressed root filesystem
# Made in the petalinux build
rootfs: tmp/$(BOARD)/$(PROJECT)/petalinux/images/linux/rootfs.tar.gz
	mkdir -p out/$(BOARD)/$(PROJECT)
	cp tmp/$(BOARD)/$(PROJECT)/petalinux/images/linux/rootfs.tar.gz out/$(BOARD)/$(PROJECT)/rootfs.tar.gz

# The compressed boot files
# Requires the petalinux build (which will make the rootfs)
boot: tmp/$(BOARD)/$(PROJECT)/petalinux/images/linux/rootfs.tar.gz
	mkdir -p out/$(BOARD)/$(PROJECT)
	@./scripts/make/status.sh "PACKAGING BOOT.BIN"
	cd tmp/$(BOARD)/$(PROJECT)/petalinux && \
		source $(PETALINUX_PATH)/settings.sh && \
		petalinux-package --boot \
		--format BIN \
		--fsbl \
		--fpga \
		--kernel \
		--boot-device sd \
		--force
	tar -czf out/$(BOARD)/$(PROJECT)/BOOT.tar.gz \
		-C tmp/$(BOARD)/$(PROJECT)/petalinux/images/linux \
		BOOT.BIN \
		image.ub \
		boot.scr \
		system.dtb


#############################################


#############################################
## Intermediate targets
#############################################

# All the cores necessary for the project
# Separated in `tmp/cores` by vendor
# The necessary cores for the specific project are extracted  
# 	from `block_design.tcl` (recursively by sub-modules)  
#		by `scripts/make/get_cores_from_tcl.sh`
cores: $(addprefix tmp/cores/, $(PROJECT_CORES))

# The Xilinx project file
# This file can be edited in Vivado to test TCL commands and changes
xpr: tmp/$(BOARD)/$(PROJECT)/project.xpr

# The hardware definition file
# This file is used by petalinux to build the linux system
xsa: tmp/$(BOARD)/$(PROJECT)/hw_def.xsa

#############################################



#############################################
## Specific targets
#############################################

# Core RTL needs to be packaged to be used in the block design flow
# Cores are packaged using the `scripts/vivado/package_core.tcl` script
tmp/cores/%: cores/%.v
	@./scripts/make/status.sh "MAKING CORE: $*"
	mkdir -p $(@D)
	$(VIVADO) -source scripts/vivado/package_core.tcl -tclargs $* $(PART)

# The project file (.xpr)
# Requires all the cores
# Built using the `scripts/vivado/project.tcl` script, which uses
# 	the block design and ports files from the project
tmp/$(BOARD)/$(PROJECT)/project.xpr: projects/$(PROJECT) $(addprefix tmp/cores/, $(PROJECT_CORES))
	@./scripts/make/status.sh "MAKING PROJECT: $(BOARD)/$(PROJECT)/project.xpr"
	mkdir -p $(@D)
	$(VIVADO) -source scripts/vivado/project.tcl -tclargs $(BOARD) $(PROJECT)

# The bitstream file (.bit)
# Requires the project file
# Built using the `scripts/vivado/bitstream.tcl` script, with bitstream compression set to false
tmp/$(BOARD)/$(PROJECT)/bitstream.bit: tmp/$(BOARD)/$(PROJECT)/project.xpr
	@./scripts/make/status.sh "MAKING BITSTREAM: $(BOARD)/$(PROJECT)/bitstream.bit"
	$(VIVADO) -source scripts/vivado/bitstream.tcl -tclargs $(BOARD)/$(PROJECT) false

# The hardware definition file
# Requires the project file
# Built using the scripts/vivado/hw_def.tcl script
tmp/$(BOARD)/$(PROJECT)/hw_def.xsa: tmp/$(BOARD)/$(PROJECT)/project.xpr
	@./scripts/make/status.sh "MAKING HW DEF: $(BOARD)/$(PROJECT)/hw_def.xsa"
	$(VIVADO) -source scripts/vivado/hw_def.tcl -tclargs $(BOARD)/$(PROJECT)

# The compressed root filesystem
# Requires the hardware definition file
# Build using the scripts/petalinux/build.sh file
tmp/$(BOARD)/$(PROJECT)/petalinux/images/linux/rootfs.tar.gz: tmp/$(BOARD)/$(PROJECT)/hw_def.xsa
	@./scripts/make/status.sh "MAKING LINUX SYSTEM: $(BOARD)/$(PROJECT)/petalinux"
	source $(PETALINUX_PATH)/settings.sh && \
		scripts/petalinux/build.sh $(BOARD) $(PROJECT)

