# This script creates a Vivado project for a given board and project name.

package require fileutil
package require json

# Arguments: board_name project_name
set board_name [lindex $argv 0]
set board_ver [lindex $argv 1]
set project_name [lindex $argv 2]

# Set the temporary directory for the project
set tmp_dir tmp/$board_name/$board_ver/$project_name

## Extract the part information from the board name and version using the provided scripts
# Define the paths to the scripts
set get_part_script [file join [pwd] scripts/make/get_part.sh]
set get_board_part_script [file join [pwd] scripts/make/get_board_part.sh]

# Ensure the scripts exist
if {![file exists $get_part_script]} {
  error "Script $get_part_script missing."
}
if {![file exists $get_board_part_script]} {
  error "Script $get_board_part_script missing."
}

# Get the part name
set part_name [exec bash $get_part_script $board_name $board_ver]
if {$part_name eq ""} {
  error "Failed to retrieve part name using $get_part_script for board $board_name version $board_ver."
} else {
  puts "Part name: $part_name"
}

# Get the board part
set board_part [exec bash $get_board_part_script $board_name $board_ver]
if {$board_part eq ""} {
  error "Failed to retrieve board part using $get_board_part_script for board $board_name version $board_ver."
} else {
  puts "Board part: $board_part"
}


## Initialize the project and dependencies
# Clear out old build files
file delete -force $tmp_dir/project.cache $tmp_dir/project.gen $tmp_dir/project.hw $tmp_dir/project.ip_user_files $tmp_dir/project.runs $tmp_dir/project.sim $tmp_dir/project.srcs $tmp_dir/project.xpr

# Create the project
create_project -part $part_name project $tmp_dir

# Set the board part
set_property BOARD_PART $board_part [current_project]

# Add the path to the custom IP core packages, if they exist
set cores_list [glob -type d -nocomplain ${tmp_dir}/cores/*]
if {[llength $cores_list] > 0} {
  set_property IP_REPO_PATHS $cores_list [current_project]
}
# Load the custom IP source files
update_ip_catalog


################################################################################
### Define a set of Tcl procedures to simplify the creation of block designs
################################################################################  

# Procedure for connecting (wiring) two pins together
# Can handle both regular and interface pins.
# Attempts to wire normal pins, then interface pins.
proc wire {name1 name2} {
  set pin1 [get_bd_pins $name1]
  set pin2 [get_bd_pins $name2]
  if {[llength $pin1] == 1 && [llength $pin2] == 1} {
    connect_bd_net $pin1 $pin2
    return
  }
  set pin1 [get_bd_intf_pins $name1]
  set pin2 [get_bd_intf_pins $name2]
  if {[llength $pin1] == 1 && [llength $pin2] == 1} {
    connect_bd_intf_net $pin1 $pin2
    return
  }
  error "** ERROR: can't connect $name1 and $name2"
}


# Procedure for assigning an address for a connected AXI interface pin
#  offset: offset of the address
#  range: range of the address
#  target_intf_pin: name of the interface pin which will be assigned an address
#  addr_space_intf_pin: name of the pin containing the address space
proc addr {offset range target_intf_pin addr_space_intf_pin} {
  set addr_space_intf [get_bd_intf_pins $addr_space_intf_pin]
  set addr_space [get_bd_addr_spaces -of_objects $addr_space_intf]
  set target_intf [get_bd_intf_pins $target_intf_pin]
  set segment [get_bd_addr_segs -of_objects $target_intf]
  assign_bd_address -target_address_space $addr_space -offset $offset -range $range $segment
}


# Automate the creation of an AXI interconnect. This creates an intermediary AXI core.
#  offset: offset of the address
#  range: range of the address
#  intf_pin: name of the interface pin to connect to the AXI interconnect
#  manager: name of the manager to connect to the AXI interconnect
#   Note: "manager" needs to be an absolute path with a "/" prefix, "intf_pin" does not
proc auto_connect_axi {offset range intf_pin manager} {
  set object [get_bd_intf_pins $intf_pin]
  set config [list Master $manager Clk Auto]
  apply_bd_automation -rule xilinx.com:bd_rule:axi4 -config $config $object
  addr $offset $range $intf_pin $manager
}


# Procedure for creating a cell
#  cell_vlnv: VLNV of the cell (vendor:library:name:version)
#    Version is optional, usually omitted for custom cores and included for Xilinx cores.
#    For repository IP defined in projects/PROJECT/cores/, the format is:
#      [directory (e.g. base)]:user:[filename (e.g. fifo_async)]
#  cell_name: name of the cell
#  cell_props: dictionary of properties to set
#  cell_conn: dictionary of pins to wire (local_name / remote_name)
proc cell {cell_vlnv cell_name {cell_props {}} {cell_conn {}}} {
  set cell [create_bd_cell -type ip -vlnv $cell_vlnv $cell_name]
  set prop_list {}
  foreach {prop_name prop_value} [uplevel 1 [list subst $cell_props]] {
    lappend prop_list CONFIG.$prop_name $prop_value
  }
  if {[llength $prop_list] > 1} {
    set_property -dict $prop_list $cell
  }
  foreach {local_name remote_name} [uplevel 1 [list subst $cell_conn]] {
    wire $cell_name/$local_name $remote_name
  }
}


# Procedure for initializing the processing system
# Initialize the processing system from the preset defined in
# boards/[board]/board_files/1.0/preset.xml
# then overwrites some properties with those in ps_props
#  ps_name: name of the processing system
#  ps_props: dictionary of properties to set
#  ps_conn: dictionary of pins to wire (local_name / remote_name)
proc init_ps {ps_name {ps_props {}} {ps_conn {}}} {

  # Create the PS
  cell xilinx.com:ip:processing_system7:5.5 $ps_name {} {}

  # Apply the automation configuration
  # - apply_board_preset applies the preset configuration in boards/[board]/board_files/1.0/preset.xml
  # - make_external externalizes the pins
  # - Manager/Subordinate (Master/Slave) control the cross-triggering feature (In/Out, not needed for any projects right now)
  set cfg_list [list apply_board_preset 1 make_external {FIXED_IO, DDR} Master Disable Slave Disable]
  apply_bd_automation -rule xilinx.com:bd_rule:processing_system7 -config $cfg_list [get_bd_cells $ps_name]

  # Set post-automation properties
  set prop_list {}
  foreach {prop_name prop_value} [uplevel 1 [list subst $ps_props]] {
    lappend prop_list CONFIG.$prop_name $prop_value
  }
  if {[llength $prop_list] > 1} {
    set_property -dict $prop_list [get_bd_cells $ps_name]
  }

  # Wire the pins
  foreach {local_name remote_name} [uplevel 1 [list subst $ps_conn]] {
    wire $ps_name/$local_name $remote_name
  }
}


# Procedure for creating a module from a tcl section
#  module_src:  name of the Tcl source file to include
#  module_name: name of the module
#  module_conn: dictionary of pins to wire (put the local name first)
proc module {module_src module_name {module_conn {}}} {
  global project_name
  set upper_instance [current_bd_instance .]
  current_bd_instance [create_bd_cell -type hier $module_name]
  
  # Check if the module source file exists
  if {![file exists projects/${project_name}/modules/${module_src}.tcl]} {
    error "Module source file projects/${project_name}/modules/${module_src}.tcl does not exist."
  }
  
  # Include the module source file
  source projects/$project_name/modules/${module_src}.tcl

  current_bd_instance $upper_instance

  # Wire the local pins externally
  foreach {local_name remote_name} [uplevel 1 [list subst $module_conn]] {
    wire $module_name/$local_name $remote_name
  }
}

# Procedure for bringing a variable from the module calling context to the module context
proc module_get_upvar {varname} {
  if {[uplevel 2 [list info exists $varname]]} {
    return [uplevel 2 [list set $varname]]
  }
  error "Variable $varname does not exist in the context calling the module."
}


##############################################################################
## End of block design helper procedures
##############################################################################

# Initialize the block design
create_bd_design system

# Execute the port definition and block design scripts for the project, by board
source projects/$project_name/block_design.tcl

# Clear out the processes defined above to avoid conflicts, now that the block design is complete
rename wire {}
rename cell {}
rename module {}
rename addr {}

# Store the block design file name
set system [get_files system.bd]

# Disable checkpoints
set_property SYNTH_CHECKPOINT_MODE None $system

# Make the RTL wrapper for the block design
generate_target all $system
make_wrapper -files $system -top

# Store the wrapper file name
set wrapper [fileutil::findByPattern $tmp_dir/project.gen system_wrapper.v]

# Add the wrapper to the project and make it the top module
add_files -norecurse $wrapper
set_property TOP system_wrapper [current_fileset]

# Load all Verilog and SystemVerilog source files from the project folder, as well as any .mem files
set files [glob -nocomplain projects/$project_name/*.v projects/$project_name/*.sv, projects/$project_name/*.mem]
if {[llength $files] > 0} {
  add_files -norecurse $files
}

# Load all XDC constraint files specific to the board from the project folder
set files [glob -nocomplain projects/$project_name/cfg/$board_name/$board_ver/xdc/*.xdc]
if {[llength $files] > 0} {
  add_files -norecurse -fileset constrs_1 $files
}

set_property VERILOG_DEFINE {TOOL_VIVADO} [current_fileset]

set_property STRATEGY Flow_PerfOptimized_high [get_runs synth_1]
set_property STRATEGY Performance_ExploreWithRemap [get_runs impl_1]

close_project
