# This script creates a custom Vivado IP core for a given core path and part name.
# The cores are packaged to tmp/board/board_version/project/[vendor_name]/[core_name] and used in the project.tcl script.

package require fileutil
package require json

# Check that the correct number of arguments are provided
if {[llength $argv] != 6} {
    error "Expected 6 arguments: board_name, board_version, project_name, vendor_name, core_name, part_name"
}
# First argument is the board name
set board_name [lindex $argv 0]
# Second argument is the board version
set board_version [lindex $argv 1]
# Third argument is the project name
set project_name [lindex $argv 2]
# Fourth argument is the vendor name
set vendor_name [lindex $argv 3]
# Fifth argument is the core name
set core_name [lindex $argv 4]
# Sixth argument is the part name
set part_name [lindex $argv 5]

# Set the source vendor path
set src_vendor_path projects/${project_name}/cores/${vendor_name}
# Set the source directory for the core
set src_path ${src_vendor_path}/${core_name}
# Set the vendor path
set tmp_vendor_path tmp/${board_name}/${board_version}/${project_name}/cores/${vendor_name}
# Set the temporary build directory
set tmp_path ${tmp_vendor_path}/${core_name}


# Clear out old build files
file delete -force $tmp_path $tmp_path.cache $tmp_path.hw $tmp_path.ip_user_files $tmp_path.sim $tmp_path.xpr

# Create the core project
create_project -part $part_name $core_name $tmp_vendor_path

# Add the main source file to the core project
add_files -norecurse $src_path/$core_name.v

# Set the main (core_name) module as the top module
set_property TOP $core_name [current_fileset]

# Load in the other source files (submodules from the core directory)
set files [glob -nocomplain $src_path/submodules/*.v]
if {[llength $files] > 0} {
  add_files -norecurse $files
}

# Set the IP packaging path
ipx::package_project -root_dir $tmp_path -import_files

# Get the Vivado core
set core [ipx::current_core]

# Set the core properties
set_property VERSION {1.0} $core
set_property NAME $core_name $core
set_property LIBRARY {user} $core
set_property VENDOR $vendor_name $core

## Extract the info for the vendor display name and company URL
# Read the json config for the board into a dict
set vendor_info_fname ${src_vendor_path}/vendor_info.json
if {[file exists $vendor_info_fname]} {
    set vendor_info_fd [open $vendor_info_fname "r"]
} else {
    error "Vendor info file ${vendor_info_fname} missing."
}
set vendor_info_str [read $vendor_info_fd]
close $vendor_info_fd
set vendor_info_dict [json::json2dict $vendor_info_str]

set_property VENDOR_DISPLAY_NAME [dict get $vendor_info_dict display_name] $core
set_property COMPANY_URL [dict get $vendor_info_dict url] $core
set_property SUPPORTED_FAMILIES {zynq Production} $core

puts "Packaging core: $vendor_name:user:$core_name:1.0"

# Package the core
ipx::create_xgui_files $core
ipx::update_checksums $core
ipx::save_core $core

# Exit the core project
close_project
