***Updated 2025-06-27***

Forked off of and based originally on [Pavel Demin](https://github.com/pavel-demin)'s [Red Pitaya Notes](http://pavel-demin.github.io/red-pitaya-notes/).

Also heavily informed by and related to the Open-MRI [OCRA project](https://github.com/OpenMRI/ocra) (which was forked off of Pavel Demin's repo as well).

Primarily written by [Lincoln Craven-Brightman](https://scholar.google.com/citations?user=be3yqVEAAAAJ&hl=en&oi=ao), with contributions from Kutay Bulun, [Thomas Witzel](https://scholar.google.com/citations?hl=en&user=Pgc_3HwAAAAJ), and [H. Fatih Uǧurdag](https://scholar.google.com/citations?user=IdZE034AAAAJ&hl=en&oi=sra). The Revision D Shim firmware is designed to work with [Don Straney](https://dcstraney.wordpress.com/)'s [Linear Shim hardware (GitHub)](https://github.com/stockmann-lab/shim_amp_hardware_linear). The project is at the request and funding of [Jason Stockmann](https://scholar.google.com/citations?user=PxfOa-0AAAAJ&hl=en), and is a continuation of the work done by Nick Arango and [Irene Kuang](https://www.irenekuang.com/) on previous revisions of the Shim Amplifier system.

# Overview

This repository contains the source code and documentation for the Revision D Shim Amplifier system, as well as a general build framework if you're interested in modifying the system or building your own projects for a Zynq 7000 series SoC. These projects build the files for a bootable SD card that contain the Linux operating system, custom software, and FPGA bitstreams used in that project, which will fully configure your board.

## Sections

- [Overview](#overview)
- [Getting started](#getting-started)
- [Building an SD card](#building-an-sd-card)
- [Example projects](#example-projects)
- [Testing](#testing)

## Background -- Zynq 7000 series

Zynq 7000 SoCs are a series of System on Chip (SoC) devices from AMD/Xilinx that combine an ARM processor with an FPGA. There are several variants of the Zynq 7000 SoC series, including the Zynq 7010, Zynq 7020, and beyond. Different variants will have different I/O, memory, and processing capabilities, but they all share the same basic architecture, which allows most projects to be ported between them with minimal changes (unless you're at the limit of one of those resources and trying to port to a less-capable variant).

Zynq 7000 SoCs are available on a number of boards, including the Red Pitaya, the Snickerdoodle, Zybo boards, and many others. Different boards will have different Zynq variants -- for instance, the Red Pitaya STEMlab 125-14 uses the smaller Zynq 7010 chip, while the Snickerdoodle Black and Red Pitaya SDRlab 122-16 use the midrange Zynq 7020. In addition, different boards will expose different amounts of the Zynq's available I/O -- for instance, while the Snickerdoodle Black and Red Pitaya SDRlab 122-16 have the same Zynq 7020 chip, the Snickerdoodle Black has significantly more I/O pins accessible. 

This repository is designed to allow compatibility with any board that uses a Zynq 7000 series chip, but may require some addition board files and configuration changes to work with a specific board. The full capabilities of the Rev D shim amplifier require the Zynq 7020 or above, as it uses the additional I/O and memory available on those chips. However, reducing the number of shim channels and buffer size would allow for porting.

## Repo structure

This repo is structured to allow for easy building of projects. It's primarily a collection of source code, configuration files, and scripts for the tools used -- Vivado and PetaLinux. There are some additional tools for testing, which you can read about in the [Testing](#testing) section.

The top-level directory contains the following folders (which each contain their own more in-depth README files):
- `boards`: Contains board files for boards that use the Zynq 7000 series SoCs. These files contain information about the board's hardware, like which Zynq variant is used or the I/O pinout. If you want to add support for a new board, you can take their board files (found online) and add a new folder here with the board's name containing the `board_files` folder.
- `custom_cores`: Contains custom cores used in the scripted build of the FPGA system. These cores are separated by "vendor", basically meaning the original author. For instance, the `pavel_demin` folder contains cores that were originally written by Pavel Demin, while the `open-mri` folder contains cores that were originally written by people in the Open-MRI OCRA project (likely Thomas Witzel). Cores in `base` were primarily written by me, Lincoln Craven-Brightman, with contributions from folks like Kutay Bulun, H. Fatih Uǧurdag, and Thomas Witzel. You can add your own custom cores here in your own folder, following the same structure as the others.
- `kernel_modules`: Contains kernel modules that can be included in the Linux kernel build for projects.
- `projects`: Contains the projects that can be built with this repo. Each project has its own folder, and is mainly defined by its `block_design.tcl` file, which defines the FPGA system's block design. Each project will also need folders under `cfg` that define the compatibility with different boards, and can have a few other special folders that augment the build process.
- `scripts`: Contains scripts that are used to build projects, separated by category (`check`, `make`, `petalinux`, and `vivado`).

There are some temporary, untracked folders that contain the intermediate and final build results: `out` and `tmp`. In addition, there's a temporary `tmp_reports` folder that contains Vivado reports for resource utilization.

Finally, there's some files:
- `environment.sh.example`: Needs to be copied as explained in [Profile Setup](#profile-setup)
- `make_defaults.mk.example`: Can optionally be copied as explained in [Optional: Makefile variable defaults](#optional-makefile-variable-defaults)
- `Makefile`: The main Makefile that is used to build everything, see [Building an SD card](#building-an-sd-card).


# Getting started

## Required tools

This repo uses the AMD/Xilinx FPGA toolchain to build projects for the chips in the Zynq 7000 SoC series family. Below are some instructions on how to set up the tools. The versions listed are the ones that are primarily used, but other versions may work as well. If you use other versions of tools, you may need to add configuration files for them to projects (PetaLinux, in particular, changes its configuration files meaningfully between versions) -- you can read more about this in the **Configuring PetaLinux** section of the `projects/` README.

I recommend using a VM for these tools, as the installation can be large and messy, and the tools' supported OSes are slightly limited. For the recommended versions listed below, I used a VM running [Ubuntu 20.04.6 (Desktop image)](https://www.releases.ubuntu.com/focal/) with 200 GB of storage(/disk space), 16 GB of RAM(/memory), and 8 CPU cores. If you're running on a Mac with a M1/M2 or other non-x86 chip, you may need to be picky with your VM to do this ([UTM](https://mac.getutm.app/) seems to be the recommended option. Make sure to select "iso image" when selecting the downloaded Ubuntu ISO). In terms of installing Ubuntu on the VM, I recommend a "Minimal Install" and not to "Download Updates" to keep it as simple and close to the original, supported edition as possible. 

My process is explained below, but is definitely not the only way to do this.

- PetaLinux (2024.2)
- Vivado (2024.2)

These can be installed together from the AMD unified installer (2024.2 version [here](https://www.xilinx.com/support/download/index.html/content/xilinx/en/downloadNav/vivado-design-tools/2024-2.html) -- select "AMD Unified Installer for FPGAs & Adaptive SoCs 2024.2: Linux Self Extracting Web Installer", do this and everything else on the system you want these tools installed to, which is recommended to be a VM). You will need to create an AMD account to download and use the installer, but it should be free to do so. 


### Unified installer

Follow the documentation [here](https://docs.amd.com/r/en-US/ug1144-petalinux-tools-reference-guide/Installation-Steps) -- make sure the dropdown version at the top of the documentation matches the version you're using. 

You should make sure the system had the required libraries. From the stock Ubuntu 20.04.6 install, I needed to install the following packages with `sudo apt install` to make PetaLinux and Vivado install successfully:
```shell
sudo apt install gcc xterm autoconf libtool texinfo zlib1g-dev gcc-multilib build-essential libncurses5-dev libtinfo5
```

To run the unified installer, you will likely need to make it executable first. This is done by running (from the folder containing the installer, which will likely be your Downloads folder):
```shell
chmod +x FPGAs_AdaptiveSoCs_Unified_2024.2_1113_1001_Lin64.bin
```

From there, you can run the installer (this will need `sudo` permissions to write to the recommended default installation directory, which is `/tools/Xilinx/`).
```shell
sudo ./FPGAs_AdaptiveSoCs_Unified_2024.2_1113_1001_Lin64.bin
```

This will open a GUI installer. You will need to log in with your AMD account again. This will take you to the "**Select Product to Install**" page. You will need to run this installer twice, once for each of the two tools (PetaLinux and Vivado). I recommend starting with PetaLinux, as it is smaller and quicker to install.

### Installing PetaLinux

On the "**Select Product to Install**" page, scroll to the bottom and select "**PetaLinux**" and click Next. This repo is primarily focused on the Zynq7000 series SoCs, so you can select "**PetaLinux arm**" under "**Select Edition to Install**" and click Next. Accept the License Agreements and click Next. You can leave everything as default under "**Select Destination Directory**" (the default will be `/tools/Xilinx/` and will create a `PetaLinux/2024.2` directory). Click Next and then Install.


### Installing Vivado

Running the unified installer again, back on the "**Select Product to Install**" page, "**Vivado**" should be the second option. Select it and click Next. Under "**Select Edition to Install**", select "**Vivado ML Standard**" and click Next. The next section, "**Vivado ML Standard**", allows you to trim the installation size to only the components needed. First, I recommend unchecking everything you can. You can then check the following options:
- **DocNav** (optional) for looking at documentation in the Vivado GUI. Documentation can also be found online.
- Under **Devices** -> **Production Devices** -> **SoCs** check **Zynq-7000** (you may need to expand the sections to see this. It's fine that it says "limited support").

Click Next. Accept the Licence Agreements and click Next. Just like with PetaLinux, you can leave everything as default under "**Select Destination Directory**" (the default will be `/tools/Xilinx/` and will create a `Vivado/2024.2` directory). Click Next and then Install.

## Changing your default shell to bash

PetaLinux requires the default shell to be bash. In Ubuntu 20.04.6 (and most other Ubuntu versions), the default shell is Dash. The default shell is the `/bin/sh` file, which is a symlink to the binary of another shell. You can check your current default shell by running:
```shell
ls -l /bin/sh
```
This will output something like:
```
lrwxrwxrwx 1 root root 4 Mar 31  2024 /bin/sh -> dash
```
This means that the default shell is currently Dash. 

To change it to bash, you can run:
```shell
sudo ln -sf /bin/bash /bin/sh
```
which overrides the symlink to point to bash instead of Dash. 

Checking again with `ls -l /bin/sh` should now output:
```
lrwxrwxrwx 1 root root 9 Apr 29 01:13 /bin/sh -> /bin/bash
```

## Profile setup

With this repository cloned into your VM (e.g. `/home/username/zynq_toolbox or something similar), you will need to set up some environment variables and modify the Vivado init script to use this repo's scripts. At the top level of this repository, you will find a file named
```
environment.sh.example
```
This is a template file for the environment variables that you need to set up. Copy this file and name the copy 
```
environment.sh
```
This file will be used by the repo, but is not tracked by git, so you can modify it without worrying about it being overwritten by a `git pull` or similar command.

You will need to edit the following variables in this file to match your setup:
- `ZYNQ_TOOLBOX`: The path to the repository root directory (e.g. `/home/username/zynq_toolbox`, as above)
- `PETALINUX_PATH`: The path to the PetaLinux installation directory (by default, this will be `/tools/Xilinx/PetaLinux/2024.2/tool`)
- `PETALINUX_VERSION`: The version of PetaLinux you are using (e.g. `2024.2`)
- `VIVADO_PATH`: The path to the Vivado installation directory (by default, this will be `/tools/Xilinx/Vivado/2024.2`)

The remaining lines are optional or do not need to be changed:
- `source $VIVADO_PATH/settings64.sh`: This line sources a Vivado script that sets up the terminal environment for Vivado. This should be left as is.
- `PETALINUX_DOWNLOADS_PATH`/`PETALINUX_SSTATE_PATH`: These are optional variables only needed if you want to do offline builds with PetaLinux. See the [Optional: PetaLinux offline build setup](#optional-building-petalinux-offline) section below for more information.

With `environment.sh` set up, you will need to source it in your shell. Add the following line to one of the files that is sourced in new bash terminals, where \[path_to_zynq_toolbox\] is the path to the root of this repository (e.g. `/home/username/zynq_toolbox`):
```bash
source [path_to_zynq_toolbox]/environment.sh
```

### Which bash file to add the source line to?

Feel free to skip this section if you already know how to set up your bash profile files.

bash has a number of profile files: `~/.bashrc`, `~/.bash_profile`, and `~/.profile` are common, as well as the lesser-used `/etc/profile` and `~/.bash_login`. Here's a short summary of [what each of those files does](https://www.baeldung.com/linux/bashrc-vs-bash-profile-vs-profile).

My preferred way is to only have `~/.profile` and `~/.bashrc`. Here's how that works:

When logging in with a terminal, you're opening an *interactive, login* shell. In this case, bash will try to source the first of whichever is present, in order: `~/.bash_profile`, then `~/.bash_login`, then `~/.profile`. `~/.profile` is also used by some other shells like Dash.

When opening a new terminal window, you're opening an *interactive, non-login* shell. In this case, bash will source `~/.bash_rc`.

`~/.profile` may contain lines sourcing `~/.bashrc`, which I would like to make sure only runs in interactive shells. To do this, I just replace any line that does so:
```bash
. "$HOME/.bashrc"
```
with:
```bash
if [[ $- == *i* ]]; then . "$HOME/.bashrc"; fi
```
which checks if the `$-` variable (containing single letter flags about the terminal status) contains `i` (interactive).

All of this gives me the following `~/.profile`, in its entirety:

```bash ~/.profile
# ~/.profile: executed by the command interpreter for login shells.
# This file is not read by bash(1), if ~/.bash_profile or ~/.bash_login
# exists.
# see /usr/share/doc/bash/examples/startup-files for examples.
# the files are located in the bash-doc package.

# the default umask is set in /etc/profile; for setting the umask
# for ssh logins, install and configure the libpam-umask package.
#umask 022

# if running bash
if [ -n "$bash_VERSION" ]; then
    # include .bashrc if it exists
    if [ -f "$HOME/.bashrc" ]; then
        if [[ $- == *i* ]]; then . "$HOME/.bashrc"; fi
    fi
fi

# set PATH so it includes user's private bin if it exists
if [ -d "$HOME/bin" ] ; then
    PATH="$HOME/bin:$PATH"
fi

# set PATH so it includes user's private bin if it exists
if [ -d "$HOME/.local/bin" ] ; then
    PATH="$HOME/.local/bin:$PATH"
fi

# Source the Rev D environment script
source $HOME/zynq_toolbox/environment.sh
```

However, `~/.profile` is only sourced if `~/.bash_profile` and `~/.bash_login` don't exist. If `~/.bash_profile` already exists, you can delete it. If it needs to exist, just source `~/.profile` in it.

```bash
#~/.bash_profile
# Just source .profile
. "$HOME/.profile"
```
You can do similarly with `~/.bash_login` if it exists, but I've never really seen it used.

## Vivado init script

The final required step is to pass some of the environment variables to Vivado. This is done by modifying the Vivado init script to source the `scripts/vivado/repo_paths.tcl` script from this repository. The Vivado init script can be located in one of three places. Vivado checks each in order (each overriding the last one):

- Install directory (`/tools/Xilinx/Vivado/<version>/Vivado_init.tcl` by default)
- Particular Vivado version (`~/.Xilinx/Vivado/<version>/Vivado_init.tc`)
- Overall Vivado (`~/.Xilinx/Vivado/Vivado_init.tcl`)

For personal use, I'd set it in the last location to override everything else. You can go to that location and create the `Vivado_init.tcl` file if it doesn't exist, or edit it if it does. It should look like:
```tcl
# Example ~/.Xilinx/Vivado/Vivado_init.tcl:
# Set up Rev D configuration
set zynq_toolbox $::env(ZYNQ_TOOLBOX)
source $zynq_toolbox/scripts/vivado/repo_paths.tcl
```

## Optional: Makefile variable defaults

The Makefile is set up to use the variables `PROJECT`, `BOARD`, and `BOARD_VER` to determine which project, board, and board version to build, as well as a couple others (see [Building an SD card](#building-an-sd-card), [Building a different board, board version, or project](#building-a-different-board-board-version-or-project), [Building PetaLinux offline](#building-petalinux-offline) and [Script targets](#script-targets) for more information). To set personal default values for these variables, you should copy
```
make_defaults.mk.example
```
into the file
```
make_defaults.mk
```
and make your edits there. Just like `environment.sh`, only the example file is tracked, so you can edit the `make_defaults.mk` file without worrying about it being overwritten by a `git pull` or similar command.

## Optional: PetaLinux offline build setup

The PetaLinux build process requires downloading a lot of files from the internet, which can be slow and unreliable. Depending on your network connection, this could add upwards of ten minutes to the build time. If you want to put in a bit of effort for a more reliable build process, you can download these files and store them locally, so that PetaLinux can use them without needing to download them each time.

For PetaLinux 2024.2, you can download the files from the [AMD download center](https://www.xilinx.com/support/download/index.html/content/xilinx/en/downloadNav/embedded-design-tools/2024-2.html). The files you will need are all the way at the bottom, under **PetaLinux Tools sstate-cache Artifacts** (you can ignore the final section, Update 1). You'll need to download two files:
- `arm sstate-cache` (TAR/GZIP - ~9 GB) 
- `Downloads` (TAR/GZIP - ~59 GB)

You'll want to extract these files to some directory on your system. From there, you can set the `PETALINUX_DOWNLOADS_PATH` and `PETALINUX_SSTATE_PATH` environment variables in your `environment.sh` file to point to the directories where you extracted these files. Each archive has a simply named directory at the top, `downloads` and `arm`. For example, if you have the following directory structure:
```
~/petalinux_downloads
├── downloads_2024.2_11061705
│   └── downloads
└── sstate-cache_2024.2_11061705
    └── arm
```
You would set the environment variables in `environment.sh` as follows:
```bash
export PETALINUX_DOWNLOADS_PATH="$HOME/petalinux_downloads/downloads_2024.2_11061705/downloads"
```
and
```bash
export PETALINUX_SSTATE_PATH="$HOME/petalinux_downloads/sstate_arm_2024.2_11061705/arm"
```

With these variables set, you can include `OFFLINE=true` in the `make` when [Building an SD card](#building-an-sd-card).

## Optional: Running tests

You can optionally run tests for individual Verilog cores or all the custom cores used for a project using [cocotb](https://www.cocotb.org/). cocotb is a Python tool that allows you to write tests for your Verilog cores in Python, which can be run in a simulator (we use [Verilator](https://www.veripool.org/verilator/) here).

### Installing cocotb

- Use `apt` to install `cocotb` and `libpython3-dev`
```shell
sudo apt install python3-cocotb libpython3-dev
```
- Use `pip` to install `cocotb` and `cocotb_coverage`
```shell
pip install cocotb cocotb_coverage
```
- You may see a warning that `~/.local/bin` is not in your `$PATH` variable. If you're using my `~/.profile` setup above, you should see that it DOES add `~/.local/bin` to your `$PATH`, but only if the folder exists. If this is your first installation with `pip` (which is likely on a new system), the folder was only just created. If you restart your shell or system, you should see those files added to your path. You can check with
```shell
which cocotb-config
```
which should output something like:
```
/home/username/.local/bin/cocotb-config
```

### Installing Verilator

You SHOULD be able to install Verilator using `apt`, but last I checked, the version in the Ubuntu 20.04.6 repositories was too old (`4.028` but cocotb requires `4.106` -- the most recent is `5.036` as of writing, so Ubuntu is REALLY out of date).

Instead, you may need to build Verilator from source. Here's the [Verilator installation instructions](https://verilator.org/guide/latest/install.html) for that method.

Here's a slightly edited rewrite of that walkthrough, for clarity. Run these lines one-by-one in a terminal. The `git clone` line will of course clone the Verilator repo wherever you run it, so make sure you're in a directory where you want the Verilator source code to be stored (i.e. NOT in this repository's root directory).

```bash
# Prerequisites:
sudo apt install git help2man perl python3 make autoconf g++ flex bison ccache
sudo apt-get install libgoogle-perftools-dev numactl perl-doc
sudo apt-get install libfl2  # Ubuntu only (ignore if gives error)
sudo apt-get install libfl-dev  # Ubuntu only (ignore if gives error)
sudo apt-get install zlibc zlib1g zlib1g-dev  # Ubuntu only (ignore if gives error)
```
```bash
git clone https://github.com/verilator/verilator   # Only first time
```
```bash
# Every time you need to build:
unset VERILATOR_ROOT  # For bash
cd verilator
git pull         # Make sure git repository is up-to-date
```
You can then choose your Verilator version by checking out a specific commit by it's tagged version number. Some commands you can run are:
```bash
git checkout master      # Use development branch (e.g. recent bug fixes)
```
```bash
git checkout stable      # Use most recent stable release
```
```bash
git tag          # See what versions exist. Arrow keys to scroll, 'q' to quit
```
```bash
git checkout v{version}  # Switch to specified release version that's listed in the git tags, like v5.036 or v4.106
```

Finally, to build, run the following commands from inside the `verilator` directory (the one you just cloned):
```bash
autoconf         # Create ./configure script
./configure      # Configure and create Makefile
make -j `nproc`  # Build Verilator itself (if error, try just 'make')
sudo make install
```

# Building an SD card

This section will walk you through the build process of a fully formed bootable Micro SD card for the Snickerdoodle Black containing the Rev D Shim firmware, Linux operating system, and FPGA bitstream. If you want to understand the steps in more detail, go through the [Example projects](#example-projects) section, which progressively build up the components and techniques used for the Rev D Shim firmware.

The entire build process is scripted by the `Makefile` and various shell and Tcl scripts in the `scripts/` directory. The main entry point is the `Makefile`, which will call the appropriate scripts to build the project. The default target and variables for the `Makefile` is the Rev D Shim firmware for the Snickerdoodle Black. As such, you can simply run the following command from the root of this repository to build the SD card:
```bash
make
```

This will output two compressed files in the `out/snickerdoodle_black/1.0/zynq_toolbox/` directory:
- `BOOT.tar.gz`: The compressed boot partition, which contains the Linux kernel, device tree, and boot scripts.
- `rootfs.tar.gz`: The compressed root filesystem, which contains all of the Linux files.

To load these files onto the Micro SD card, you'll first need one with the proper partitioning scheme. This follows the instructions given in the [PetaLinux Tools Documentation (UG1144)](https://docs.amd.com/r/en-US/ug1144-petalinux-tools-reference-guide/Preparing-the-SD-Card). You can use any disk partitioning tool to do this, but Linux ones are generally better/easier to use. Writing the files to the SD card is already easiest if your VM has access to your SD card reader / USB port, so I'd recommend doing the partitioning through your VM as well -- GParted is a good graphical tool for this, which you can install with:
```bash
sudo apt install gparted
```
if it isn't already.

Use GParted to partition the SD card as follows:
- One partition of type `fat32` with a size of 1 GiB, labeled `BOOT`. Make sure this has 4MiB of unallocated free space before it.
- One partition of type `ext4` with a size of whatever is left on the SD card, labeled `RootFS`.

Once the SD card is partitioned, you can uncompress the `BOOT.tar.gz` and `rootfs.tar.gz` files into the respective partitions. If you're using the same Ubuntu system on a VM that I was, and are using the default `BOARD`, `BOARD_VER`, and `PROJECT` (`snickerdoodle_black`, `1.0`, `zynq_toolbox`) you can do this with the following [target](#script-targets) (may need to eject and re-insert the SD card after partitioning):
```bash
make write_sd 
```
which will attempt to find the SD card partitions automatically at `/media/username/BOOT` and `/media/username/RootFS`, where `username` is your username on the system. If the partitions are mounted somewhere else, you can specify the mount folder as an additional argument:
```bash
make write_sd MOUNT_DIR=[mountpoint]
```
where `[mountpoint]` is the folder containing the mounted `BOOT` and `RootFS` directories. As with any `make` targets, you can modify the `BOARD`, `BOARD_VER`, and `PROJECT` variables to build for a different board, board version, or project (see the [Building a different board, board version, or project](#building-a-different-board-board-version-or-project) section below).

You can similarly clean the SD card files from the mounted SD card with:
```bash
make clean_sd
```
again with an optional `MOUNT_DIR` argument to specify the mount point of the SD card partitions.

If you have some other mounting scheme, you'll need to manually uncompress the files into the appropriate partitions with `tar`:
```bash
tar -xzf out/snickerdoodle_black/1.0/BOOT.tar.gz -C [BOOT_mountpoint]
tar -xzf out/snickerdoodle_black/1.0/rootfs.tar.gz -C [RootFS_mountpoint]
```

If your board isn't the Snickerdoodle Black, or you want to modify the project or build your own, you should read the [Example projects](#example-projects) section to get a sense of how everything works.

## Building a different board, board version, or project

The Makefile is set up to read variables for `BOARD`, `BOARD_VER`, and `PROJECT` from the command line. These can be used to build with a different board, board version, or project. For example, to build the `shim_controller_v0` project for version `1.0` of the Red Pitaya `sdrlab_122_16` board, you can run:
```bash
make BOARD=sdrlab_122_16 BOARD_VER=1.0 PROJECT=shim_controller_v0
```

Boards and board versions are defined in the `boards/` directory, where the board files for a given board are given under `boards/[BOARD]/board_files/[BOARD_VER]/`. Projects are defined based on folders in the `projects/` directory, where each project has its own folder. Note that projects need to be configured to work with a specific board and board version -- this is done under `projects/[PROJECT]/cfg/[BOARD]/[BOARD_VER]/`, and requires configuration files for `petalinux` and the Vivado Xilinx Design Constraint `xdc` files. You can read more about the requirements for this configuration in the `projects/` directory's README file.

## Building PetaLinux offline

If you set up [Optional: PetaLinux offline build setup](#optional-building-petalinux-offline) above, you can include the `OFFLINE=true` variable in the `make` command to use the local files instead of downloading them. For example, to build the Rev D Shim firmware for the Snickerdoodle Black with offline PetaLinux, you can run:
```bash
make OFFLINE=true
```

## Intermediate build files and targets

The Makefile is set up to build the project in a series of steps, with intermediate files stored in the `tmp/` directory. These targets can also be made individually, if you want to debug the build or explore the intermediate files. There are also some targets for cleaning or running tests. `Makefile` has a lot of comments, so feel free to check that out as well. The main targets are:

### Default target
- `all`: The default target, which will be run if no target is provided. This will build `sd` and `tests`.

### Script targets
- `tests`: Run tests for all cores in the project. Test summaries per core will be placed in `custom_cores/[vendor]/cores/[core]/tests/test_status`, and a summary of all core tests for the project will be placed in `projects/[project]/tests/core_tests_summary`.
- `write_sd`: Write the SD card files to the SD card. The default mount point is `/media/[username]/`, but can be overridden with the `MOUNT_DIR` variable. This will write the `BOOT.tar.gz` and `rootfs.tar.gz` files to the appropriate partitions on the SD card, as described in the [Building an SD card](#building-an-sd-card) section. Uses the `scripts/make/write_sd.sh` script.
- `petalinux_cfg`: Generate or update the PetaLinux system configuration files for the project, under `projects/[project]/cfg/[board]/[board_ver]/petalinux/[petalinux_ver]/`. Uses the `scripts/petalinux/petalinux_cfg.sh` script. Requires the terminal to be above a certain size to display the PetaLinux config GUI.
- `petalinux_rootfs_cfg`: Generate or update the PetaLinux root filesystem configuration files for the project, under `projects/[project]/cfg/[board]/[board_ver]/petalinux/[petalinux_ver]/`. Uses the `scripts/petalinux/petalinux_rootfs_cfg.sh` script. Requires the terminal to be above a certain size to display the PetaLinux config GUI. 
- `clean_sd`: Clean the SD card files from a mounted SD card. The default mount point is `/media/[username]/`, but can be overridden with the `MOUNT_DIR` variable. Uses the `scripts/make/clean_sd.sh` script. 
- `clean_project`: Remove a single project's intermediate and temporary files, including Vivado-packaged cores from `tmp/`
- `clean_build`: Remove all the intermediate and temporary files, including Vivado-packaged cores from `tmp/`, as well as reports in `tmp_reports`.
- `clean_tests`: Remove the `results` directory from all core test folders (under `custom_cores/[vendor]/cores/[core]/tests/`), but leave the `test_status` file.
- `clean_test_results`: Remove the `test_status` file from all core test folders, as well as the `core_tests_summary` file from all project test folders (under `projects/[project]/tests/`). Runs `clean_tests` first.
- `clean_all`: Run all the clean targets above and additionally remove any output files in `out/`.

### Main build targets
- `sd`: Build the full SD card files for the project (`rootfs` and `boot`), which will be placed in `out/[board]/[board_ver]/[project]/`.
- `bit` Build a standalone bitstream for the project, which will be placed in `out/[board]/[board_ver]/[project]/`. This can be used if you're using a different workflow than PetaLinux and just want the bitstream file to load the FPGA configuration.
- `rootfs`: Build the root filesystem half of the `sd` files
- `boot`: Build the boot partition half of the `sd` files

### Intermediate build targets
- `cores`: Build all the Vivado-packaged custom cores for the project, which will be placed in `tmp/cores/[vendor]/[core]/`. This is the first step of the overall `sd` target.
- `xpr`: Build the Vivado project file for the project, which will be placed at `tmp/[board]/[board_ver]/[project]/project.xpr`. This step requires `cores` and is part of the overall `sd` target.
- `xsa`: Build the Vivado Xilinx hardware definition `XSA` file for the project, which will be placed at `tmp/[board]/[board_ver]/[project]/hw_def.xsa`. This step requires `xpr` and is part of the overall `sd` target.
- `petalinux`: Build the PetaLinux project for the project, which will be the folder `tmp/[board]/[board_ver]/[project]/petalinux/`. This step requires `xsa` and is part of the overall `sd` target.

There are other specific targets in the Makefile, but I don't recommend using them directly unless you know what you're doing. 

## Using the Rev D Shim Amplifier

Once you've built the Rev D Shim firmware, please refer to the README in the `projects/rev_d_shim/` directory for instructions on how to use it.

# Example projects

To understand the build processes in this repo, it's recommended to explore the example projects in the `projects/` directory, as well as the README in the `projects/` directory itself. As a quick summary, each project has its own folder, and the example projects are prefixed with `ex##_`, where `##` is the example number. They're ordered to progressively build up the scripting and configuration concepts I personally needed to learn to build the Rev D Shim firmware, so they should be a good starting point for understanding how to build your own projects. 

You should read through the README in each of their respective folders, but in brief, the example projects are:

## EX01 -- Basics

This example project is mostly a template for the minimum viable project. It will walk you through the basic steps that any project will use, explaining the fundamental Vivado and PetaLinux build steps, including how to incorporate basic software or files in the built SD card. This is necessary to build the Rev D Shim PS and PL components.

## EX02 -- AXI interface

This example project explores more of the Vivado Tcl scripting capabilities and demonstrates the basic AXI interface, which will be how the Zynq's CPU / processing system (PS) communicates with the FPGA / programmable logic (PL). It includes some playground software to try out various AXI interfaces. This is necessary for the Rev D Shim firmware to actually control the hardware, as it needs to communicate with the FPGA to set the shim channels and read the buffer data (among other things).

## EX03 -- UART

This example project demonstrates some configuration options for the PS's interfaces, including its UART interface. It's a good overview of how to connect the Zynq's PS to an external computer via a UART interface. This is necessary for the Rev D Shim firmware to communicate with an external host computer outside of the scanner room.

## EX04 -- Interrupts

This example project covers interrupts from the PL to the PS and software to handle that, allowing the PL to signal the PS when it needs attention. This is necessary for the safety features of the Rev D Shim firmware.

## EX05 -- DMA

This example project covers the Direct Memory Access (DMA) interface, which allows the PS to transfer data to and from the PL through the off-chip DDR memory.

# Testing

Testing is done using [cocotb](https://www.cocotb.org/), a Python-based testbench framework for digital design verification. It allows you to write tests in Python and run them in a simulator, such as Verilator. To install the tools needed for testing, please refer to the [Optional: Running tests](#optional-running-tests) section above.

To run tests for a specific core, you can use the `test_core.sh` script in the `scripts/make/` directory with the `vendor` and `core` arguments. For example, to test the `fifo_sync` core from `base`, you can run:
```bash
./scripts/make/test_core.sh base fifo_sync
```
This will run the tests for the `fifo_sync` core under `custom_cores/base/cores/fifo_sync/tests/src` and output a test status report at `custom_cores/base/cores/fifo_sync/tests/test_status`.

To run tests for all cores in a project, you can use the make target `tests`. For example, to run tests for the `rev_d_shim` project, you can run:
```bash
make tests PROJECT=rev_d_shim
```
