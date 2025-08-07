//--------------------------------------------------------------------
Author:
Prof. H. Fatih Ugurdag
Ozyegin Univ., Turkiye

Contact Info:
US ph +1 727-377-0182
fatih.ugurdag@ozyegin.edu.tr

Date:
August, 2025
//--------------------------------------------------------------------

The purpose of this README is to turn a UART (i.e., serial) connection
into a TCP/IP connection. From a layman point of view, we could call
it "making a serial link an Ethernet link".

Although there are supposed to be multiple solutions. I could only get
"the Point to Point Protocol (PPP) solution" working. In other words,
the solution is: running PPP over the serial link. The TCP/IP stack
can sit on PPP.

Once the necessary setup is done (done once), connecting your Linux
machine (Host) to the FPGA (running Petalinux) is very easy:

step 1. Run the PPPserver on Petalinux => Turn on or reboot the FPGA

step 2. Run the PPPclient on the Host => pppd call <IPaddrOfFPGA>

Initially, pppd call will print a lot of messages on the screen. Once
pppd stops printing messages on the Host, it means the TCP/IP link has
been established. At that point, your TCP/IP capable programs in C,
Python, or your favorite language can function. It is even such that
many of them can run concurrently. The direction of any TCP/IP stream
could be Host to FPGA or FPGA to Host. In addition, programs such as
scp, rcp, sftp, ftp, rsync, ssh, telnet, etc. can be run.

The SETUP

- Install PPP on the Host OS

- Build Petalinux with PPP

- Run pppd server on Petalinux at boot time by putting it in
  /etc/init.d/mystartup

- Also make sure the contents of /etc/ppp/options on Petalinux are
  correct

- Make sure we have a file at path /etc/ppp/peers/IPaddrOfFPGA with
  the right contents on the Host OS

After the above overview, let us give a detailed walk-through.

Since, we initially do not have a TCP/IP link established, we need to
run a simple serial terminal program (e.g., picocom, minicom, etc.) to
connect to the FPGA. By the way, the FPGA Board I have is a legacy
Zybo board with a Z10 chip.

Host> picocom -b 115200 /dev/ttyUSBn

Obviously, picocom needs to be installed on the Host. 115200 is the
baud rate. The n in ttyUSBn could be 0, 1, or 2. The correct one can
be identified with ls -l. If not, whichever responds. The FPGA starts
with the First Stage Bootloader set to 115200. Then, it starts uboot,
which is also set to 115200 as a default. That can be changed using
uboot command setenv baudrate nnnnnn, which can be saved in a .env
file in the boot directory with saveenv command. Next boot uses
whatever .env has as settings. Uboot calls the Petalinux kernel, which
is, in my case, set to 921600 baud because I use a UART running at
921600 baud on the FPGA. Picocom, in the meantime, can always be set
to any baud rate using C-a C-b nnnnnn to match the FPGA's baud rate at
the moment. During the boot process, the uboot count-down may get
interrupted. In that case, manually run boot.scr.

Petalinux> more /etc/init.d/mystartup
pppd noauth &

If you create the below file, its contents are appended to the pppd
command line above. 192.168.1.10:192.168.1.2 below indicates the
TCP/IP bridge established, where 192.168.1.10 is the IPaddrOfFPGA and
192.168.1.2 is the IPaddrOfHost.

Petalinux> more /etc/ppp/options
/dev/ttyPS0 921600
192.168.1.10:192.168.1.2
noauth        # Disable authentication (for testing)
# debug       # Optional: enable debugging
persist       # Keep connection alive
nocrtscts
# local
nodetach
asyncmap 0

Make sure the above files have the displayed contents. For that, you
will need to become root. Do "sudo su -". Next time you connect to the
FPGA, you will not have to do these. When the kernel boots, you will
start seeing gibberish and the login prompt in between. The gibberish
will stop when the pppd on the Host also runs and connects to the pppd
on Petalinux.

On the Host, make sure you have the following files with the contents
shown below:

Host> more /etc/ppp/peers/192.168.1.10
921600
192.168.1.2:192.168.1.10
noauth        # Disable authentication (for testing)
debug       # Optional: enable debugging
persist       # Keep connection alive
nocrtscts
# local
nodetach
asyncmap 0

If you are running rsync, rcp, telnet, there is probably no need to
install anything special on Petalinux. However, scp and sftp require
sshd. ftp requires ftp server up and running.

Note that scp and some other tools appear to be stalled soon after
they are run. Just ignore that and wait. They successfully complete
the transfer, and the average transfer rate happens to be such that
actually there was no stall.
