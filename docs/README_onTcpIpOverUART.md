//--------------------------------------------------------------------
Author:
Prof. H. Fatih Ugurdag
Ozyegin Univ., Turkiye

Contact Info:
US ph +1 727-377-0182
fatih.ugurdag@ozyegin.edu.tr

Date:
July, 2025
//--------------------------------------------------------------------

The purpose of this README is to turn a UART (i.e., serial) connection
into a TCP/IP connection. From a layman point of view, we could call
it making a serial link an Ethernet link.

Although there are supposed to be multiple solutions. I could only get
working the Point to Point Protocol (PPP) solution. In other words,
the solution is running PPP over the serial link. The TCP/IP stack can
sit on PPP. Here is what we need to do:

- Install PPP on the Host OS

- Build Petalinux with PPP

- Run pppd server on Petalinux at boot time by putting it in
  /etc/init.d/mystartup

- Also make sure the contents of /etc/ppp/options on Petalinux are
  correct

- Run "pppd call <IPaddr>" on the Host OS

- Make sure we have a file at path /etc/ppp/peers/IPaddr with the
  right contents on the Host OS

After the above overview, let us give a detailed walk-through.

Host> picocom -b 115200 /dev/ttyUSBn

Obviously, picocom needs to be installed on the Host. 115200 is the
baud rate. The n in ttyUSBn could be 0, 1, or 2. The correct one can
be identified with ls -l. If not, whichever responds. The FPGA starts
with the First Stage Bootloader set to 115200. Then, it starts uboot,
which is also set to 115200 as a default. That can be changed using
uboot command setenv baudrate nnnnnn, which can be saved in a .env
file in the boot directory with saveenv command. Next boot uses
whatever .env has as settings. Uboot calls the Petalinux kernel, which
is, in my case, set to 921600 baud. Picocom in the meantime can always
be set to any baud rate using C-a C-b nnnnnn to match the FPGA's baud
rate at the moment. During the boot process, the uboot count-down may
get interrupted. In that case, manually run boot.scr.

Petalinux> more /etc/init.d/mystartup
pppd noauth &

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

- What are the right contents of the mentioned files

- Also explain how file xfer is done with rsync and what does not work
  in the case of scp, rcp, sftp, ftp

- Work in progress
