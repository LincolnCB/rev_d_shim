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

There is a Linux library called "lrzsz".

It needs to be in the rootfs_config when petalinux-build is
run. Package lrzsz contains two programs: "rz" and "sz".

Program rz receives a file from the host (h) and saves it on Petalinux
(p) file system, whereas sz sends a file from Petalinux to the host,
which saves it on the host file system. Think of it like this: rz=h2p
and sz=p2h.

Most mainstream terminal emulators over UART, such as minicom and
picocom, have the capability to talk to rz and sz (through Zmodem
protocol, optionally Xmodem or Ymodem).

I tried files as large as 100kB in both directions with success. At a
baud rate of 115.2kbps, we get a net file xfer rate of around
92kbps. Most FPGA Boards support baud rates up to 921.6kbps.

On picocom, for rz, we type rz at the Petalinux prompt, then Ctrl-a
Ctrl-s (s for send) and enter the file name when asked.

On picocom again, for sz, we type sz <name of file to send from
Petalinux>, then Ctrl-a Ctrl-r (r for receive). It asks for file name
again. Just press Enter. If you enter a file name at that point, sz
locks up for a while then terminates without success.
