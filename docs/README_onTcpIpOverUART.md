The purpose of this README is to turn a UART (i.e., serial) connection into a TCP/IP connection. From a layman point of view, we could call it making a serial link an Ethernet link.

Although there are supposed to b multiple solutions. I could only get working the PPP (i.e., Point to Point Protocol) solution. In other words, the solution is running PPP over the serial link. The TCP/IP stack can sit on PPP. Here is what we need to do:

- Install PPP on the Host OS
- Build Petalinux with PPP
- Run pppd server on Petalinux at boot time by putting it in /etc/init.d/mystartup
- Also make sure the contents of /etc/ppp/options is right on Petalinux
- Run "pppd call <IPaddr>" on the Host OS
- Make sure we have a file at path /etc/ppp/peers/IPaddr with the right contents on the Host OS

Give more detail for the above and polish the bullets below:

- When is the link established
- What are the right contents of the mentioned files
- Also explain how file xfer is done with rsync and what does not work in the case of scp, rcp, sftp, ftp

fatih.ugurdag@ozyegin.edu.tr
