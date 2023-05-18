Getting CAN setup (see http://www.peak-system.com/fileadmin/media/linux/index.htm)
You will need to install this:
libpopt-dev

Install peak linux driver
https://www.peak-system.com/quick/PCAN-Linux-Driver

Download PCAN Peak driver for Linux
https://www.peak-system.com/quick/BasicLinux


To use their GUI you need several unlisted dependencies:
sudo apt install tix-dev tk-dev tk8.6-dev libxft-dev libfontconfig1-dev libfreetype6-dev libpng-dev


There is a GUI for debugging the CAN bus: PCAN-View
You can install this from the PCANBasic driver page, then run with pcanview

For now, we will set up the roll actuator with ID 3 and the handheld actuator with ID 1

To enable PDO we can send 01 00 to ID 0 with a 100ms cycle time
To disable PDO we can send 08 00 to ID 0 with a 100ms cycle time

203 is the ID the roll actuator
We need to do state transitions 2,3,4 in https://www.maxongroup.com/medias/sys_master/root/8834324856862/EPOS4-Firmware-Specification-En.pdf pg 2-13
To do this we have to send the "shut down" command, followed by the "switch on and enable" command
so this results in sending the following two in sequence

06 00
0f 00

To send velocity command we send 0.1 of rpm to 503
For example to send 10 rpm we send 0x 64 00 00 00 to 503
To send 0.1 rpm we send 0x 01 00 00 00 to 503

To disable the roll actuator we send 0x 00 00 to 203

