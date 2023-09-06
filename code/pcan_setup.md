

For now, we will set up the roll actuator with ID 3 and the handheld actuator with ID 1

To enable PDO we can send 01 00 to ID 0
To disable PDO we can send 80 00 to ID 0 

203 is the ID the roll actuator
We need to do state transitions 2,3,4 in https://www.maxongroup.com/medias/sys_master/root/8834324856862/EPOS4-Firmware-Specification-En.pdf pg 2-13
To do this we have to send the "shut down" command, followed by the "switch on and enable" command
so this results in sending the following two in sequence

For troubleshooting: To view all the messages on the bus, run the following command
candump can0