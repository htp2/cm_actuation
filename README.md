# cm_actuation

This repository contains the code related to the "handheld" actuation unit for the continuum manipulator. This includes the accompanying roll actuator that attaches to a base robot so the actuation can be used in a cooperative or fully-robotic capacity.

The repository also contains the code / files / etc needed to simulate the actuation unit in AMBF.

A core component of the repository is code that allows the actuation unit to be controlled via a CAN bus. This utilizes the canopen interface, and borrows some code and functionality from Dr. Simon Leonard's work, though specified for use on the specific hardware used here. A ROS wrapper is also provided for this functionality.

## Building
Currently, this repository is only being used with a project based on ROS(1), so the assumption is that you will build it within a catkin_ws. Place the repository in the src folder of your catkin_ws, and then run catkin build.

## Helpful Notes
### Useful terminal commands
In order to start a CAN bus (i.e. make it available for read/write on a PC), you need to run the following command:
```ip link set can0 up type can bitrate 1000000```

There are a couple of useful can utilities for debugging

Continually print all messages on a CAN bus:
```candump can0```

Send a message on a CAN bus:
```cansend can0 <id>#<msg>```

Evaluate the current load on the bus (how near capacity are you):
```canbusload can0@1000000``` where 1000000 is the bitrate of the bus

Check to see info on can bus (e.g. if it is up or down, etc):
```ip -details link show can0```

Turn off a CAN bus (e.g. for the old turn it off and back on trick):
```ip link set can0 down```

