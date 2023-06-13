# cm_actuation

This repository contains the code related to the "handheld" actuation unit for the continuum manipulator. This includes the accompanying roll actuator that attaches to a base robot so the actuation can be used in a cooperative or fully-robotic capacity.

The repository also contains the code / files / etc needed to simulate the actuation unit in AMBF.

A core component of the repository is code that allows the actuation unit to be controlled via a CAN bus. This utilizes the canopen interface, and borrows some code and functionality from Dr. Simon Leonard's work, though specified for use on the specific hardware used here. A ROS wrapper is also provided for this functionality. Please feel free to use/change/add as needed to make this part of the code work for whatever Maxon motor setup you wish to use. Note that, while fairly general and built with trying to help future users be able to use directly or only with light modification to match their needs, this code was made for a specific purpose, and you should make sure to keep that in mind before you use it directly for another application. TLDR; I hope you find this useful, but please use at your own risk.

## Building
Currently, this repository is only being used with a project based on ROS(1), so the assumption is that you will build it within a catkin_ws. Place the repository in the src folder of your catkin_ws, and then run catkin build.

## BIGSSMaxonCAN
A CAN interface for Maxon motors implemented to achieve basic functionality for applications in the BIGSS lab at JHU. This includes serial-based communication for position / velocity commands as well as reading values such as position, velocity, current, and torque from the motor. The implementation is divided into two levels:
- BIGSSMaxonCAN
This is where all of the motor interfacing happens over CAN. It uses the CANopen framework and achieves communication directly with the default Linux drivers. 
- BIGSSMaxonCANROS
A ROS (for now just ROS1, though it should be fairly easy for someone to make an equivalent BIGSSMaxonCANROS2) wrapper for BIGSSMaxonCAN that exposes most of the functionality to ROS using standard messages / services. Where appropriate commands keep as closely to the [CRTK convention](https://github.com/collaborative-robotics/documentation/wiki/Robot-API).

An example application that creates a ros node is in ```bigss_maxon_can_ros_node.cpp``` and additionally a simple Python based PyQT5 GUI that utilizes the ROS topics and services is provided in ```roll_act_simple_ros_ui.py```. You can also (of course) send these through the terminal, or through a separate script (e.g. ```example_sinusoidal_vel_script.py```). Due to the CRTK compatability, it is also very likely you could directly use the CRTK [python client](https://github.com/collaborative-robotics/crtk_python_client) or [matlab client](https://github.com/collaborative-robotics/crtk_matlab_client), though I have not tested this myself.

### Description of functionality
BIGSSMaxonCAN is a class that creates several CAN nodes which will read / write to the maxon motor contoller on a CAN bus. There is no "loop" at this level. The assumption is that you will call the commands from your own (likely multi-threaded if you want "real-time"-like results) application, such as is done in the bigss_maxon_can_ros_node via BIGSSMaxonCANROS.

Implementation for the following features is included: Profile position mode, profile velocity mode, homing, enable, disable, halt, reading position, velocity, current, torque, operation mode, and parts of the status word including is_homed, is_enabled.
Note: reading commands requires the PDO outputs to be setup with the correct payloads. There are several helpful functions implemented within BIGSSMaxonCAN that you could take advantage of if you want to add and/or change payloads for your specific requirements.

TODO[Justin]: You could put those payloads here?

The following features are "mostly" implemented, but were not needed for the project, so left undone with ```FUTURE``` tags providing guidance: Cyclic Synchronous Position, Velocity, Torque modes. These are modes where you are expected to send regular, high-frequency commands (e.g. you generate your own low-level trajectories etc, and to use them correctly you require some firmware changes (e.g. setting the interpolation time on the maxon motor controller to match your command rates, etc.)

### Specifying motor info
Obviously, you cannot write code that will drive any motor without knowing some information about it. We consider some items to be "fixed" here (see payloads), and then allow for invidual motors properties to be specified on construction of BIGSSMaxonCAN. This can be done two ways:
1. Preferred: use of the "supported_actuator" constructor. Those actuators which are known and supported will have these values hardcoded in this constructor, and you merely call the constructor with a string input and it will set all of those values based on that string. If you have a new actuator you want to support, you can simply add it as another supported_actuator in that constructor.
2. Alternate: If you don't want to change the source code, or just want to quickly test a new actuator, there is an alternate constructor where you can specify all of these properties directly.

- m_node_id: This is usually set with a dip switch on the maxon controller
- m_cobid_map: map of strings to the Communication Object Identifier (aka COBID). Examples are "enable_state", "pvm_target", etc. This is used for directing commands to the right place
- m_needs_homing: boolean, would be false if, for example you have an absolute encoder
- m_homing_sequence: vector of the low-level CANframe data to send (looking at example is probably easiest) for homing. Different for each actuator. If m_needs_homing is false, this will be ignored (just make empty).
- m_encoder_to_rad: constant double that converts encoder counts to radians
- m_maxonvel_to_rad_per_sec: constant double that converts Maxon's "rpm" values to rad/sec (NOTE: this can be scaled by e.g. 0.1 for better resolution)
- m_motor_rated_torque_nm = constant double that reflects the "rated_torque" value set in the firmware for the motor. Maxon outputs torque values relative to this

## Helpful Notes
### Useful terminal commands
In order to start a CAN bus (i.e. make it available for read/write on a PC), you need to run the following command:
```ip link set can0 up type can bitrate 1000000```

There are a couple of useful can utilities for debugging

Continually print all messages on a CAN bus:
```candump can0```

Want to see candump but exclude a certain ID (e.g. 2A0)?... probably a way to use candump masks as well.
```candump can0 | grep -v '2A0'```

Send a message on a CAN bus:
```cansend can0 <id>#<msg>```

Evaluate the current load on the bus (how near capacity are you):
```canbusload can0@1000000``` where 1000000 is the bitrate of the bus

Check to see info on can bus (e.g. if it is up or down, etc):
```ip -details link show can0```

Turn off a CAN bus (e.g. for the old turn it off and back on trick):
```ip link set can0 down```
