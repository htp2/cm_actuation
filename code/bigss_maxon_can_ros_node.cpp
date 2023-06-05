#include <iostream>
#include <memory>
#include <ros/ros.h>
#include <cm_actuation/BIGSSMaxonCANROS.hpp>

int main(int argc, char **argv)
{
    // ros init
    ros::init(argc, argv, "bigss_maxon_can_ros_node");
    
    // make ros node handle
    ros::NodeHandle ros_nh("~");

    // get parameters
    std::string can_device_name = "can0";
    std::string supported_actuator_name = "roll_actuator";
    double pub_hz = 100.0;

    // create BIGSSMaxonCANROS object
    auto bigss_maxon_can_ros = std::make_unique<BIGSSMaxonCANROS>(ros_nh, can_device_name, supported_actuator_name, pub_hz);


    ros::AsyncSpinner spinner(0);
    spinner.start();
    ros::waitForShutdown();
    // ros spin
    // ros::spin();
}

