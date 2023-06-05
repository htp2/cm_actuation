#include <memory>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <cm_actuation/BIGSSMaxonCANROS.hpp>


BIGSSMaxonCANROS::BIGSSMaxonCANROS(const ros::NodeHandle ros_nh, const std::string& can_device_name, const std::string& supported_actuator_name, double pub_hz)
: m_rosNodeHandle(ros_nh)
{
    m_measured_js_pub = m_rosNodeHandle.advertise<sensor_msgs::JointState>("measured_js", 1);
    m_measured_jv_pub = m_rosNodeHandle.advertise<sensor_msgs::JointState>("measured_jv", 1);
    m_servo_jp_sub = m_rosNodeHandle.subscribe("servo_jp", 1, &BIGSSMaxonCANROS::servo_jp_cb, this);
    m_servo_jv_sub = m_rosNodeHandle.subscribe("servo_jv", 1, &BIGSSMaxonCANROS::servo_jv_cb, this);

    // create timer to publish measured_js and measured_jv}
    m_pub_timer = m_rosNodeHandle.createTimer(ros::Duration(1.0/pub_hz), &BIGSSMaxonCANROS::pub_timer_cb, this);

    m_maxon_can = std::make_unique<BIGSSMaxonCAN>(can_device_name, supported_actuator_name);

    //temp hardset TODO: make homing functions, etc.
    m_maxon_can->enable_PDO();
    m_maxon_can->set_enable_state();
}

// note the BIGSSMaxonCAN has a constructor that allows you to directly specify your own cobid map.
// An equivalent constructor for the ROS wrapper could be easily added if needed

void BIGSSMaxonCANROS::servo_jp_cb(const sensor_msgs::JointState& msg)
{
    m_maxon_can->set_operation_mode(BIGSSMaxonCAN::SupportedOperatingModes::PPM);
    m_maxon_can->PPM_command(msg.position[0]);
}

void BIGSSMaxonCANROS::servo_jv_cb(const sensor_msgs::JointState& msg)
{
    m_maxon_can->set_operation_mode(BIGSSMaxonCAN::SupportedOperatingModes::CSV);
    m_maxon_can->CSV_command(msg.velocity[0]);
}

void BIGSSMaxonCANROS::pub_timer_cb(const ros::TimerEvent& event)
{
    // publish measured_js and measured_jv
    sensor_msgs::JointState js_msg;
    js_msg.header.stamp = ros::Time::now();
    js_msg.position.push_back(measured_js);
    m_measured_js_pub.publish(js_msg);

    sensor_msgs::JointState jv_msg;
    jv_msg.header.stamp = ros::Time::now();
    jv_msg.velocity.push_back(measured_jv);
    m_measured_jv_pub.publish(jv_msg);
}
