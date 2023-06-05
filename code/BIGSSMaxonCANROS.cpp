#include <memory>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <cm_actuation/BIGSSMaxonCANROS.hpp>


BIGSSMaxonCANROS::BIGSSMaxonCANROS(const ros::NodeHandle ros_nh, const std::string& can_device_name, const std::string& supported_actuator_name, double pub_hz, LowlevelVelMode vel_mode, LowlevelPosMode pos_mode)
: m_rosNodeHandle(ros_nh), m_lowlevel_vel_mode(vel_mode), m_lowlevel_pos_mode(pos_mode)
{
    m_measured_js_pub = m_rosNodeHandle.advertise<sensor_msgs::JointState>("measured_js", 1);
    m_measured_jv_pub = m_rosNodeHandle.advertise<sensor_msgs::JointState>("measured_jv", 1);
    m_servo_jp_sub = m_rosNodeHandle.subscribe("servo_jp", 1, &BIGSSMaxonCANROS::servo_jp_cb, this);
    m_servo_jv_sub = m_rosNodeHandle.subscribe("servo_jv", 1, &BIGSSMaxonCANROS::servo_jv_cb, this);

    // create timer to publish measured_js and measured_jv}
    m_pub_timer = m_rosNodeHandle.createTimer(ros::Duration(1.0/pub_hz), &BIGSSMaxonCANROS::pub_timer_cb, this);

    //TODO: settable read / telemetry rate
    m_read_timer = m_rosNodeHandle.createTimer(ros::Duration(1.0/1000.0), &BIGSSMaxonCANROS::read_timer_cb, this);

    m_maxon_can = std::make_unique<BIGSSMaxonCAN>(can_device_name, supported_actuator_name);

    //temp hardset TODO: make homing functions, etc.
    m_maxon_can->enable_PDO();
    m_maxon_can->set_enable_state();
}

// note the BIGSSMaxonCAN has a constructor that allows you to directly specify your own cobid map.
// An equivalent constructor for the ROS wrapper could be easily added if needed

void BIGSSMaxonCANROS::servo_jp_cb(const sensor_msgs::JointState& msg)
{
    if(m_lowlevel_pos_mode == LowlevelPosMode::CSP)
    {
        m_maxon_can->set_operation_mode(BIGSSMaxonCAN::SupportedOperatingModes::CSP);
        m_maxon_can->CSP_command(msg.position[0]);
    }
    else if (m_lowlevel_pos_mode == LowlevelPosMode::PPM)
    {
        m_maxon_can->set_operation_mode(BIGSSMaxonCAN::SupportedOperatingModes::PPM);
        m_maxon_can->PPM_command(msg.position[0]);
    }
    else{
        ROS_ERROR("BIGSSMaxonCANROS: unsupported lowlevel_pos_mode");
    }
}

void BIGSSMaxonCANROS::servo_jv_cb(const sensor_msgs::JointState& msg)
{
    if(m_lowlevel_vel_mode == LowlevelVelMode::CSV)
    {
        m_maxon_can->set_operation_mode(BIGSSMaxonCAN::SupportedOperatingModes::CSV);
        m_maxon_can->CSV_command(msg.velocity[0]);

    }
    else if (m_lowlevel_vel_mode == LowlevelVelMode::PVM)
    {
        m_maxon_can->set_operation_mode(BIGSSMaxonCAN::SupportedOperatingModes::PVM);
        m_maxon_can->PVM_command(msg.velocity[0]);
    }
    else{
        ROS_ERROR("BIGSSMaxonCANROS: unsupported lowlevel_vel_mode");
    }
}

void BIGSSMaxonCANROS::pub_timer_cb(const ros::TimerEvent& event)
{
    // publish measured_js and measured_jv
    sensor_msgs::JointState js_msg;
    js_msg.header.stamp = ros::Time::now();
    js_msg.header.frame_id = "measured_js";
    js_msg.name.push_back("measured_js");
    js_msg.position.push_back(m_measured_js);
    m_measured_js_pub.publish(js_msg);

    sensor_msgs::JointState jv_msg;
    jv_msg.header.stamp = ros::Time::now();
    jv_msg.header.frame_id = "measured_jv";
    jv_msg.name.push_back("measured_jv");
    jv_msg.velocity.push_back(m_measured_jv);
    m_measured_jv_pub.publish(jv_msg);
}

void BIGSSMaxonCANROS::read_timer_cb(const ros::TimerEvent& event)
{
    // TODO: this callback can hang becuase the "read" functionality is blocking. Hangs until the next CAN message is received.
    m_maxon_can->read_and_parse_known_data();
    m_measured_js = m_maxon_can->m_position_rad;
    m_measured_jv = m_maxon_can->m_velocity_rad_per_sec;
}
