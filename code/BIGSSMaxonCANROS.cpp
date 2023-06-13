#include <memory>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_srvs/Trigger.h>
#include <std_msgs/Bool.h>

#include <cm_actuation/BIGSSMaxonCANROS.hpp>

// Please note: due to use of multiple timers, please use the async spinner your the main function (you will need multiple threads)
// e.g. ros::AsyncSpinner spinner(0); spinner.start(); ros::waitForShutdown();

BIGSSMaxonCANROS::BIGSSMaxonCANROS(const ros::NodeHandle ros_nh, const std::string& can_device_name, const std::string& supported_actuator_name, 
    double pub_hz, OpModes vel_mode, OpModes pos_mode)
: m_rosNodeHandle(ros_nh), m_lowlevel_vel_mode(vel_mode), m_lowlevel_pos_mode(pos_mode)
{
    m_measured_js_pub = m_rosNodeHandle.advertise<sensor_msgs::JointState>("measured_js", 1);
    m_is_homed_pub = m_rosNodeHandle.advertise<std_msgs::Bool>("is_homed", 1);
    m_is_enabled_pub = m_rosNodeHandle.advertise<std_msgs::Bool>("is_enabled", 1);

    m_move_jp_sub = m_rosNodeHandle.subscribe("move_jp", 1, &BIGSSMaxonCANROS::move_jp_cb, this);
    m_servo_jv_sub = m_rosNodeHandle.subscribe("servo_jv", 1, &BIGSSMaxonCANROS::servo_jv_cb, this);
    m_enable_srv = m_rosNodeHandle.advertiseService("enable", &BIGSSMaxonCANROS::enable_srv_cb, this);
    m_disable_srv = m_rosNodeHandle.advertiseService("disable", &BIGSSMaxonCANROS::disable_srv_cb, this);
    m_set_cmd_mode_vel_srv = m_rosNodeHandle.advertiseService("set_cmd_mode_vel", &BIGSSMaxonCANROS::set_cmd_mode_vel_cb, this);
    m_set_cmd_mode_pos_srv = m_rosNodeHandle.advertiseService("set_cmd_mode_pos", &BIGSSMaxonCANROS::set_cmd_mode_pos_cb, this);
    m_home_srv = m_rosNodeHandle.advertiseService("home", &BIGSSMaxonCANROS::home_srv_cb, this);
    m_halt_srv = m_rosNodeHandle.advertiseService("halt", &BIGSSMaxonCANROS::halt_srv_cb, this);

    // create timer to publish measured_js
    m_pub_timer = m_rosNodeHandle.createTimer(ros::Duration(1.0/pub_hz), &BIGSSMaxonCANROS::pub_timer_cb, this);

    // Try to read as fast as possible / telemetry rate
    m_read_timer = m_rosNodeHandle.createTimer(ros::Duration(1.0/10000.0), &BIGSSMaxonCANROS::read_timer_cb, this);

    // create a timer for RTR (this is where you can periodically ping the device to send out updates)
    m_rtr_timer = m_rosNodeHandle.createTimer(ros::Duration(1.0/100.0), &BIGSSMaxonCANROS::rtr_timer_cb, this);

    //FUTURE: could make a settable velocity timeout rate
    m_vel_cmd_timeout_timer = m_rosNodeHandle.createTimer(ros::Duration(1.0), &BIGSSMaxonCANROS::vel_cmd_timeout_timer_cb, this);
    m_vel_cmd_timeout_timer.stop(); // only will be enabled when we are in velocity mode

    // create BIGSSMaxonCAN object
    ROS_INFO("BIGSSMaxonCANROS: Attempting to create BIGSSMaxonCAN object");
    m_maxon_can = std::make_unique<BIGSSMaxonCAN>(can_device_name, supported_actuator_name);
    ROS_INFO("BIGSSMaxonCANROS: BIGSSMaxonCAN object created");

    m_maxon_can->enable_PDO(); // enables communication with device
    m_maxon_can->set_enable_state(); 
}
// note the BIGSSMaxonCAN has a constructor that allows you to directly specify your own cobid map.
// An equivalent constructor for the ROS wrapper could be easily added if needed

BIGSSMaxonCANROS::~BIGSSMaxonCANROS()
{
    m_maxon_can->set_disable_state();
    m_pub_timer.stop();
    m_read_timer.stop();
    m_rtr_timer.stop();
    m_vel_cmd_timeout_timer.stop();
}

bool BIGSSMaxonCANROS::move_jp(const double position_rad, const double profile_velocity_rad_per_sec){
    if(m_lowlevel_pos_mode == OpModes::CSP)
    {
        m_maxon_can->CSP_command(position_rad); // FUTURE: Not fully implemented on BIGSSMaxonCAN yet, probably should be a servo_jp command not move_jp
    }
    else if (m_lowlevel_pos_mode == OpModes::PPM)
    {
        m_maxon_can->PPM_command(position_rad, profile_velocity_rad_per_sec); 
    }
    else{
        ROS_ERROR("BIGSSMaxonCANROS: unsupported lowlevel_pos_mode");
        return false;
    }
    return true;
}

bool BIGSSMaxonCANROS::servo_jv(const double velocity_rad_per_sec){
    if(m_lowlevel_vel_mode == OpModes::CSV)
    {
        m_maxon_can->CSV_command(velocity_rad_per_sec);
    }
    else if (m_lowlevel_vel_mode == OpModes::PVM)
    {
        m_maxon_can->PVM_command(velocity_rad_per_sec);
    }
    else{
        ROS_ERROR("BIGSSMaxonCANROS: unsupported lowlevel_vel_mode");
        return false;
    }
    return true;
}


void BIGSSMaxonCANROS::move_jp_cb(const sensor_msgs::JointState& msg)
{
    move_jp(msg.position[0], msg.velocity[0]);
}

void BIGSSMaxonCANROS::servo_jv_cb(const sensor_msgs::JointState& msg)
{
    auto command_sent = servo_jv(msg.velocity[0]);

    // we restart the timeout timer every time we receive a new non-zero velocity command
    if (command_sent && msg.velocity[0] != 0.0){
        m_vel_cmd_timeout_timer.stop();
        m_vel_cmd_timeout_timer.start();
    }
    else
        m_vel_cmd_timeout_timer.stop();
}

void BIGSSMaxonCANROS::pub_timer_cb(const ros::TimerEvent& event)
{
    // publish measured_js and measured_jv
    sensor_msgs::JointState js_msg;
    js_msg.header.stamp = ros::Time::now();
    js_msg.header.frame_id = "bigss_maxon_can"; // TODO replace with name
    js_msg.name.push_back("bigss_maxon_can");
    js_msg.position.push_back(m_measured_jp);
    js_msg.velocity.push_back(m_measured_jv);
    js_msg.effort.push_back(m_measured_jt);
    m_measured_js_pub.publish(js_msg);

    // publish is_homed
    std_msgs::Bool is_homed_msg;
    is_homed_msg.data = m_is_homed;
    m_is_homed_pub.publish(is_homed_msg);

    // publish is_enabled
    std_msgs::Bool is_enabled_msg;
    is_enabled_msg.data = m_maxon_can->m_is_enabled;
    m_is_enabled_pub.publish(is_enabled_msg);
}

void BIGSSMaxonCANROS::read_timer_cb(const ros::TimerEvent& event)
{
    // FUTURE: this callback can hang becuase the "read" functionality is blocking. Hangs until the next CAN message is received.
    // usually not an issue, but might have to kill the proccess, not just Ctrl+C if stopping node while it is waiting for a CAN message
    m_maxon_can->read_and_parse_known_data();
    m_measured_jp = m_maxon_can->m_position_rad;
    m_measured_jv = m_maxon_can->m_velocity_rad_per_sec;
    m_measured_jt = m_maxon_can->m_torque_nm;
    m_is_homed = m_maxon_can->m_is_homed;
}

void BIGSSMaxonCANROS::vel_cmd_timeout_timer_cb(const ros::TimerEvent& event)
{
    if (m_lowlevel_vel_mode == OpModes::CSV || m_lowlevel_vel_mode == OpModes::PVM)
    {
        ROS_WARN("BIGSSMaxonCANROS: No new velocity command received after specified timeout interval. Zeroing out velocity command.");
        m_vel_cmd_timeout_timer.stop();
        servo_jv(0.0); // zero out velocity command if no new command is received for a while
    }
}

void BIGSSMaxonCANROS::rtr_timer_cb(const ros::TimerEvent& event)
{
    m_maxon_can->send_transmit_requests();
}

bool BIGSSMaxonCANROS::enable_srv_cb(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
    res.success = m_maxon_can->set_enable_state();
    if (res.success)
        res.message = "BIGSSMaxonCANROS: enabled";
    else
        res.message = "BIGSSMaxonCANROS: failed to enable";
    return true;
}

bool BIGSSMaxonCANROS::disable_srv_cb(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
    res.success = m_maxon_can->set_disable_state();
    if (res.success)
        res.message = "BIGSSMaxonCANROS: disabled";
    else
        res.message = "BIGSSMaxonCANROS: failed to disable";
    return true;
}

bool BIGSSMaxonCANROS::set_cmd_mode_vel_cb(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
    res.success = m_maxon_can->set_operation_mode(m_lowlevel_vel_mode);
    if (res.success)
        res.message = "BIGSSMaxonCANROS: set_cmd_mode_vel";
    else
        res.message = "BIGSSMaxonCANROS: failed to set_cmd_mode_vel";
    return true;
}

bool BIGSSMaxonCANROS::set_cmd_mode_pos_cb(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
    res.success = m_maxon_can->set_operation_mode(m_lowlevel_pos_mode);
    if (res.success)
        res.message = "BIGSSMaxonCANROS: set_cmd_mode_pos";
    else
        res.message = "BIGSSMaxonCANROS: failed to set_cmd_mode_pos";
    return true;
}

bool BIGSSMaxonCANROS::home_srv_cb(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
    res.success = m_maxon_can->perform_homing_sequence();
    if (res.success)
        res.message = "BIGSSMaxonCANROS: homing sequence started";
    else
        res.message = "BIGSSMaxonCANROS: failed to start homing sequence";
    return true;
}

bool BIGSSMaxonCANROS::halt_srv_cb(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
    res.success = m_maxon_can->halt();
    if (res.success)
        res.message = "BIGSSMaxonCANROS: halt";
    else
        res.message = "BIGSSMaxonCANROS: failed to halt";
    return true;
}
