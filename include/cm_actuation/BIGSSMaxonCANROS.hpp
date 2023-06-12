#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_srvs/Trigger.h>
#include <cm_actuation/BIGSSMaxonCAN.hpp>

// ROS1 ~CRTK compatible driver for BIGSS Maxon CAN
// enum LowlevelVelMode{PVM, CSV};
// enum LowlevelPosMode{PPM, CSP};

// this is just for readability
typedef BIGSSMaxonCAN::SupportedOperatingModes OpModes;

class BIGSSMaxonCANROS
{

public:
    BIGSSMaxonCANROS(const ros::NodeHandle ros_nh, const std::string &can_device_name, const std::string &supported_actuator_name, 
        double pub_hz = 100.0, OpModes vel_mode = OpModes::CSV, OpModes pos_mode = OpModes::PPM);
    ~BIGSSMaxonCANROS();
    
private:
    std::unique_ptr<BIGSSMaxonCAN> m_maxon_can;
    ros::NodeHandle m_rosNodeHandle;
    ros::Publisher m_measured_js_pub;
    ros::Publisher m_measured_jv_pub;
    ros::Subscriber m_servo_jp_sub;
    ros::Subscriber m_servo_jv_sub;
    ros::ServiceServer m_enable_srv;
    ros::ServiceServer m_disable_srv;
    ros::ServiceServer m_set_cmd_mode_vel_srv;
    ros::ServiceServer m_set_cmd_mode_pos_srv;

    OpModes m_lowlevel_vel_mode = OpModes::CSV; // default to CSV
    OpModes m_lowlevel_pos_mode = OpModes::PPM; // default to PPM

    double m_measured_js = 0.0;
    double m_measured_jv = 0.0;

    bool servo_jp(const double position_rad);
    bool servo_jv(const double velocity_rad_per_sec);

    ros::Timer m_pub_timer;
    void pub_timer_cb(const ros::TimerEvent &event);
    ros::Timer m_read_timer;
    void read_timer_cb(const ros::TimerEvent &event);
    ros::Timer m_vel_cmd_timeout_timer; // for saftey, zero out velocity command if no new command is received for a while
    void vel_cmd_timeout_timer_cb(const ros::TimerEvent &event);

    void servo_jp_cb(const sensor_msgs::JointState &msg);
    void servo_jv_cb(const sensor_msgs::JointState &msg);
    bool enable_srv_cb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
    bool disable_srv_cb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
    bool set_cmd_mode_vel_cb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
    bool set_cmd_mode_pos_cb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
};