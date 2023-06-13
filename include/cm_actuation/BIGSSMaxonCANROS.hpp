#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_srvs/Trigger.h>
#include <cm_actuation/BIGSSMaxonCAN.hpp>

// ROS1 ~CRTK compatible driver for BIGSS Maxon CAN

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
    ros::Publisher m_is_homed_pub;
    ros::Publisher m_is_enabled_pub; // FUTURE: could support/implement full state machine, or at least crtk compliant operating_state

    ros::Subscriber m_move_jp_sub;
    ros::Subscriber m_servo_jv_sub;
    ros::ServiceServer m_enable_srv;
    ros::ServiceServer m_disable_srv;
    ros::ServiceServer m_set_cmd_mode_vel_srv;
    ros::ServiceServer m_set_cmd_mode_pos_srv;
    ros::ServiceServer m_home_srv;
    ros::ServiceServer m_halt_srv;

    OpModes m_lowlevel_vel_mode = OpModes::PVM; // default //FUTURE: make this configurable
    OpModes m_lowlevel_pos_mode = OpModes::PPM; // default //FUTURE: make this configurable

    double m_measured_jp = 0.0; // joint position (rad)
    double m_measured_jv = 0.0; // joint velocity (rad/s)
    double m_measured_jt = 0.0; // joint torque (Nm)
    bool m_is_homed = false;

    bool move_jp(const double position_rad, const double profile_velocity_rad_per_sec);
    bool servo_jv(const double velocity_rad_per_sec);

    ros::Timer m_pub_timer; // how often to publish ros messages
    void pub_timer_cb(const ros::TimerEvent &event); // FUTURE: Could slow down publishing rate for some info (e.g. is_homed, is_enabled)
    ros::Timer m_read_timer; // how often to read from can (you should make arbitrarily high to read as fast as possible)
    void read_timer_cb(const ros::TimerEvent &event);
    ros::Timer m_rtr_timer; // how often to send RTR requests (this forces updates from device even if nothing changed) Useful for e.g. detecting state on startup
    void rtr_timer_cb(const ros::TimerEvent &event);
    ros::Timer m_vel_cmd_timeout_timer; // for saftey, zero out velocity command if no new command is received for a while
    void vel_cmd_timeout_timer_cb(const ros::TimerEvent &event);


    void move_jp_cb(const sensor_msgs::JointState &msg);
    void servo_jv_cb(const sensor_msgs::JointState &msg);
    bool enable_srv_cb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
    bool disable_srv_cb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
    bool set_cmd_mode_vel_cb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
    bool set_cmd_mode_pos_cb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
    bool home_srv_cb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
    bool halt_srv_cb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
};