#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <cm_actuation/BIGSSMaxonCAN.hpp>

// ROS1 ~CRTK compatible driver for BIGSS Maxon CAN
enum LowlevelVelMode{PVM, CSV};
enum LowlevelPosMode{PPM, CSP};
class BIGSSMaxonCANROS
{

public:
    BIGSSMaxonCANROS(const ros::NodeHandle ros_nh, const std::string &can_device_name, const std::string &supported_actuator_name, double pub_hz = 100.0, LowlevelVelMode vel_mode = CSV, LowlevelPosMode pos_mode = PPM);
    ~BIGSSMaxonCANROS(){};
    
private:
    std::unique_ptr<BIGSSMaxonCAN> m_maxon_can;
    ros::NodeHandle m_rosNodeHandle;
    ros::Publisher m_measured_js_pub;
    ros::Publisher m_measured_jv_pub;
    ros::Subscriber m_servo_jp_sub;
    ros::Subscriber m_servo_jv_sub;

    LowlevelVelMode m_lowlevel_vel_mode = CSV; // default to CSV
    LowlevelPosMode m_lowlevel_pos_mode = PPM; // default to PPM

    double m_measured_js = 0.0;
    double m_measured_jv = 0.0;

    ros::Timer m_pub_timer;
    void pub_timer_cb(const ros::TimerEvent &event);
    ros::Timer m_read_timer;
    void read_timer_cb(const ros::TimerEvent &event);

    void servo_jp_cb(const sensor_msgs::JointState &msg);
    void servo_jv_cb(const sensor_msgs::JointState &msg);
};