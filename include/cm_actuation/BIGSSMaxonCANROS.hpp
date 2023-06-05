#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <cm_actuation/BIGSSMaxonCAN.hpp>

// ROS1 ~CRTK compatible driver for BIGSS Maxon CAN

class BIGSSMaxonCANROS
{

public:
    BIGSSMaxonCANROS(const ros::NodeHandle ros_nh, const std::string &can_device_name, const std::string &supported_actuator_name, double pub_hz = 100.0);
    ~BIGSSMaxonCANROS(){};

private:
    std::unique_ptr<BIGSSMaxonCAN> m_maxon_can;
    ros::NodeHandle m_rosNodeHandle;
    ros::Publisher m_measured_js_pub;
    ros::Publisher m_measured_jv_pub;
    ros::Subscriber m_servo_jp_sub;
    ros::Subscriber m_servo_jv_sub;

    double measured_js = 0.0;
    double measured_jv = 0.0;

    ros::Timer m_pub_timer;
    void pub_timer_cb(const ros::TimerEvent &event);
    void servo_jp_cb(const sensor_msgs::JointState &msg);
    void servo_jv_cb(const sensor_msgs::JointState &msg);
};