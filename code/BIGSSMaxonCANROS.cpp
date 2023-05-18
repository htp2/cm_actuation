#include <iostream>

#include "canopen.hpp"
#include <ros/ros.h>

class BIGSSMaxonCANROS
{
    public:
        BIGSSMaxonCANROS();
        ~BIGSSMaxonCANROS(){};

    private:
        ros::NodeHandle m_rosNodeHandle;
        ros::Publisher m_measured_js_pub;
        ros::Publisher m_measured_jv_pub;
        ros::Subscriber m_servo_jp_sub;
        ros::Subscriber m_servo_jv_sub;


}

BIGSSMaxonCANROS::BIGSSMaxonCANROS()
{
    
}