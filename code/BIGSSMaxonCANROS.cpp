#include <iostream>

#include "canopen.hpp"
#include <ros/ros.h>

class BIGSSMaxonCANROS
{
    public:
        BIGSSMaxonCANROS(const std::string& devicename);
        ~BIGSSMaxonCANROS(){};

    private:
        ros::NodeHandle m_rosNodeHandle;
        ros::Publisher m_measured_js_pub;
        ros::Publisher m_measured_jv_pub;
        ros::Subscriber m_servo_jp_sub;
        ros::Subscriber m_servo_jv_sub;
        std::unique_ptr<CANopen> canopen;


};

struct BIGSSMaxonCANCMD{
    

};

BIGSSMaxonCANROS::BIGSSMaxonCANROS(const std::string& devicename)
{
    canopen = std::make_unique<CANopen>(devicename, SocketCAN::Rate::RATE_1000, SocketCAN::Loopback::LOOPBACK_OFF);
    canopen->Open();
}

