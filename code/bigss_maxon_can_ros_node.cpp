#include <iostream>
#include <memory>
#include <ros/ros.h>
#include <cm_actuation/BIGSSMaxonCANROS.hpp>
#include <boost/program_options.hpp>

int main(int argc, char **argv)
{
    namespace po = boost::program_options;
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "produce help message")
        ("can_device_name,c", po::value<std::string>()->default_value("can0"), "can device name")
        ("supported_actuator_name,a", po::value<std::string>()->default_value("roll_act"), "supported actuator name")
        ("pub_hz,p", po::value<double>()->default_value(500.0), "publish rate (hz)");

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    // parse command line options
    if (vm.count("help"))
    {
        std::cout << desc << "\n";
        return 1;
    }

    std::string can_device_name = vm["can_device_name"].as<std::string>();
    std::string supported_actuator_name = vm["supported_actuator_name"].as<std::string>();
    double pub_hz = vm["pub_hz"].as<double>();
   
    // ros init
    ros::init(argc, argv, supported_actuator_name);
    // make ros node handle
    ros::NodeHandle ros_nh(supported_actuator_name);

    // get parameters
    // std::string can_device_name = "can0";
    // std::string supported_actuator_name = "roll_actuator";
    // double pub_hz = 500.0;

    // create BIGSSMaxonCANROS object
    auto bigss_maxon_can_ros = std::make_unique<BIGSSMaxonCANROS>(ros_nh, can_device_name, supported_actuator_name, pub_hz);


    ros::AsyncSpinner spinner(0);
    spinner.start();
    ros::waitForShutdown();
    // ros spin
    // ros::spin();
}

