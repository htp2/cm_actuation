#include <iostream>

#include "canopen.hpp"

// boost includes for program options
#include <boost/program_options.hpp>

int main(int argc, char **argv)
{
    // boost program options first is devicename which defaults to can0
    // get devicename from command line
    std::string devicename;
    boost::program_options::options_description desc("Allowed options");
    desc.add_options()
        ("help", "produce help message")
        ("devicename", boost::program_options::value<std::string>(&devicename)->default_value("can0"), "device name")
    ;
    boost::program_options::variables_map vm;
    boost::program_options::store(boost::program_options::parse_command_line(argc, argv, desc), vm);
    boost::program_options::notify(vm);

    if (vm.count("help")) {
        std::cout << desc << "\n";
        return 1;
    }

    std::cout << "devicename: " << devicename << std::endl;

    CANopen canopen(devicename, SocketCAN::Rate::RATE_1000, SocketCAN::Loopback::LOOPBACK_OFF);
    canopen.Open();
    CiA301::COBID main_cobid = 0x0;
    CiA301::COBID roll_cobid = 0x203;
    CiA301::COBID roll_vel_cobid = 0x503;
    CiA301::Object object;

    auto pre_operational_state = 0x80;

    auto node_id_all = 0x00;
    auto node_id_roll = 0x03;
    auto node_id_act = 0x01;


    // set PDO i.e. send 0x0100 to main_cobid
    CiA301::Object obj;







    std::cout << "Press enter to continue" << std::endl;
    std::cin.get();

    // send 0x0600 to roll_cobid
    // obj. = {0x06, 0x00};
    canopen.Write(roll_cobid, obj);
    std::cout << "Press enter to continue" << std::endl;
    std::cin.get();

    // send 0x0f00 to roll_cobid
    obj.data = {0x0f, 0x00};
    canopen.Write(roll_cobid, obj);

    // sleep(1);

    // canopen.Read(cobid, object);
    // std::cout << "cobid: " << cobid << std::endl;
    // std::cout << "object: " << object << std::endl;
    
    // disaable pdo 

    // wait for user input
    std::cout << "Press enter to continue" << std::endl;
    std::cin.get();

    obj.data = {0x80, 0x00};
    canopen.Write(main_cobid, obj);

    obj.data = {0x00, 0x00};
    canopen.Write(main_cobid, obj);
    
    canopen.Close();
	return 0;
}

bool set_operational_mode(unsigned char node_id)
{
    auto operational_state = 0x01;
    canopen.Write(main_cobid, CiA301::Object({operational_state, node_id}));
}