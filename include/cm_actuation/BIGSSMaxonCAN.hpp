#include <iostream>

#include "canopen.hpp"
#include <map>
#include <math.h>

// disclaimer: this is NOT general code for controlling any maxon motor over CANopen. It contains some specifc assumptions about the motor firmware
// settings, etc. It should only be used without modification for controlling the continuum manipulator actuation unit and roll actuator. 
// That said, light modification should make it usable for other maxon motors / projects.
// Basic CANopen functionality is borrowed from Simon Leonard's maxon_rtt package. That package was not used here as it uses OROCOS which is not 
// compatabile with the software framework (utilizing ROS Noetic) used for the rest of the robot system code at the time of writing.


class BIGSSMaxonCAN
{
    public:
        BIGSSMaxonCAN(const std::string& devicename, const std::map<std::string, CiA301::COBID> cobid_map, const SocketCAN::Rate rate = SocketCAN::Rate::RATE_1000);
        BIGSSMaxonCAN(const std::string& devicename, const std::string& supported_actuator_name, const SocketCAN::Rate rate = SocketCAN::Rate::RATE_1000);

        ~BIGSSMaxonCAN();
      
        enum SupportedOperatingModes  //TODO? :supportedCommandModes? Could have separate enum for keeping track of all modes for operational state variable
        {
            PPM = 0x01, // profile position mode
            PVM = 0x03, // profile velocity mode
            HMM = 0x06, // homing mode
            CSP = 0x08, // cyclic synchronous position mode
            CSV = 0x09, // cyclic synchronous velocity mode
            CST = 0x0A, // cyclic synchronous torque mode
        };
        bool enable_PDO(const CiA301::Node::ID node_id=0x00);
        bool disable_PDO(const CiA301::Node::ID node_id=0x00);
        bool set_enable_state();
        bool set_disable_state();
        bool perform_canned_homing();

        bool set_quick_stop();
        bool clear_quick_stop();
        bool set_operation_mode(const SupportedOperatingModes mode);

        bool PVM_command(const double velocity_rad_per_sec);
        bool PPM_command(const double position_rad);
        bool CSV_command(const double velocity_rad_per_sec);
        bool CSP_command(const double position_rad);
        bool CST_command(const double torque);
        

    private:
        const double M_RAD_PER_SEC_TO_RPM = 60.0 / (2.0 * M_PI);
        const double M_RPM_TO_RAD_PER_SEC = 1.0 / M_RAD_PER_SEC_TO_RPM;
        std::map<std::string, CiA301::COBID> m_cobid_map;
        CiA301::Node::ID m_node_id;
        
        bool write_can_sequence(const CiA301::COBID cobid, const std::vector<CiA301::Object> cmds);
        bool write_can_sequence(const std::vector<std::pair<const CiA301::COBID, const std::vector<CiA301::Object>>>& cmds);
        CiA301::Object pack_int32_into_can_obj(const int32_t value);
        bool extract_cobid_if_supported(const std::string& cmd_name, CiA301::COBID& cobid);



    private:
        std::unique_ptr<CANopen> canopen;

        SupportedOperatingModes m_operating_mode;

};
