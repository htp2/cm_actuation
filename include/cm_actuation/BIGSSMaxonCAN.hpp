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
        // This is the preferred constructor (i.e. you use a 'hardcoded' supported / known actuator)
        BIGSSMaxonCAN(const std::string& devicename, const std::string& supported_actuator_name, const SocketCAN::Rate rate = SocketCAN::Rate::RATE_1000);
        
        // This is the alternate constructor where you can specify all the properties needed yourself (e.g. if you have a new actuator that is not yet supported and don't add it to supported actuator list and recompile)
        BIGSSMaxonCAN(const std::string& devicename, const std::map<std::string, CiA301::COBID> cobid_map, const CiA301::Node::ID node_id, 
            const SocketCAN::Rate rate, const bool needs_homing, const std::vector<CiA301::Object> homing_sequence, const double encoder_to_rad, const double maxonvel_to_rad_per_sec, const double motor_rated_torque_nm);

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

        void initialize_can(const std::string& devicename, const SocketCAN::Rate rate = SocketCAN::Rate::RATE_1000);
        bool enable_PDO(const CiA301::Node::ID node_id=0x00);
        bool disable_PDO(const CiA301::Node::ID node_id=0x00);
        bool set_enable_state();
        bool set_disable_state();

        bool set_quick_stop();
        bool clear_quick_stop();
        bool set_operation_mode(const SupportedOperatingModes mode);

        bool PVM_command(const double velocity_rad_per_sec);
        bool PPM_command(const double position_rad, const double profile_velocity_rad_per_sec);
        bool CSV_command(const double velocity_rad_per_sec);
        bool CSP_command(const double position_rad);
        bool CST_command(const double torque);

        bool read_and_parse_known_data();
        bool send_transmit_requests();
        bool perform_homing_sequence();
        bool halt();


        double m_position_rad;
        double m_velocity_rad_per_sec;
        double m_torque_nm;
        double m_current_amp;
        bool m_is_homed = false;
        bool m_is_enabled = false;
        bool m_is_faulted = false;
        bool m_is_quick_stopped = false;

    private:
        const double M_RAD_PER_SEC_TO_RPM = 60.0 / (2.0 * M_PI);
        const double M_RPM_TO_RAD_PER_SEC = 1.0 / M_RAD_PER_SEC_TO_RPM;
        const double M_ROT_TO_RAD = 2.0 * M_PI;
        const double M_RAD_TO_ROT = 1.0 / M_ROT_TO_RAD; 
        const double M_MILLIX_TO_X = 1.0 / 1000.0; // milli to base SI
        std::map<std::string, CiA301::COBID> m_cobid_map;
        CiA301::Node::ID m_node_id;
        bool m_needs_homing = true; // set to false if, for example, you have an absolute encoder or don't care about homing
        std::vector<CiA301::Object> m_homing_sequence = {};
        double m_encoder_to_rad;
        double m_maxonvel_to_rad_per_sec;
        double m_motor_rated_torque_nm;

        bool write_can_sequence(const CiA301::COBID cobid, const std::vector<CiA301::Object> cmds);
        bool write_can_sequence(const std::vector<std::pair<const CiA301::COBID, const std::vector<CiA301::Object>>>& cmds);
        CiA301::Object pack_int32_into_can_obj(const int32_t value);
        bool extract_cobid_if_supported(const std::string& cmd_name, CiA301::COBID& cobid);
        bool check_if_in_correct_mode(const SupportedOperatingModes mode);
        bool check_is_homed_before_moving();



    private:
        std::unique_ptr<CANopen> canopen_reader;
        std::unique_ptr<CANopen> canopen_commander;
        std::unique_ptr<CANopen> canopen_rtr;

        SupportedOperatingModes m_operating_mode;

};
