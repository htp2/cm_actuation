#include <iostream>
#include <memory>
#include <thread>
#include <chrono>

#include "canopen.hpp"
#include <cm_actuation/BIGSSMaxonCAN.hpp>

// This is the preferred constructor (i.e. you use a 'hardcoded' supported / known actuator)
BIGSSMaxonCAN::BIGSSMaxonCAN(const std::string &devicename, const std::string &supported_actuator_name, const SocketCAN::Rate rate)
{
    if (supported_actuator_name == "roll_act")
    {
        m_node_id = 0x03;
        m_cobid_map = {
            {"enable_pdo", 0x000},
            {"enable_state", 0x203},
            {"homing", 0x203},
            {"pvm_target", 0x403},
            {"pvm_exec", 0x503},
            {"ppm_target", 0x303},
            {"ppm_exec", 0x503},
            {"csp_target", 0x303},
            {"csv_target", 0x403},
            {"cst_target", 0x503},
            {"op_mode", 0x203},
            {"quick_stop", 0x203},
            {"read_pos_vel", 0x283},
            {"read_cur_tor", 0x383},
            {"read_stat_op", 0x183},
        };
        m_needs_homing = true; // this would be false if, for example you have an absolute encoder
        m_homing_sequence = {
            CiA301::Object({0x0F, 0x01, 0x06, 0x17, 0x64, 0x00, 0x00, 0x00}),
            CiA301::Object({0x0F, 0x00, 0x06, 0x17, 0x64, 0x00, 0x00, 0x00}),
            CiA301::Object({0x1F, 0x00, 0x06, 0x17, 0x64, 0x00, 0x00, 0x00})};
        m_encoder_to_rad = M_ROT_TO_RAD / (4 * 12800.0);              // 12800 ticks per rotation and quadrature encoder
        m_maxonvel_to_rad_per_sec = M_RPM_TO_RAD_PER_SEC / 2.0 * 0.1; // 2:1 I/O gear ratio, 0.1 increment on rpm command
        m_motor_rated_torque_nm = 1203.162 * M_MILLIX_TO_X;           // 1203.162 mNm
    }
    if (supported_actuator_name == "act_unit")
    {
        m_node_id = 0x01;
        m_cobid_map = {
            {"enable_pdo", 0x000},
            {"enable_state", 0x201},
            {"pvm_target", 0x401},
            {"pvm_exec", 0x501},
            {"ppm_target", 0x301},
            {"ppm_exec", 0x501},
            {"csp_target", 0x301},
            {"csv_target", 0x401},
            {"cst_target", 0x501},
            {"op_mode", 0x201},
            {"quick_stop", 0x201},
            {"read_pos_vel", 0x281},
            {"read_cur_tor", 0x381},
            {"read_stat_op", 0x181},
        };
        m_needs_homing = false; // absolute encoder
        m_homing_sequence = {};
        m_encoder_to_rad = M_ROT_TO_RAD / (4096.0);              // 4096 ticks per rotation
        m_maxonvel_to_rad_per_sec = M_RPM_TO_RAD_PER_SEC / 1.0 * 0.1; // 1:1 I/O gear ratio, 0.1 increment on rpm command
        m_motor_rated_torque_nm = 115.059 * M_MILLIX_TO_X;           // 115.059 mNm
    }

    else
    {
        std::cerr << "BIGSSMaxonCAN: supported_actuator_name not recognized. Will not create." << std::endl;
        return;
    }
    initialize_can(devicename, rate);
}

// This is the alternate constructor where you can specify all the properties needed yourself (e.g. if you have a new actuator that is not yet supported and don't add it to supported actuator list and recompile)
BIGSSMaxonCAN::BIGSSMaxonCAN(const std::string &devicename, const std::map<std::string, CiA301::COBID> cobid_map, const CiA301::Node::ID node_id, const SocketCAN::Rate rate,
                             const bool needs_homing, const std::vector<CiA301::Object> homing_sequence, const double encoder_to_rad, const double maxonvel_to_rad_per_sec, const double motor_rated_torque_nm)
    : m_node_id(node_id), m_cobid_map(cobid_map), m_needs_homing(needs_homing), m_homing_sequence(homing_sequence), m_encoder_to_rad(encoder_to_rad), m_maxonvel_to_rad_per_sec(maxonvel_to_rad_per_sec), m_motor_rated_torque_nm(motor_rated_torque_nm)
{
    initialize_can(devicename, rate);
}

void BIGSSMaxonCAN::initialize_can(const std::string &devicename, const SocketCAN::Rate rate)
{
    canopen_reader = std::make_unique<CANopen>(devicename, rate, SocketCAN::Loopback::LOOPBACK_OFF);
    canopen_commander = std::make_unique<CANopen>(devicename, rate, SocketCAN::Loopback::LOOPBACK_OFF);
    canopen_rtr = std::make_unique<CANopen>(devicename, rate, SocketCAN::Loopback::LOOPBACK_OFF);

    canopen_reader->Open();
    canopen_commander->Open();
    canopen_rtr->Open();
}

BIGSSMaxonCAN::~BIGSSMaxonCAN()
{
    canopen_commander->Close();
}

bool BIGSSMaxonCAN::write_can_sequence(const CiA301::COBID cobid, const std::vector<CiA301::Object> cmds)
{
    // small helper function to write a sequence of commands to a cobid, returns early with false if any command fails
    // this sends all to the same cobid
    for (auto cmd : cmds)
    {
        auto result = canopen_commander->Write(cobid, cmd);
        if (result != CANopen::ESUCCESS)
        {
            return false;
        }
        // put a millisecond delay between each command
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return true;
}

bool BIGSSMaxonCAN::write_can_sequence(const std::vector<std::pair<const CiA301::COBID, const std::vector<CiA301::Object>>> &cmds)
{
    // small helper function to write a sequence of commands to a cobid, returns early with false if any command fails
    // this sends to different cobids
    for (auto cmd : cmds)
    {
        for (auto cmd_obj : cmd.second)
        {
            auto result = canopen_commander->Write(cmd.first, cmd_obj);
            if (result != CANopen::ESUCCESS)
            {
                return false;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
    return true;
}

CiA301::Object BIGSSMaxonCAN::pack_int32_into_can_obj(const int32_t value)
{
    return CiA301::Object({static_cast<unsigned char>(value & 0xFF),
                           static_cast<unsigned char>((value >> 8) & 0xFF),
                           static_cast<unsigned char>((value >> 16) & 0xFF),
                           static_cast<unsigned char>((value >> 24) & 0xFF)});
}

bool BIGSSMaxonCAN::extract_cobid_if_supported(const std::string &cmd_name, CiA301::COBID &cobid)
{
    // helper function to extract cobid from cobid map if it exists
    if (m_cobid_map.find(cmd_name) != m_cobid_map.end())
    {
        cobid = m_cobid_map.at(cmd_name);
        return true;
    }
    else
    {
        std::cerr << "BIGSSMaxonCAN: COBID needed for command was not found in map. Will not execute command." << std::endl;
        return false;
    }
}

bool BIGSSMaxonCAN::check_if_in_correct_mode(const SupportedOperatingModes mode)
{
    // helper function to check if in correct mode before sending command
    if (m_operating_mode != mode)
    {
        std::cerr << "BIGSSMaxonCAN: Not in correct mode to send command. Will not execute command." << std::endl;
        return false;
    }
    return true;
}

bool BIGSSMaxonCAN::enable_PDO(const CiA301::Node::ID node_id)
{
    // node_id = 0x00 means all nodes, or you can specify a node_id
    CiA301::COBID cobid;
    if (!extract_cobid_if_supported("enable_pdo", cobid))
        return false;
    auto result = canopen_commander->Write(cobid, CiA301::Object({0x01, node_id}));
    return result == CANopen::ESUCCESS;
}

bool BIGSSMaxonCAN::disable_PDO(const CiA301::Node::ID node_id)
{
    // node_id = 0x00 means all nodes, or you can specify a node_id
    CiA301::COBID cobid;
    if (!extract_cobid_if_supported("enable_pdo", cobid))
        return false;
    auto result = canopen_commander->Write(node_id, CiA301::Object({0x80, node_id}));
    return result == CANopen::ESUCCESS;
}

bool BIGSSMaxonCAN::set_enable_state()
{
    CiA301::COBID cobid;
    if (!extract_cobid_if_supported("enable_state", cobid))
        return false;
    auto cmd1 = CiA301::Object({0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}); // disable per state machine chart
    auto cmd2 = CiA301::Object({0x0f, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}); // 0x01 for safety (halt bit) instead of 0x00 (execute)
    auto result = write_can_sequence(cobid, {cmd1, cmd2});
    return result;
}

bool BIGSSMaxonCAN::set_disable_state()
{
    CiA301::COBID cobid;
    if (!extract_cobid_if_supported("enable_state", cobid))
        return false;
    auto cmd = CiA301::Object({0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00});
    auto result = canopen_commander->Write(cobid, cmd);
    return result == CANopen::ESUCCESS;
}

bool BIGSSMaxonCAN::set_quick_stop()
{
    CiA301::COBID cobid;
    if (!extract_cobid_if_supported("quick_stop", cobid))
        return false;
    auto cmd = CiA301::Object({0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00});
    auto result = canopen_commander->Write(cobid, cmd);
    return result == CANopen::ESUCCESS;
}

bool BIGSSMaxonCAN::clear_quick_stop()
{
    CiA301::COBID cobid;
    if (!extract_cobid_if_supported("quick_stop", cobid))
        return false;
    auto cmd = CiA301::Object({0x0f, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}); // 0x01 for safety (halt bit) instead of 0x00 (execute)
    auto result = canopen_commander->Write(cobid, cmd);
    return result == CANopen::ESUCCESS;
}

bool BIGSSMaxonCAN::set_operation_mode(const SupportedOperatingModes mode)
{
    CiA301::COBID cobid;
    if (!extract_cobid_if_supported("op_mode", cobid))
        return false;
    auto cmd = CiA301::Object({0x0f, 0x01, mode, 0x00, 0x00, 0x00, 0x00, 0x00});
    auto result = canopen_commander->Write(cobid, cmd);
    auto bool_res = result == CANopen::ESUCCESS;
    if (bool_res)
        m_operating_mode = mode;
    return bool_res;
}

bool BIGSSMaxonCAN::PVM_command(const double velocity_rad_per_sec)
{
    if (!check_is_homed_before_moving())
        return false;

    if (!check_if_in_correct_mode(SupportedOperatingModes::PVM))
        return false;

    CiA301::COBID cobid1;
    if (!extract_cobid_if_supported("pvm_target", cobid1))
        return false;
    CiA301::COBID cobid2;
    if (!extract_cobid_if_supported("pvm_exec", cobid2))
        return false;
    auto velocity_rpm = velocity_rad_per_sec / m_maxonvel_to_rad_per_sec;
    auto command_int32 = static_cast<int32_t>(velocity_rpm);
    auto cmd1 = pack_int32_into_can_obj(command_int32);
    auto cmd2 = CiA301::Object({0x0f, 0x00, 0x03, 0x60, 0x00, 0x00, 0x00, 0x00}); // set velocity target

    auto result = write_can_sequence({{cobid1, {cmd1}}, {cobid2, {cmd2}}});

    return result;
}

bool BIGSSMaxonCAN::PPM_command(const double target_position_rad, const double profile_velocity_rad_per_sec)
{
    if (!check_is_homed_before_moving())
        return false;

    if (!check_if_in_correct_mode(SupportedOperatingModes::PPM))
        return false;

    CiA301::COBID cobid1;
    if (!extract_cobid_if_supported("ppm_target", cobid1))
        return false;
    CiA301::COBID cobid2;
    if (!extract_cobid_if_supported("ppm_exec", cobid2))
        return false;
    auto target_position_encoder = target_position_rad / m_encoder_to_rad;
    auto command_int32 = static_cast<int32_t>(target_position_encoder);
    auto cmd1 = pack_int32_into_can_obj(command_int32);

    auto profile_velocity_rpm = profile_velocity_rad_per_sec / m_maxonvel_to_rad_per_sec;
    command_int32 = static_cast<int32_t>(profile_velocity_rpm);
    // we have to prepend the data with two bytes for command word and postpend by two zero bytes, hence manually parsing the command_int32 and not using the function. the CiA301::Object constructor is very picky
    auto cmd2 = CiA301::Object({0x3F, 0x00, static_cast<unsigned char>(command_int32 & 0xFF), static_cast<unsigned char>((command_int32 >> 8) & 0xFF), static_cast<unsigned char>((command_int32 >> 16) & 0xFF), static_cast<unsigned char>((command_int32 >> 24) & 0xFF), 0x00, 0x00});

    auto cmd3 = CiA301::Object({0x0f, 0x00, static_cast<unsigned char>(command_int32 & 0xFF), static_cast<unsigned char>((command_int32 >> 8) & 0xFF), static_cast<unsigned char>((command_int32 >> 16) & 0xFF), static_cast<unsigned char>((command_int32 >> 24) & 0xFF), 0x00, 0x00});

    auto result = write_can_sequence({{cobid1, {cmd1}}, {cobid2, {cmd2, cmd3}}});
    return result;
}

bool BIGSSMaxonCAN::CSV_command(const double velocity_rad_per_sec)
{
    if (!check_is_homed_before_moving())
        return false;

    if (!check_if_in_correct_mode(SupportedOperatingModes::CSV))
        return false;

    CiA301::COBID cobid;
    if (!extract_cobid_if_supported("csv_target", cobid))
        return false;
    auto velocity_rpm = velocity_rad_per_sec / m_maxonvel_to_rad_per_sec;
    auto command_int32 = static_cast<int32_t>(velocity_rpm);
    auto cmd = pack_int32_into_can_obj(command_int32);
    auto result = canopen_commander->Write(cobid, cmd);
    return result == CANopen::ESUCCESS;
}

bool BIGSSMaxonCAN::CSP_command(const double position_rad)
{
    // print warning not implemented
    std::cout << "BIGSSMaxonCAN: CSP_command not implemented yet." << std::endl;
    return false;
}

bool BIGSSMaxonCAN::CST_command(const double torque)
{
    // print warning not implemented
    std::cout << "BIGSSMaxonCAN: CST_command not implemented yet." << std::endl;
    return false;
}

bool BIGSSMaxonCAN::read_and_parse_known_data()
{
    CiA301::COBID cobid;
    CiA301::Object object;
    canopen_reader->Read(cobid, object);

    // if object is empty, must be a transmit command so just return true
    if (object.data.data.empty())
        return true;

    // make sure cobid is in our map
    if (m_cobid_map.find("read_pos_vel") != m_cobid_map.end() && cobid == m_cobid_map.at("read_pos_vel"))
    {
        // first 4 bytes are position, next 4 bytes are velocity, both in little endian
        // position is in rotations, velocity is in 0.1 RPM
        m_position_rad = static_cast<double>(object.data.data[0] | (object.data.data[1] << 8) | (object.data.data[2] << 16) | (object.data.data[3] << 24)) * m_encoder_to_rad;
        m_velocity_rad_per_sec = static_cast<double>(object.data.data[4] | (object.data.data[5] << 8) | (object.data.data[6] << 16) | (object.data.data[7] << 24)) * m_maxonvel_to_rad_per_sec;
        return true;
    }
    else if (m_cobid_map.find("read_cur_tor") != m_cobid_map.end() && cobid == m_cobid_map.at("read_cur_tor"))
    {
        // first 4 bytes is current, next 2 bytes is torque, both in little endian
        // current is supplied in milliamps
        m_current_amp = static_cast<double>(object.data.data[0] | (object.data.data[1] << 8 | (object.data.data[2] << 16) | (object.data.data[3] << 24))) * M_MILLIX_TO_X;
        // torque given as 0.1% of motor rated torque
        m_torque_nm = static_cast<double>(static_cast<int16_t>(object.data.data[4] | (object.data.data[5] << 8))) * 0.001 * m_motor_rated_torque_nm;
        return true;
    }
    else if (m_cobid_map.find("read_stat_op") != m_cobid_map.end() && cobid == m_cobid_map.at("read_stat_op"))
    {
        // first 2 bytes is statusword, next 1 bytes is operation mode, both in little endian
        // see manual for full statusword breakdown. Taking bit 2 for operation enabled bit 3 for fault bit 5 for quick stop bit bit 15 for homed bit
        m_is_enabled = (object.data.data[0] & 0x04) != 0;       // bit 2
        m_is_faulted = (object.data.data[0] & 0x08) != 0;       // bit 3
        m_is_quick_stopped = (object.data.data[0] & 0x20) != 0; // bit 5
        // bit 15 of the status word is actually bit 7 of the second byte
        if (m_needs_homing)
            m_is_homed = (object.data.data[1] & 0x80) != 0; // bit 15 of status word, bit 7 of second byte
        else
            m_is_homed = true;

        m_operating_mode = static_cast<SupportedOperatingModes>(object.data.data[2]);
        return true;
    }
    else
    {
        return false;
    }
    return true;
}

bool BIGSSMaxonCAN::send_transmit_requests()
{
    CiA301::COBID cobid;
    if (extract_cobid_if_supported("read_pos_vel", cobid))
    {
        canopen_rtr->WriteRTR(cobid);
        // std::this_thread::sleep_for(std::chrono::milliseconds(1)); // TODO: can we get better implementation?
    }
    if (extract_cobid_if_supported("read_cur_tor", cobid))
    {
        canopen_rtr->WriteRTR(cobid);
        // std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    if (extract_cobid_if_supported("read_stat_op", cobid))
        canopen_rtr->WriteRTR(cobid);
    return true;
}

bool BIGSSMaxonCAN::perform_homing_sequence()
{
    // if m_homing_sequence is empty we assume that homing is not required for this actuator and return true
    if (m_homing_sequence.empty())
    {
        std::cerr << "BIGSSMaxonCAN: homing sequence is empty. Will not perform homing." << std::endl;
        return false;
    }
    // if we are already homed we will not home again
    if (m_is_homed)
    {
        std::cerr << "BIGSSMaxonCAN: already homed. Will not perform homing." << std::endl;
        return false;
    }
    CiA301::COBID cobid;
    if (!extract_cobid_if_supported("homing", cobid))
    {
        std::cerr << "BIGSSMaxonCAN: homing cobid not found. Will not perform homing." << std::endl;
        return false;
    }

    // perform homing sequence
    auto result = write_can_sequence(cobid, m_homing_sequence);
    // if (result)
    // m_is_homed = true;
    return result;
}

bool BIGSSMaxonCAN::check_is_homed_before_moving()
{
    if (!m_is_homed)
        std::cerr << "BIGSSMaxonCAN: not homed. Will not perform command. Either perform homing or set m_needs_homing to false" << std::endl;
    return m_is_homed;
}

bool BIGSSMaxonCAN::halt()
{
    if (!m_is_enabled)
    {
        std::cerr << "BIGSSMaxonCAN: not enabled. Will not execute command." << std::endl;
        return false;
    }

    // if you are in one of the profile modes, send send the halt command
    if (m_operating_mode == SupportedOperatingModes::PPM || m_operating_mode == SupportedOperatingModes::PVM || m_operating_mode == SupportedOperatingModes::HMM)
    {
        CiA301::COBID cobid;
        if (!extract_cobid_if_supported("enable_state", cobid))
            return false;
        auto cmd = CiA301::Object({0x0f, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00});
        auto result = canopen_commander->Write(cobid, cmd);
        return result == CANopen::ESUCCESS;
    }
    else
    {
        // FUTURE: implement an equivalent halting behavior for the other modes
        std::cerr << "BIGSSMaxonCAN: halt only works for profile modes. Will not execute command." << std::endl;
        return false;
    }
}
