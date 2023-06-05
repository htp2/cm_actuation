#include <iostream>
#include <memory>

#include "canopen.hpp"
#include <cm_actuation/BIGSSMaxonCAN.hpp>

BIGSSMaxonCAN::BIGSSMaxonCAN(const std::string &devicename, const std::map<std::string, CiA301::COBID> cobid_map, const SocketCAN::Rate rate)
    : m_cobid_map(cobid_map)
{
    // pass in your own cobid map
    canopen = std::make_unique<CANopen>(devicename, rate, SocketCAN::Loopback::LOOPBACK_OFF);
    canopen->Open();
}

BIGSSMaxonCAN::BIGSSMaxonCAN(const std::string &devicename, const std::string &supported_actuator_name, const SocketCAN::Rate rate)
{
    if (supported_actuator_name == "roll_actuator")
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
            {"quick_stop", 0x203}};
    }
    else
    {
        std::cerr << "BIGSSMaxonCAN: supported_actuator_name not recognized. Will not create." << std::endl;
        return;
    }

    canopen = std::make_unique<CANopen>(devicename, rate, SocketCAN::Loopback::LOOPBACK_OFF);
    canopen->Open();
}

BIGSSMaxonCAN::~BIGSSMaxonCAN()
{
    canopen->Close();
}

bool BIGSSMaxonCAN::write_can_sequence(const CiA301::COBID cobid, const std::vector<CiA301::Object> cmds)
{
    // small helper function to write a sequence of commands to a cobid, returns early with false if any command fails
    // this sends all to the same cobid
    for (auto cmd : cmds)
    {
        auto result = canopen->Write(cobid, cmd);
        if (result != CANopen::ESUCCESS)
        {
            return false;
        }
    }
    return true;
}

bool BIGSSMaxonCAN::write_can_sequence(const std::vector<std::pair<const CiA301::COBID, const std::vector<CiA301::Object>>>& cmds)
{
    // small helper function to write a sequence of commands to a cobid, returns early with false if any command fails
    // this sends to different cobids
    for (auto cmd : cmds)
    {
        for (auto cmd_obj : cmd.second)
        {
            auto result = canopen->Write(cmd.first, cmd_obj);
            if (result != CANopen::ESUCCESS)
            {
                return false;
            }
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

bool BIGSSMaxonCAN::enable_PDO(const CiA301::Node::ID node_id)
{
    // node_id = 0x00 means all nodes, or you can specify a node_id
    CiA301::COBID cobid;
    if (!extract_cobid_if_supported("enable_pdo", cobid))
        return false;
    auto result = canopen->Write(cobid, CiA301::Object({0x01, node_id}));
    return result == CANopen::ESUCCESS;
}

bool BIGSSMaxonCAN::disable_PDO(const CiA301::Node::ID node_id)
{
    // node_id = 0x00 means all nodes, or you can specify a node_id
    CiA301::COBID cobid;
    if (!extract_cobid_if_supported("enable_pdo", cobid))
        return false;
    auto result = canopen->Write(node_id, CiA301::Object({0x80, node_id}));
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
    auto result = canopen->Write(cobid, cmd);
    return result == CANopen::ESUCCESS;
}

bool BIGSSMaxonCAN::perform_canned_homing()
{
    CiA301::COBID cobid;
    if (!extract_cobid_if_supported("homing", cobid))
        return false;
    auto cmd1 = CiA301::Object({0x0f, 0x01, 0x06, 0x17, 0x64, 0x00, 0x00, 0x00}); // set operating mode to homing
    auto cmd2 = CiA301::Object({0x0f, 0x00, 0x06, 0x17, 0x64, 0x00, 0x00, 0x00}); // ready
    auto cmd3 = CiA301::Object({0x1f, 0x00, 0x06, 0x17, 0x64, 0x00, 0x00, 0x00}); // execute (set to 10 RPM)
    auto result = write_can_sequence(cobid, {cmd1, cmd2, cmd3});
    return result;
}

bool BIGSSMaxonCAN::set_quick_stop()
{
    CiA301::COBID cobid;
    if (!extract_cobid_if_supported("quick_stop", cobid))
        return false;
    auto cmd = CiA301::Object({0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00});
    auto result = canopen->Write(cobid, cmd);
    return result == CANopen::ESUCCESS;
}

bool BIGSSMaxonCAN::clear_quick_stop()
{
    CiA301::COBID cobid;
    if (!extract_cobid_if_supported("quick_stop", cobid))
        return false;
    auto cmd = CiA301::Object({0x0f, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}); // 0x01 for safety (halt bit) instead of 0x00 (execute)
    auto result = canopen->Write(cobid, cmd);
    return result == CANopen::ESUCCESS;
}

bool BIGSSMaxonCAN::set_operation_mode(const SupportedOperatingModes mode)
{
    CiA301::COBID cobid;
    if (!extract_cobid_if_supported("op_mode", cobid))
        return false;
    auto cmd = CiA301::Object({0x0f, 0x01, mode, 0x00, 0x00, 0x00, 0x00, 0x00});
    auto result = canopen->Write(cobid, cmd);
    return result == CANopen::ESUCCESS;
}

bool BIGSSMaxonCAN::PVM_command(const double velocity_rad_per_sec)
{
    // TODO: Check op mode / should switch or just error out if not in PVM mode?

    // motor firmware set to accept an int32 for velocity in 0.1*RPM i.e. input of 10 --> 1 RPM
    CiA301::COBID cobid1;
    if (!extract_cobid_if_supported("pvm_target", cobid1))
        return false;
    CiA301::COBID cobid2;
    if (!extract_cobid_if_supported("pvm_exec", cobid2))
        return false;
    auto velocity_rpm = velocity_rad_per_sec * M_RAD_PER_SEC_TO_RPM;
    auto command_int32 = static_cast<int32_t>(velocity_rpm * 10.0);
    auto cmd1 = pack_int32_into_can_obj(command_int32);
    auto cmd2 = CiA301::Object({0x0f, 0x00, 0x03, 0x60, 0x00, 0x00, 0x00, 0x00}); // set velocity target

    auto result = write_can_sequence({{cobid1, {cmd1}}, {cobid2, {cmd2}}});

    return result;
}

bool BIGSSMaxonCAN::PPM_command(const double position_rad)
{
    // print warning not implemented
    std::cout << "BIGSSMaxonCAN: PPM_command not implemented yet." << std::endl;
    return false;
}

bool BIGSSMaxonCAN::CSV_command(const double velocity_rad_per_sec)
{
    CiA301::COBID cobid;
    if (!extract_cobid_if_supported("csv_target", cobid))
        return false;
    auto velocity_rpm = velocity_rad_per_sec * M_RAD_PER_SEC_TO_RPM;
    auto command_int32 = static_cast<int32_t>(velocity_rpm * 10.0);
    auto cmd = pack_int32_into_can_obj(command_int32);
    auto result = canopen->Write(cobid, cmd);
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
