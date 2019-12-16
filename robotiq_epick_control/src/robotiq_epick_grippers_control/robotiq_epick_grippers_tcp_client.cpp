#include "robotiq_epick_grippers_control/robotiq_epick_grippers_tcp_client.h"
#include <bitset>
#include "robotiq_modbus_tcp/tcp_manager.h"
#include <ros/ros.h>
// See Robotiq's documentation for the register mapping

// An effort to keep the lines less than 100 char long
namespace robotiq_vacuum_grippers_control
{

RobotiqEpickTcpClient::RobotiqEpickTcpClient(robotiq_modbus_tcp::TcpManager& manager)
  : manager_(manager)
{}

/*
  See support.robotiq.com -> manual for the register output meanings
*/
void RobotiqEpickTcpClient::writeOutputs(const GripperOutput& output)
{
  // array containing all 6 output registers
  std::array<uint8_t, 6> map = {0, 0, 0, 0, 0, 0};

  // Pack the Action Request register byte
  map[0] = ((output.rACT & 0x1) |
            ((output.rMOD << 0x1) & 0x2) |
            ((output.rGTO << 0x3) & 0x8) |
            ((output.rATR << 0x4) & 0x10));
  // registers 1 & 2 reserved by Robotiq
  map[3] = output.rPR;
  map[4] = output.rSP;
  map[5] = output.rFR;

  if(manager_.write<6, 0>(map) == -1){
    ROS_WARN("FAILED TO WRITE");
  }
}

RobotiqEpickTcpClient::GripperInput RobotiqEpickTcpClient::readInputs() const
{
  std::array<uint8_t, 6> map = {0, 0, 0, 0, 0, 0};

  if(manager_.readInput<6,0>(map) == -1){
    ROS_WARN("FAILED TO READ INPUTS");
    // TODO: throw error
   }

  // Decode Input Registers
  GripperInput input;
  input.gACT = map[0] & 0x1;
  input.gMOD = (map[0] >> 0x1) & 0x3;
  input.gGTO = (map[0] >> 0x3) & 0x1;
  input.gSTA = (map[0] >> 0x4) & 0x3;
  input.gOBJ = (map[0] >> 0x6) & 0x3;
  input.gVAS = map[1] & 0x3;
  input.gFLT = map[2] & 0xF;
  input.gPR  = map[3];
  input.gPO  = map[4];

  return input;
}

RobotiqEpickTcpClient::GripperOutput RobotiqEpickTcpClient::readOutputs() const
{
  std::array<uint8_t, 6> map = {0, 0, 0, 0, 0, 0};

  if(manager_.readOutput<6, 0>(map) == -1){
    ROS_WARN("FAILED TO READ INPUTS");
    // TODO: throw error
   }


  GripperOutput output;
  output.rACT = map[0] & 1;
  output.rMOD = (map[0] >> 0x1) & 0x1;
  output.rGTO = (map[0] >> 0x3) & 0x1;
  output.rATR = (map[0] >> 0x4) & 0x1;
  output.rPR  = map[3];
  output.rSP  = map[4];
  output.rFR  = map[5];

  return output;
}
} // end of robotiq_vacuum_grippers_control namespace
