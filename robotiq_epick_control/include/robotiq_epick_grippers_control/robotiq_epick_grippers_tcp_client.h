#pragma once

#include <robotiq_epick_control/RobotiqVacuumGrippers_robot_output.h>
#include <robotiq_epick_control/RobotiqVacuumGrippers_robot_input.h>

// Forward declaration of EtherCatManager
namespace robotiq_modbus_tcp
{
  class TcpManager;
}

namespace robotiq_vacuum_grippers_control
{

/**
 * \brief This class provides a client for the EtherCAT manager object that
 *        can translate robot input/output messages and translate them to
 *        the underlying IO Map.
 */

class RobotiqEpickTcpClient
{
public:
  typedef robotiq_epick_control::RobotiqVacuumGrippers_robot_output GripperOutput;
  typedef robotiq_epick_control::RobotiqVacuumGrippers_robot_input GripperInput;

  /**
   * \brief Constructs a control interface to a epick Robotiq gripper on
   *        the given LAN.
   *
   * @param[in] manager The interface to an TCP network that the gripper
   *                    is connected to.
   *
   */
   RobotiqEpickTcpClient(robotiq_modbus_tcp::TcpManager& manager);

  /**
   * \brief Write the given set of control flags to the memory of the gripper
   *
   * @param[in] output The set of output-register values to write to the gripper
   */
  void writeOutputs(const GripperOutput& output);

  /**
   * \brief Reads set of input-register values from the gripper.
   * \return The gripper input registers as read from the controller IOMap
   */
  GripperInput readInputs() const;

  /**
   * \brief Reads set of output-register values from the gripper.
   * \return The gripper output registers as read from the controller IOMap
   */
  GripperOutput readOutputs() const;

private:
  robotiq_modbus_tcp::TcpManager& manager_;
};

}
