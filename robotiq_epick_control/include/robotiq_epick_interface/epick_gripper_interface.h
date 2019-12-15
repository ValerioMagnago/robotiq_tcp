#pragma once

#include <ros/ros.h>
#include <robotiq_epick_control/RobotiqVacuumGrippers_robot_output.h>
#include <robotiq_epick_control/RobotiqVacuumGrippers_robot_input.h>

namespace epick_interface
{

/**
 * \brief This class provides a interface to the Epick driver conntecting to its input and output topics
 */

class EpickGripper
{
public:

  typedef robotiq_epick_control::RobotiqVacuumGrippers_robot_output GripperOutput;
  typedef robotiq_epick_control::RobotiqVacuumGrippers_robot_input  GripperInput;

  enum ActuatorStatus{
    STANBY = 0b00,
    GRIPPING = 0b01,
    PASSIVE_RELEASING = 0b10,
    ACTIVE_RELEASING  = 0b11,
  };

  enum FaultStatus{
    // No fault
    NO_FAULT = 0x0,
    // Priority faults (0x0 < gFLT <= 0x7) (LED is blue)
    ACTION_DELAYED = 0x5,
    POROUS_MATERIAL = 0x3,
    GRIPPING_TIMEOUT  = 0x6,
    ACTIVATION_NOT_SET  = 0x7,
    // Minor faults (0x8 <= gFLT <= 0x9) (LED continuous red)
    HIGH_TEMPERATURE  = 0x8,    
    NO_COMMUNICATION  = 0x9,
    // Major faults (0xA <= gFLT <= 0xF) (LED blinking red/blue) - Reset is required (rising edge on activation bit rACT required)
    UNDER_VOLTAGE = 0xA,
    PROGRESS_AUTOM_RELEASE = 0xB,
    INTERNAL_FAULT = 0xC,
    END_AUTOM_RELEASE = 0xF
  };

  EpickGripper(ros::NodeHandle& nh);
  bool waitForConnection(const ros::Duration& timeout);
  bool isReady();
  bool isReset();
  bool isMoving();
  bool isStopped();
  bool objectDetected();
  FaultStatus    getFaultStatus();
  ActuatorStatus getActuatorStatus();
  float getPressure();
  float getRequestedPressure();
  bool isClosed();
  bool isOpened();
  bool waitUntilStopped(const ros::Duration& timeout);
  bool waitUntilMoving(const ros::Duration& timeout);
  void reset();
  void statusCb(GripperInput msg);
  bool activate(const ros::Duration& timeout);
  void loadParams();
  void initSubscribers();
  void initPublishers();
  void autoRelease();
  bool stop(bool block, const ros::Duration& timeout);

    /**
   * \brief Send a command to the epick gripper driver
   *
   * @param[in] target max vacuum or pressure in KPa in range [0 155].
   *            Positive --> releasing with positive pressure
   *            Negative --> generate vacuum for gripping obj
   * @param[in] Time window prior to a gripping error. The vacuum
   *            generator will stop after the timeout period.
   * @param[in] minimum acceptable vacuum/pressure on the workpiece in range [-100,0]. 
   *             Once the object is detected, the vacuum generator will keep 
   *             the vacuum level in between the minimum and maximum vacuum level.
   *            
   * \return success of the operation
   */  
  bool drop(const float& max_press_kpa, const float& action_timeout_sec,
                  const float& min_press_kpa, const bool block, const ros::Duration& timeout);

    /**
   * \brief Send a command to the epick gripper driver
   *
   * @param[in] target max vacuum or pressure in KPa in range [-100 -10].
   *            Positive --> releasing with positive pressure
   *            Negative --> generate vacuum for gripping obj
   * @param[in] Time window prior to a gripping error. The vacuum
   *            generator will stop after the timeout period.
   * @param[in] minimum acceptable vacuum/pressure on the workpiece in range [-100,0]. 
   *             Once the object is detected, the vacuum generator will keep 
   *             the vacuum level in between the minimum and maximum vacuum level.
   *            
   * \return success of the operation
   */
  bool grasp(const float& max_press_kpa, const float& action_timeout_sec,
                  const float& min_press_kpa, const bool block, const ros::Duration& timeout);
  
  /**
   * \brief Send a command to the epick gripper driver
   *
   * @param[in] target max vacuum or pressure in KPa in range [-100 +155].
   *            Positive --> releasing with positive pressure
   *            Negative --> generate vacuum for gripping obj
   * @param[in] Time window prior to a gripping error. The vacuum
   *            generator will stop after the timeout period.
   * @param[in] minimum acceptable vacuum/pressure on the workpiece in range [-100,0]. 
   *             Once the object is detected, the vacuum generator will keep 
   *             the vacuum level in between the minimum and maximum vacuum level.
   *            
   * \return success of the operation
   */
  bool goTo(const float& max_press_kpa, const float& action_timeout_sec,
             const float& min_press_kpa, const bool block, const ros::Duration& timeout);
  


private:
  ros::NodeHandle nh_;
  GripperInput cur_status_;

  ros::Publisher cmd_pub_;
  std::string cmd_pub_topic_name_;

  ros::Subscriber state_sub_;
  std::string state_sub_topic_name_;

};

}
