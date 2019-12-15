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

  EpickGripper(ros::NodeHandle& nh);
  bool waitForConnection(const ros::Duration& timeout);
  bool isReady();
  bool isReset();
  bool isMoving();
  bool isStopped();
  bool objectDetected();
  uint8_t getFaultStatus();
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
  bool open(const float& rdel, const float& mrprl,
            const float& block, const ros::Duration& timeout);
  bool close(const float& rdel, const float& mrprl,
             const float& block, const ros::Duration& timeout);
  bool goTo(const float& press, const float& rdel, const float& mrprl,
             const bool block, const ros::Duration& timeout);


private:
  ros::NodeHandle nh_;
  GripperInput cur_status_;

  ros::Publisher cmd_pub_;
  std::string cmd_pub_topic_name_;

  ros::Subscriber state_sub_;
  std::string state_sub_topic_name_;

};

}
