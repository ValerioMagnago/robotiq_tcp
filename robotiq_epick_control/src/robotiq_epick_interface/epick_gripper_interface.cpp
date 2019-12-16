#include "robotiq_epick_interface/epick_gripper_interface.h"

#include <ros/spinner.h>
#include <ros/callback_queue.h>
#include <algorithm>    // std::min std::max

#include <boost/bind.hpp>

namespace epick_interface
{
  inline float clip(float value, const float min, const float max){
    value = std::min(value, max);
    return std::max(value, min);
  }

  inline uint8_t toUint8(float value){
    return static_cast<uint8_t>(clip(round(value), 0., 255.));
  }

  inline uint8_t kpaToUint8(const float& relative_KPa){
    return toUint8(relative_KPa + 100.);
  }

  inline float uintToKpa(const uint8_t gP){
    return static_cast<float>(gP - 100); // KPa
  }

  inline uint8_t secToUint8(const float& rdel_sec){
    return toUint8(rdel_sec*10);
  }

  EpickGripper::EpickGripper(ros::NodeHandle& nh): nh_(nh)
  {
    loadParams();
    initPublishers();
    initSubscribers();
  }

  bool EpickGripper::waitForConnection(const ros::Duration& timeout)
  {
    ros::Duration(0.1).sleep();
    ros::Rate r(30);
    const ros::Time start_time = ros::Time::now();
    while(ros::ok()){
      if(ros::Time::now() - start_time > timeout){
        return false;
      }else{
        return true;
      }
      r.sleep();
    }
    return false;
  }

  bool EpickGripper::isReady()
  {
    return cur_status_.gSTA == 3 && cur_status_.gACT == 1;
  }

  bool EpickGripper::isReset()
  {
    return  cur_status_.gSTA == 0 || cur_status_.gACT == 0;
  }

  bool EpickGripper::isMoving()
  {
    return cur_status_.gGTO == 1 && cur_status_.gOBJ == 0;
  }

  bool EpickGripper::isStopped()
  {
    return cur_status_.gOBJ != 0;
  }

  bool EpickGripper::objectDetected()
  {
    return cur_status_.gOBJ == 1 || cur_status_.gOBJ == 2;
  }

  EpickGripper::FaultStatus EpickGripper::getFaultStatus()
  {
    return static_cast<EpickGripper::FaultStatus>(cur_status_.gFLT);
  }

  EpickGripper::ActuatorStatus EpickGripper::getActuatorStatus()
  {
    return static_cast<EpickGripper::ActuatorStatus>(cur_status_.gVAS);
  }

  float EpickGripper::getPressure()
  {
    return uintToKpa(cur_status_.gPO);
  }

  float EpickGripper::getRequestedPressure()
  {
    return uintToKpa(cur_status_.gPR);
  }

  bool EpickGripper::isClosed()
  {
    return cur_status_.gPO >= 230;
  }

  bool EpickGripper::isOpened()
  {
    return cur_status_.gPO <= 13;
  }

  bool EpickGripper::waitUntilStopped(const ros::Duration& timeout)
  {
    ros::Rate r(30);
    const ros::Time start_time = ros::Time::now();
    while(ros::ok()){
      if(ros::Time::now() - start_time > timeout){
        return false;
      }

      if(isStopped()){
        return true;
      }
    }
    return false;
  }

 bool EpickGripper::waitUntilMoving(const ros::Duration& timeout)
  {
    ros::Rate r(30);
    const ros::Time start_time = ros::Time::now();
    while(ros::ok()){
      if(ros::Time::now() - start_time > timeout){
        return false;
      }

      if(isStopped()){
        return true;
      }
    }
    return false;
  }

  void EpickGripper::reset()
  {
    GripperOutput cmd;
    cmd.rACT = 0;
    cmd.rGTO = 0;
    cmd_pub_.publish(cmd);
    return;
  }

  bool EpickGripper::activate(const ros::Duration& timeout)
  {
    GripperOutput cmd;
    cmd.rACT = 1;
    cmd.rMOD = 0;
    cmd.rGTO = 0;
    cmd.rPR = 0;
    cmd.rSP = 0;
    cmd.rFR = 0;
    cmd_pub_.publish(cmd);
    ros::Rate r(30);
    ros::Time start_time = ros::Time::now();
    while(ros::ok()){
      if(ros::Time::now() - start_time > timeout){
        return false;
      }

      if(isReady()){
        return true;
      }
      ros::spinOnce();
      r.sleep();
    }
    return false;
  }

  void EpickGripper::loadParams()
  {

    cmd_pub_topic_name_   = "output";
    state_sub_topic_name_ = "input";
    return;
  }

  void EpickGripper::initSubscribers()
  {
    state_sub_ = nh_.subscribe(state_sub_topic_name_, 1,
                               &EpickGripper::statusCb, this);
  }

  void EpickGripper::initPublishers()
  {
    cmd_pub_ = nh_.advertise<GripperOutput>(cmd_pub_topic_name_, 1);
  }

  void EpickGripper::statusCb(GripperInput msg)
  {
    cur_status_ = msg;
  }

  void EpickGripper::autoRelease()
  {
    GripperOutput cmd;
    cmd.rACT = 1;
    cmd.rATR = 1;
    cmd_pub_.publish(cmd);
  }

  bool EpickGripper::goTo(const float& max_press_kpa, const float& action_timeout_sec,
                  const float& min_press_kpa, const bool block, const ros::Duration& timeout
                   ){
    // REGULATE VACUUM PRESSURE REQUEST
    const float staurated_min = std::min(min_press_kpa, 0.f);
    GripperOutput cmd;
    cmd.rACT = 1;
    cmd.rMOD = 1;
    cmd.rGTO = 1;
    cmd.rPR = kpaToUint8(max_press_kpa);
    cmd.rSP = secToUint8(action_timeout_sec);
    cmd.rFR = kpaToUint8(staurated_min);
    //cmd.rATR = open_valve_;
    cmd_pub_.publish(cmd);

    ROS_WARN_STREAM("GRIP pressure uint  " << static_cast<int>(cmd.rPR));
    ros::Duration(0.1).sleep();

    if(block){
      if(!waitUntilMoving(timeout)){
          return false;
      }
      return waitUntilStopped(timeout);
    }
    return true;
  }

  bool EpickGripper::stop(bool block, const ros::Duration& timeout)
  {
    GripperOutput cmd;
    cmd.rACT = 1;
    cmd.rGTO = 0;
    cmd_pub_.publish(cmd);
    ros::Duration(0.1).sleep();
    if(block)
      return waitUntilStopped(timeout);
    return true;
  }

  bool EpickGripper::drop(const float& max_press_kpa, const float& action_timeout_sec,
                  const float& min_press_kpa, const bool block, const ros::Duration& timeout)
  {
    if(!isReady()){
      ROS_WARN("Impossible to actuate epick while it is not in ready state!");
      return false;
    }
    if(isOpened())
      return true;
    float gripping_press = std::max(0.f, max_press_kpa); // we need positive pressure to release
    return goTo(gripping_press, action_timeout_sec, min_press_kpa, block, timeout);
  }

  bool EpickGripper::grasp(const float& max_press_kpa, const float& action_timeout_sec,
                  const float& min_press_kpa, const bool block, const ros::Duration& timeout)
  {
    if(!isReady()){
     ROS_WARN("Impossible to actuate epick while it is not in ready state!");
      return false;
    }
    if(isClosed())
      return true;
    ROS_WARN_STREAM("GRIP pressure  " << max_press_kpa);
    float gripping_press = std::min(-10.f, max_press_kpa); // we need negative pressure to grasp
    return goTo(gripping_press, action_timeout_sec, min_press_kpa, block, timeout);
  }

}
