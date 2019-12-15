#include "robotiq_epick_interface/epick_gripper_interface.h"

#include <ros/spinner.h>
#include <ros/callback_queue.h>
#include <algorithm>    // std::min std::max

#include <boost/bind.hpp>

namespace epick_interface
{
  const float min_pressure = 0;
  const float max_pressure = 0.087;
  const float m_pressure = (max_pressure - min_pressure)/(13. - 230.);
  const float offset_pressure = 230.;

  inline float clip(float value, const float min, const float max){
    value = std::min(value, max);
    return std::max(value, min);
  }

  inline uint8_t intPressure(const float& press){
    return static_cast<uint8_t>(round(clip((13.-230.)/0.087 * press + 230., 0, 255)));
  }

  inline float getPressureFromInt(const uint8_t gP){
    float pressure = m_pressure*(gP - offset_pressure);
    pressure = std::max(pressure, min_pressure);
    pressure = std::min(pressure, max_pressure);
    return pressure;
  }

  inline uint8_t gripTimeoutToInt(const float& rdel){
    return static_cast<int>(round(clip(255./(0.1-0.013) * (rdel-0.013), 0, 255)));
  }

  inline uint8_t minRelPressToInt(const float& mrprl){
    return static_cast<int>(round(clip(255./(100.-30.) * (mrprl-30.), 0, 255)));
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

  uint8_t EpickGripper::getFaultStatus()
  {
    return cur_status_.gFLT;
  }

  float EpickGripper::getPressure()
  {
    return getPressureFromInt(cur_status_.gPO);
  }

  float EpickGripper::getRequestedPressure()
  {
    return getPressureFromInt(cur_status_.gPR);
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
    cmd_pub_.publish(cmd);
    return;
  }

  bool EpickGripper::activate(const ros::Duration& timeout)
  {
    GripperOutput cmd;
    cmd.rACT = 1;
    cmd.rMOD = 0;
    cmd.rGTO = 1;
    cmd.rPR = 0;
    cmd.rSP = 150;
    cmd.rFR = 50;
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

  bool EpickGripper::goTo(const float& press, const float& rdel, const float& mrprl,
                          const bool block, const ros::Duration& timeout){
    GripperOutput cmd;
    cmd.rACT = 1;
    cmd.rGTO = 1;
    cmd.rPR = intPressure(press);
    cmd.rSP = gripTimeoutToInt(rdel);
    cmd.rFR = minRelPressToInt(mrprl);
    cmd_pub_.publish(cmd);
    ros::Duration(0.1).sleep();

    if(block){
      if(waitUntilMoving(timeout)){
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

  bool EpickGripper::open(const float& rdel, const float& mrprl, const float& block, const ros::Duration& timeout)
  {
    if(isOpened())
      return true;
    return goTo(1.0, rdel, mrprl, block, timeout);
  }

  bool EpickGripper::close(const float& rdel, const float& mrprl, const float& block, const ros::Duration& timeout)
  {
    if(isClosed())
      return true;
    return goTo(1.0, rdel, mrprl, block, timeout);
  }

}
