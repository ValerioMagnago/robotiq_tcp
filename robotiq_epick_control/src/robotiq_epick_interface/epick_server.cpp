#include "robotiq_epick_interface/epick_server.h"

#include <algorithm>
#include <assert.h>

#include <std_msgs/Bool.h>


namespace epick_interface
{

  EpickServer::EpickServer(ros::NodeHandle& nh):
    nh_(nh), gripper_(nh)
  {
    initialize();
  }

  EpickServer::~EpickServer()
  {
    // Custom Callback Queue
    queue_.clear();
    queue_.disable();
    nh_.shutdown();
    callback_queue_thread_.join();
  }

  // Load the controller
  void EpickServer::initialize()
  {
    ROS_INFO_NAMED("epick_server", "Loading params");

    connect_count_ = 0;
    status_ = false;
    pub_topic_name_  = "status";
    srv1_topic_name_ = "on";
    srv2_topic_name_ = "off";

    // Custom Callback Queue
    ros::AdvertiseOptions ao =
      ros::AdvertiseOptions::create<std_msgs::Bool>(
                                                    pub_topic_name_, 1,
                                                    boost::bind(&EpickServer::Connect, this),
                                                    boost::bind(&EpickServer::Disconnect, this),
                                                    ros::VoidPtr(), &queue_);
    pub_ = nh_.advertise(ao);

    // Custom Callback Queue
    ros::AdvertiseServiceOptions aso1 =
      ros::AdvertiseServiceOptions::create<std_srvs::Empty>(
                                                       srv1_topic_name_,
                                                       boost::bind(&EpickServer::OnServiceCallback,
                                                                    this, _1, _2),
                                                       ros::VoidPtr(), &queue_);
    srv1_ = nh_.advertiseService(aso1);

    ros::AdvertiseServiceOptions aso2 =
      ros::AdvertiseServiceOptions::create<std_srvs::Empty>(
                                                      srv2_topic_name_,
                                                      boost::bind(&EpickServer::OffServiceCallback,
                                                             this, _1, _2),
                                                      ros::VoidPtr(), &queue_);
    srv2_ = nh_.advertiseService(aso2);


    param_f_ = boost::bind(&EpickServer::paramCb, this, _1, _2);
    param_server_.setCallback(param_f_);

    // Custom Callback Queue
    //callback_queue_thread_ = boost::thread( boost::bind( &EpickServer::QueueThread,this ) );
  }

  void EpickServer::paramCb(robotiq_epick_control::EpickConfig &config, uint32_t level){
    // TODO: need to add some input control!!
    grasp_params_.max_press_kpa = static_cast<float>(config.g_max_press);
    grasp_params_.action_timeout_sec =  static_cast<float>(config.g_action_timeout);
    grasp_params_.min_press_kpa =  static_cast<float>(config.g_min_press);
    grasp_params_.block = true;
    grasp_params_.timeout = ros::Duration(grasp_params_.action_timeout_sec*1.2);

    drop_params_.max_press_kpa = static_cast<float>(config.g_max_press);
    drop_params_.action_timeout_sec =  static_cast<float>(config.g_action_timeout);
    drop_params_.min_press_kpa =  static_cast<float>(config.g_min_press);
    drop_params_.block = true;
    drop_params_.timeout = ros::Duration(drop_params_.action_timeout_sec*1.2);
  }

  bool EpickServer::OnServiceCallback(std_srvs::Empty::Request &req,
                                      std_srvs::Empty::Response &res)
  {
    if(gripper_.isReset()){
      ROS_WARN("Need to reset the gripper before grasping!");
      gripper_.reset();

      if(!gripper_.activate(ros::Duration(1))){
        ROS_WARN("Impossible to activate the gripper!!");
        return false;
      }
    }

    ROS_INFO("vacuum_gripper gripping");
    return gripper_.grasp(grasp_params_.max_press_kpa, grasp_params_.action_timeout_sec,
        grasp_params_.min_press_kpa, grasp_params_.block, grasp_params_.timeout);

  }
  bool EpickServer::OffServiceCallback(std_srvs::Empty::Request &req,
                                       std_srvs::Empty::Response &res)
  {
    if(gripper_.isReset()){
      ROS_WARN("Need to reset the gripper before dropping!");
      gripper_.reset();

      if(!gripper_.activate(ros::Duration(1))){
        ROS_WARN("Impossible to activate the gripper!!");
        return false;
      }
    }

    ROS_INFO("vacuum_gripper dropping");
    return gripper_.drop(drop_params_.max_press_kpa, drop_params_.action_timeout_sec,
        drop_params_.min_press_kpa, drop_params_.block, drop_params_.timeout);

  }
  // Update the controller
  void EpickServer::Publish()
  {
    std_msgs::Bool grasping_msg;
    grasping_msg.data = gripper_.objectDetected();
    pub_.publish(grasping_msg);
  }

  // Custom Callback Queue
  // custom callback queue thread
  void EpickServer::QueueThread()
  {
    static const double timeout = 0.01;
    ros::Rate r(30);
    while (nh_.ok()){
      r.sleep();
      if(static_cast<uint8_t>(gripper_.getFaultStatus())>=0xA){
        // Major faults (0xA <= gFLT <= 0xF)
        switch(gripper_.getFaultStatus()){
        case EpickGripper::FaultStatus::UNDER_VOLTAGE:
          ROS_WARN_STREAM("Resetting Epick Major faults: \n\t under minimum operating voltage");
          break;
        case EpickGripper::FaultStatus::PROGRESS_AUTOM_RELEASE:
          ROS_WARN_STREAM("Resetting Epick Major faults: \n\t Automatic release in progress (Vacuum/pressure detected)");
          break;
        case EpickGripper::FaultStatus::INTERNAL_FAULT:
          ROS_WARN_STREAM("Resetting Epick Major faults: \n\t Internal fault; contact support@robotiq.com.");
          break;

        case EpickGripper::FaultStatus::END_AUTOM_RELEASE:
          ROS_WARN_STREAM("Resetting Epick Major faults: \n\t  Automatic release completed (Vacuum/pressure not detected)");
          break;
        }
        // Reset is required
        gripper_.reset();
        continue;
      }

      if(static_cast<uint8_t>(gripper_.getFaultStatus())>=0x8){
        // Minor faults (0x8 <= gFLT <= 0x9) (LED continuous red)
        switch(gripper_.getFaultStatus()){
        case EpickGripper::FaultStatus::HIGH_TEMPERATURE:
          ROS_WARN_STREAM("Stalling Epick Minor faults: \n\t Maximum operating temperature exceeded, wait for cool-down.");
          break;
        case EpickGripper::FaultStatus::NO_COMMUNICATION:
          ROS_WARN_STREAM("Stalling Epick Minor faults: \n\t No communication during at least 1 second.");
          break;
        }
        continue;
      }


      // No faults / Priority faults
      queue_.callAvailable(ros::WallDuration(timeout));
      ros::spinOnce(); // Needed to update grasping!!
      Publish(); // Publish gripping status
    }
  }

  // Someone subscribes to me
  void EpickServer::Connect()
  {
    this->connect_count_++;
  }

  // Someone subscribes to me
  void EpickServer::Disconnect()
  {
    this->connect_count_--;
  }

}  // namespace
