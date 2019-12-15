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

    // Custom Callback Queue
    //callback_queue_thread_ = boost::thread( boost::bind( &EpickServer::QueueThread,this ) );

  }

  bool EpickServer::OnServiceCallback(std_srvs::Empty::Request &req,
                                      std_srvs::Empty::Response &res)
  {
    if (status_) {
      ROS_WARN_NAMED("vacuum_gripper", "gazebo_ros_vacuum_gripper: already status is 'on'");
    } else {
      status_ = true;
      if(gripper_.isReset()){
        gripper_.reset();
        gripper_.activate(ros::Duration(1));
      }
      ROS_INFO_NAMED("vacuum_gripper", "gazebo_ros_vacuum_gripper: status: off -> on");
    }
    return true;
  }
  bool EpickServer::OffServiceCallback(std_srvs::Empty::Request &req,
                                       std_srvs::Empty::Response &res)
  {
    if (status_) {
      status_ = false;
      ROS_INFO_NAMED("vacuum_gripper", "gazebo_ros_vacuum_gripper: status: on -> off");
    } else {
      ROS_WARN_NAMED("vacuum_gripper", "gazebo_ros_vacuum_gripper: already status is 'off'");
    }
    return true;
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
      if(gripper_.getFaultStatus()>=0xA){
        // Major faults (0xA <= gFLT <= 0xF)
        switch(gripper_.getFaultStatus()){
        case 0xA:
          ROS_WARN_STREAM("Resetting Epick Major faults: \n\t under minimum operating voltage");
          break;
        case 0xB:
          ROS_WARN_STREAM("Resetting Epick Major faults: \n\t Automatic release in progress (Vacuum/pressure detected)");
          break;
        case 0xC:
          ROS_WARN_STREAM("Resetting Epick Major faults: \n\t Internal fault; contact support@robotiq.com.");
          break;

        case 0xF:
          ROS_WARN_STREAM("Resetting Epick Major faults: \n\t  Automatic release completed (Vacuum/pressure not detected)");
          break;
        }
        // Reset is required
        gripper_.reset();
        continue;
      }

      if(gripper_.getFaultStatus()>=0x8){
        // Minor faults (0x8 <= gFLT <= 0x9) (LED continuous red)
        switch(gripper_.getFaultStatus()){
        case 0xA:
          ROS_WARN_STREAM("Stalling Epick Minor faults: \n\t Maximum operating temperature exceeded, wait for cool-down.");
          break;
        case 0xF:
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
