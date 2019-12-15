#include "ros/ros.h"

#include <boost/bind.hpp>
#include <boost/ref.hpp>

#include "robotiq_epick_grippers_control/robotiq_epick_grippers_tcp_client.h"
#include "robotiq_epick_control/RobotiqVacuumGrippers_robot_input.h"
#include "robotiq_modbus_tcp/tcp_manager.h"



void changeCallback(robotiq_vacuum_grippers_control::RobotiqEpickTcpClient& client,
                    const robotiq_vacuum_grippers_control::RobotiqEpickTcpClient::GripperOutput::ConstPtr& msg)
{
  client.writeOutputs(*msg);
}


int main(int argc, char** argv)
{
  using robotiq_modbus_tcp::TcpManager;
  using robotiq_vacuum_grippers_control::RobotiqEpickTcpClient;

  typedef RobotiqEpickTcpClient::GripperOutput GripperOutput;
  typedef RobotiqEpickTcpClient::GripperInput GripperInput;

  ros::init(argc, argv, "robotiq_epick_driver_node");

  ros::NodeHandle nh ("~");

  // Parameter names
  std::string ip;
  int port;
  bool activate;

  nh.param<std::string>("epick_ip", ip, "10.30.99.123");
  nh.param<int>("port", port, 502);
  nh.param<bool>("activate", activate, true);

  // Start ethercat manager
  TcpManager manager(ip, port);

  // register client
  RobotiqEpickTcpClient client(manager);

  // conditionally activate the gripper
  if (activate)
  {
    // Check to see if resetting is required? Or always reset?
    GripperOutput out;
    out.rACT = 0x1;
    client.writeOutputs(out);
  }

  // Sorry for the indentation, trying to keep it under 100 chars
  ros::Subscriber sub =
        nh.subscribe<GripperOutput>("output", 1,
                                    boost::bind(changeCallback, boost::ref(client), _1));

  ros::Publisher pub = nh.advertise<GripperInput>("input", 100);

  ros::Rate rate(10); // 10 Hz

  while (ros::ok())
  {
    RobotiqEpickTcpClient::GripperInput input = client.readInputs();
    pub.publish(input);
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
