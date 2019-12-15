#include "ros/ros.h"
#include "robotiq_epick_interface/epick_server.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "epick_server_node");

  ros::NodeHandle nh;

  epick_interface::EpickServer server(nh);
  server.QueueThread();

  return 0;
}
