# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(robotiq_epick_control)

## Compile as C++17, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)


find_package(catkin REQUIRED
  dynamic_reconfigure
  message_generation
  roscpp
  rospy
)

find_package(Boost REQUIRED COMPONENTS system)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

################################################
## Declare ROS messages, services and actions ##
################################################
add_message_files(
  FILES
  RobotiqVacuumGrippers_robot_input.msg
  RobotiqVacuumGrippers_robot_output.msg
)

catkin_python_setup()
generate_messages(
  DEPENDENCIES
  std_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################
generate_dynamic_reconfigure_options(
   cfg/Epick.cfg
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
    CATKIN_DEPENDS
      message_runtime
      roscpp
      rospy
      modbus
    INCLUDE_DIRS
      include
)

set(CMAKE_CXX_IMPLICIT_LINK_DIRECTORIES /usr/local/lib ${CMAKE_CXX_IMPLICIT_LINK_DIRECTORIES})

include_directories(
  include
  /usr/local/include
  ${Boost_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)


add_executable(robotiq_epick_tcp_node
  src/robotiq_epick_grippers_control/robotiq_epick_grippers_tcp_node.cpp
  src/robotiq_epick_grippers_control/robotiq_epick_grippers_tcp_client.cpp
)

target_link_libraries(robotiq_epick_tcp_node
  ${catkin_LIBRARIES}
  modbus
)

add_dependencies(robotiq_epick_tcp_node
  robotiq_epick_control_generate_messages_cpp
)


add_executable(robotiq_epick_server_node
  src/robotiq_epick_interface/epick_server_node.cpp
  src/robotiq_epick_interface/epick_server.cpp
  src/robotiq_epick_interface/epick_gripper_interface.cpp
)

target_link_libraries(robotiq_epick_server_node
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES}
)

add_dependencies(robotiq_epick_server_node
  robotiq_epick_control_generate_messages_cpp
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/groovy/api/catkin/html/adv_user_guide/variables.html

install(PROGRAMS nodes/Robotiq2FGripperSimpleController.py nodes/Robotiq2FGripperStatusListener.py
    nodes/Robotiq2FGripperTcpNode.py nodes/Robotiq2FGripperRtuNode.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
