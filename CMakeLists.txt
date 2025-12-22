cmake_minimum_required(VERSION 3.0.2)
project(prost_ros)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)

add_service_files(
  FILES
  StartPlanning.srv
  SubmitObservation.srv
)

add_message_files(
  FILES
  KeyValue.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/prost_bridge.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
