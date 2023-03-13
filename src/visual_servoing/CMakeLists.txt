cmake_minimum_required(VERSION 2.8.3)
project(visual_servoing)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  ackermann_msgs
  visualization_msgs
  message_generation
)

add_message_files(
  FILES
  ConeLocationPixel.msg
  ConeLocation.msg
  ParkingError.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
  ackermann_msgs
  visualization_msgs
)

if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch)
endif()

catkin_package(CATKIN_DEPENDS
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  ackermann_msgs
  visualization_msgs
  message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/cone_detector.py
  scripts/cone_sim_marker.py
  scripts/homography_transformer.py
  scripts/parking_controller.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
