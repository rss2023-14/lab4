cmake_minimum_required(VERSION 2.8.3)
project(wall_follower)

find_package(catkin REQUIRED COMPONENTS
  ackermann_msgs
  geometry_msgs
  rospy
  sensor_msgs
  std_msgs
  visualization_msgs
)

if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch)
endif()

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  src/visualization_tools.py
  src/viz_example.py
  src/wall_follower.py
  src/safety_controller.py
  src/metrics.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
