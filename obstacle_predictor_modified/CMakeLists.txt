cmake_minimum_required(VERSION 2.8.3)
project(obstacle_predictor)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_python_setup()

catkin_package(CATKIN_DEPENDS rospy)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  #${OpenCV_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/obstacle_predictor.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
