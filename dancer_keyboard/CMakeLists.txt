cmake_minimum_required(VERSION 2.8.3)
project(dancer_keyboard)
find_package(catkin REQUIRED)

catkin_package(
)
include_directories(
)
catkin_install_python(PROGRAMS
   dancer_keyboard.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
