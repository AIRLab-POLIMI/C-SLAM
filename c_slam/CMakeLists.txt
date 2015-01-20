cmake_minimum_required(VERSION 2.4.6)

find_package(catkin)
project(c_slam)

catkin_package()

#install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)