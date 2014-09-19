cmake_minimum_required(VERSION 2.8.3)
project(roamfree_c_slam)

find_package(catkin REQUIRED COMPONENTS roscpp tf)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake/modules)

find_package(ROAMFREE REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)

set(Boost_USE_STATIC_LIBS OFF)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)

catkin_package()

set(CMAKE_CXX_FLAGS "-g -std=c++11 ${OTHERS_DEFINE}")

include_directories(include 
                    ${catkin_INCLUDE_DIRS}
		    ${ROAMFREE_INCLUDE_DIR}
		    ${EIGEN3_INCLUDE_DIR}
		    ${Boost_INCLUDE_DIRS}  
)

add_executable(full_slam_imu
	src/FullSlamImu.cpp
)

target_link_libraries(full_slam_imu
  ${catkin_LIBRARIES}
  ${ROAMFREE_LIBRARIES}  
)