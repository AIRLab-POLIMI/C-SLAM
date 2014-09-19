set(CMAKE_CXX_FLAGS "-g -std=c++11 ${OTHERS_DEFINE}")

include_directories(
  ${ROAMFREE_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIR}
  ${Boost_INCLUDE_DIRS}  
)

add_executable(full_slam_imu
	FullSlamImu.cpp
)

target_link_libraries(full_slam_imu
  ${catkin_LIBRARIES}
  ${ROAMFREE_LIBRARIES}  
)