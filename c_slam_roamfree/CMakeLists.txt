cmake_minimum_required(VERSION 2.8.3)
project(c_slam_roamfree)

find_package(catkin REQUIRED COMPONENTS roscpp 
                                        tf 
                                        tf_conversions 
                                        c_slam_msgs
                                        roamfree)
                                        
find_package(OpenCV REQUIRED)
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

add_executable(c_localization
	           src/FullSlamImu.cpp
	           src/ImuHandler.cpp
	           src/TracksHandler.cpp)

target_link_libraries(c_localization
                      ${catkin_LIBRARIES}
                      ${ROAMFREE_LIBRARIES})

add_executable(circular_test
               src/CircularTest.cpp)
               
add_executable(displayer
               src/Displayer.cpp)               

target_link_libraries(circular_test ${catkin_LIBRARIES})
target_link_libraries(displayer ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})


add_dependencies(c_localization ${catkin_EXPORTED_TARGETS})
add_dependencies(circular_test ${catkin_EXPORTED_TARGETS})
add_dependencies(displayer ${catkin_EXPORTED_TARGETS})

