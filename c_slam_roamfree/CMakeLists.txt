cmake_minimum_required(VERSION 2.8.3)
project(c_slam_roamfree)

find_package(catkin REQUIRED COMPONENTS roscpp 
                                        tf 
                                        tf_conversions 
                                        c_slam_msgs
                                        roamfree)

#this is needed if no system level FindEigen3.cmake is available
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake/modules)
                                        
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
		            ${EIGEN3_INCLUDE_DIR}
		            ${Boost_INCLUDE_DIRS}  
)

add_executable(c_localization
	           src/FullSlamImu.cpp
	           src/ImuHandler.cpp)

add_executable(c_localization_rectangles
	           src/FullSlamImu_rectangles.cpp
	           src/RectangleHandler.cpp
	           src/ImuHandler.cpp)

target_link_libraries(c_localization
                      ${catkin_LIBRARIES})

target_link_libraries(c_localization_rectangles
                      ${catkin_LIBRARIES})

add_executable(circular_test
               src/TestPublisher.cpp
               src/CircularTest.cpp)
               
add_executable(firefly_test
               src/TestPublisher.cpp
               src/FireflyTest.cpp)
               
add_executable(displayer
               src/Displayer.cpp)               

target_link_libraries(circular_test ${catkin_LIBRARIES})
target_link_libraries(firefly_test ${catkin_LIBRARIES})
target_link_libraries(displayer ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})


add_dependencies(c_localization ${catkin_EXPORTED_TARGETS})
add_dependencies(c_localization_rectangles ${catkin_EXPORTED_TARGETS})
add_dependencies(circular_test ${catkin_EXPORTED_TARGETS})
add_dependencies(displayer ${catkin_EXPORTED_TARGETS})

