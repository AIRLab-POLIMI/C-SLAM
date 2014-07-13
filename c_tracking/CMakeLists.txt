cmake_minimum_required(VERSION 2.8.3)

find_package(catkin REQUIRED COMPONENTS roscpp cv_bridge image_transport)
find_package(OpenCV REQUIRED)

project(c_tracking)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

#build the recognition node
add_executable(cognitive_tracker src/CMT.cpp
                                src/main.cpp)

#link libriaries
target_link_libraries(cognitive_tracker ${catkin_LIBRARIES})