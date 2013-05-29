cmake_minimum_required(VERSION 2.4.6)
find_package(catkin)
project(c_vision)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

find_package(OpenCV REQUIRED)

#common commands for building c++ executables and libraries
add_executable(example src/main.cpp
			src/Callbacks.cpp
			src/BorderDetection.cpp  
			src/CornerDetector.cpp  
			src/LineDetector.cpp)

target_link_libraries(example ${OpenCV_LIBS})

