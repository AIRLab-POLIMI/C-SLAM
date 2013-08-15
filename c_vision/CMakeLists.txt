cmake_minimum_required(VERSION 2.8.3)

find_package(catkin REQUIRED COMPONENTS roscpp cv_bridge image_transport)
find_package(OpenCV REQUIRED)

project(c_vision)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

#build the recognition library
add_library(recognition_library STATIC src/BorderDetection.cpp  
			src/CornerDetector.cpp  
			src/LineDetector.cpp
			src/ClusterFilter.cpp
			src/Cluster.cpp)

#build test executables
add_executable(detector src/detector.cpp src/Callbacks.cpp)
add_executable(comparator src/comparator.cpp src/Callbacks.cpp)

#build the recognition node
add_executable(cognitive_vision src/main.cpp src/Dispatcher.cpp src/CognitiveDetector.cpp)

#link libriaries
target_link_libraries(recognition_library ${OpenCV_LIBRARIES})
target_link_libraries(detector recognition_library)
target_link_libraries(comparator recognition_library)
target_link_libraries(cognitive_vision recognition_library ${catkin_LIBRARIES})
