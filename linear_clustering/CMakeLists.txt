cmake_minimum_required(VERSION 2.8.3)

find_package(catkin REQUIRED)
find_package(OpenCV REQUIRED)

project(linear_clustering)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

#build the recognition node
add_library(linear_clustering 	src/Cluster.cpp  
				src/ClusterFilter.cpp  
				src/LineDetector.cpp)

#link libriaries
target_link_libraries(linear_clustering ${catkin_LIBRARIES})
