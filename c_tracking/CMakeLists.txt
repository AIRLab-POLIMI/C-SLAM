cmake_minimum_required(VERSION 2.8.3)

find_package(catkin REQUIRED COMPONENTS roscpp 
                                        cv_bridge 
                                        image_transport
                                        dynamic_reconfigure
                                        c_slam_msgs
                                        tf)
                                        
find_package(OpenCV REQUIRED)

project(c_tracking)

generate_dynamic_reconfigure_options(cfg/Parameters.cfg)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

#build the tracking node
add_executable(c_tracker src/CMT.cpp
                         src/CMTFeatureExtractor.cpp
                         src/CMTUtils.cpp
                         src/Dispatcher.cpp
                         src/ParameterServer.cpp
                         src/main.cpp)
                                 
#link libriaries
target_link_libraries(c_tracker ${catkin_LIBRARIES})
                       
#Build the object selector node
add_executable(object_selector src/ObjectSelector.cpp)

#link libriaries
target_link_libraries(object_selector ${catkin_LIBRARIES})

#add dependencies
add_dependencies(c_tracker ${catkin_EXPORTED_TARGETS}  ${PROJECT_NAME}_gencfg)
add_dependencies(object_selector ${catkin_EXPORTED_TARGETS})	