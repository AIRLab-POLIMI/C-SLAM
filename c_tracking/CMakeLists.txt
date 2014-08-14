cmake_minimum_required(VERSION 2.8.3)

find_package(catkin REQUIRED COMPONENTS roscpp 
                                        cv_bridge 
                                        image_transport
                                        geometry_msgs
                                        sensor_msgs
                                        message_generation
                                        tf)
                                        
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)

project(c_tracking)

add_message_files(
  FILES
  NamedPolygon.msg
  TrackedObject.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  sensor_msgs
)

catkin_package(CATKIN_DEPENDS message_runtime)

include_directories(include ${catkin_INCLUDE_DIRS}
							${PCL_INCLUDE_DIRS})

#build the tracking node
add_executable(c_tracker src/CMT.cpp
                         src/CMTFeatureExtractor.cpp
                         src/CMTUtils.cpp
                         src/Dispatcher.cpp
                         src/main.cpp)
                                 
#link libriaries
target_link_libraries(c_tracker ${catkin_LIBRARIES})
                       
#Build the object selector node
add_executable(object_selector src/ObjectSelector.cpp)

#link libriaries
target_link_libraries(object_selector ${catkin_LIBRARIES})

#add dependency to generated messages
add_dependencies(c_tracker ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(object_selector ${PROJECT_NAME}_generate_messages_cpp)

#avoid tf warnings
add_definitions(-DBOOST_SIGNALS_NO_DEPRECATION_WARNING)
