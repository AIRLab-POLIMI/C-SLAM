cmake_minimum_required(VERSION 2.8.3)

find_package(catkin REQUIRED COMPONENTS roscpp 
                                        cv_bridge 
                                        image_transport
                                        image_geometry
                                        geometry_msgs
                                        message_generation)
find_package(OpenCV REQUIRED)

project(c_tracking)

add_message_files(
  FILES
  NamedPolygon.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
)

catkin_package(CATKIN_DEPENDS message_runtime)

include_directories(include ${catkin_INCLUDE_DIRS})

#build the tracking node
add_executable(cognitive_tracker src/CMT.cpp
                                 src/CMTFeatureExtractor.cpp
                                 src/CMTUtils.cpp
                                 src/MappingTracker.cpp
                                 src/RobotPose.cpp
                                 src/Dispatcher.cpp
                                 src/main.cpp)
                                 
#link libriaries
target_link_libraries(cognitive_tracker ${catkin_LIBRARIES})
                       
#Build the object selector node
add_executable(object_selector src/ObjectSelector.cpp)

#link libriaries
target_link_libraries(object_selector ${catkin_LIBRARIES})
