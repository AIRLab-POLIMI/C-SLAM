cmake_minimum_required(VERSION 2.8.3)

find_package(catkin REQUIRED COMPONENTS roscpp 
										cv_bridge 
										image_transport
										image_geometry
										dynamic_reconfigure
										c_slam_msgs
										c_fuzzy
										tf)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

project(c_vision)

generate_dynamic_reconfigure_options(cfg/Parameters.cfg)

catkin_package()

set(CMAKE_CXX_FLAGS "-g -std=c++11 ${OTHERS_DEFINE}")

include_directories(include 
					include/lib_cognitive_vision
					include/lib_metric_rectification
					${catkin_INCLUDE_DIRS}
					${EIGEN3_INCLUDE_DIR})
					
set(LIB_VISION_SOURCE_DIR ${PROJECT_SOURCE_DIR}/src/lib_cognitive_vision)
set(LIB_RECTIFICATION_SOURCE_DIR ${PROJECT_SOURCE_DIR}/src/lib_metric_rectification)

file(GLOB_RECURSE LIB_VISION_SOURCE 
                  ${LIB_VISION_SOURCE_DIR}/*.cpp)

#build the cognitive vision library
add_library(cognitive_vision STATIC
			${LIB_VISION_SOURCE})
			
			

#build the recognition node
add_executable(c_vision_detector src/BaseLogic.cpp
								 src/DetectorLogic.cpp 
                                 src/Detector.cpp)
                                 
#build the mapping and localization node
add_executable(c_vision_recognizer src/BaseLogic.cpp
						           src/Recognizer.cpp
							       src/RecognizerLogic.cpp)

#link libraries
target_link_libraries(c_vision_detector cognitive_vision 
					  					${catkin_LIBRARIES})
					  					
target_link_libraries(c_vision_recognizer cognitive_vision
					  				      ${catkin_LIBRARIES})
					  				      
					  				      
#Morphological test							       
add_executable(morph_test src/Test.cpp)
target_link_libraries(morph_test ${catkin_LIBRARIES})
					  				      
#add dependencies
add_dependencies(cognitive_vision ${catkin_EXPORTED_TARGETS})
add_dependencies(c_vision_detector ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
add_dependencies(c_vision_recognizer ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)					  				      
					  				