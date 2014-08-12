cmake_minimum_required(VERSION 2.8.3)

find_package(catkin REQUIRED COMPONENTS roscpp 
										cv_bridge 
										image_transport
										image_geometry)
find_package(OpenCV REQUIRED)

project(c_vision)

catkin_package()

include_directories(include 
					include/lib_cognitive_vision
					${catkin_INCLUDE_DIRS})
					
set(LIB_VISION_SOURCE_DIR ${PROJECT_SOURCE_DIR}/src/lib_cognitive_vision)


#build the cognitive vision library
add_library(cognitive_vision STATIC
			${LIB_VISION_SOURCE_DIR}/ParameterServer.cpp
            ${LIB_VISION_SOURCE_DIR}/Cluster.cpp
			${LIB_VISION_SOURCE_DIR}/CognitiveDetector.cpp
			${LIB_VISION_SOURCE_DIR}/DBSCAN.cpp
            ${LIB_VISION_SOURCE_DIR}/ClusterDetector.cpp
			${LIB_VISION_SOURCE_DIR}/HighLevelDetector.cpp
			${LIB_VISION_SOURCE_DIR}/LineDetector.cpp 
			${LIB_VISION_SOURCE_DIR}/ImageView.cpp 
			${LIB_VISION_SOURCE_DIR}/LineFilter.cpp
			${LIB_VISION_SOURCE_DIR}/ObjectClassificator.cpp
			${LIB_VISION_SOURCE_DIR}/Pole.cpp
			${LIB_VISION_SOURCE_DIR}/Rectangle.cpp)

#build the recognition node
add_executable(c_vision_detector src/BaseLogic.cpp
								 src/DetectorLogic.cpp 
                                 src/Detector.cpp)
                                 
#build the mapping and localization node
add_executable(c_vision_slam src/BaseLogic.cpp
						     src/SLAM.cpp
							 src/SLAMLogic.cpp)

#link libraries
target_link_libraries(c_vision_detector cognitive_vision 
					  					${catkin_LIBRARIES})
					  					
target_link_libraries(c_vision_slam cognitive_vision 
					  				${catkin_LIBRARIES})
					  				
#avoid tf and filters warnings
add_definitions(-DBOOST_SIGNALS_NO_DEPRECATION_WARNING)					  					
