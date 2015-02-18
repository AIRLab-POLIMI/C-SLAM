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

#Build localization library

add_library(localization STATIC
            src/ImuHandler.cpp   
            src/FullSlamImu.cpp
	        src/FullSlamConfig.cpp)
	        
add_library(roamfree_extra STATIC
            src/roamfree_extra/ObjectInitializationStrategy.cpp
            src/roamfree_extra/ObjectSufficientZChange.cpp
            src/roamfree_extra/ObjectSufficientParallax.cpp           
	        src/roamfree_extra/RectangleHandler.cpp
            src/roamfree_extra/AnchoredRectangleHandler.cpp
            src/roamfree_extra/AnchoredRectangleHandlerBootstrap.cpp)
            	        
add_library(testing STATIC
            src/test/RectangleGenerator.cpp
            src/test/TestPublisher.cpp
            src/test/FireflyTestPublisher.cpp)

#Build localization executables
add_executable(c_localization
               src/FullSlamImu_FHP.cpp)

add_executable(c_localization_rectangles
	           src/FullSlamImu_rectangles.cpp)

add_executable(c_localization_anchoredrectangles
	           src/FullSlamImu_anchoredrectangles.cpp)

#build test

add_executable(circular_test
               src/test/CircularTest.cpp)
               
add_executable(firefly_test
               src/test/FireflyTest.cpp)
               
add_executable(firefly_room_test
               src/test/FireflyRoomTest.cpp)

#build displayer               
add_executable(displayer
               src/display/Displayer.cpp)               

#Link libraries

target_link_libraries(c_localization
                      localization
                      ${catkin_LIBRARIES})

target_link_libraries(c_localization_rectangles
                      localization
                      roamfree_extra
                      ${catkin_LIBRARIES})

target_link_libraries(c_localization_anchoredrectangles
                      localization
                      roamfree_extra
                      ${catkin_LIBRARIES})
                      
target_link_libraries(circular_test testing ${catkin_LIBRARIES})
target_link_libraries(firefly_test testing ${catkin_LIBRARIES})
target_link_libraries(firefly_room_test testing ${catkin_LIBRARIES})

target_link_libraries(displayer ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#Add message dependencies
add_dependencies(c_localization ${catkin_EXPORTED_TARGETS})
add_dependencies(c_localization_rectangles ${catkin_EXPORTED_TARGETS})
add_dependencies(c_localization_anchoredrectangles ${catkin_EXPORTED_TARGETS})
add_dependencies(testing ${catkin_EXPORTED_TARGETS})
add_dependencies(circular_test ${catkin_EXPORTED_TARGETS})
add_dependencies(firefly_test ${catkin_EXPORTED_TARGETS})
add_dependencies(firefly_room_test ${catkin_EXPORTED_TARGETS})
add_dependencies(displayer ${catkin_EXPORTED_TARGETS})

