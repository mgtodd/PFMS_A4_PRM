cmake_minimum_required(VERSION 2.8.3)
project(a4_prm)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    a4_setup
    roscpp
    sensor_msgs
    std_msgs
    nav_msgs
    rosbag
    roslib
)

add_definitions( -DANALYSIS=ON )

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system signals)
find_package(Eigen3 REQUIRED)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated -Wdeprecated-declarations")
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

message ( STATUS " CMake C++ FLAGS ${CMAKE_CXX_FLAGS}")



###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
    #  INCLUDE_DIRS include
    #  LIBRARIES drawing_tools
#  CATKIN_DEPENDS roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  src
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

## Declare C++ libraries
add_library(${PROJECT_NAME}-node src/node.cpp)
add_library(${PROJECT_NAME}-ros_interface src/RosInterface.cpp)
add_library(${PROJECT_NAME}-graph src/graph.cpp)
add_library(${PROJECT_NAME}-transform src/transform.cpp)
add_library(${PROJECT_NAME}-prm src/prm.cpp src/graph.cpp)
add_library(${PROJECT_NAME}-prm_controller src/PrmController.cpp)

## Declare a C++ executable
add_executable(${PROJECT_NAME}-run src/main.cpp)
# add_executable(${PROJECT_NAME}-run src/main.cpp src/graph.cpp src/node.cpp src/prm.cpp src/PrmController.cpp src/RosInterface.cpp src/transform.cpp)


## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}-run ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME}-run ${catkin_LIBRARIES} ${PROJECT_NAME}-graph)
target_link_libraries(${PROJECT_NAME}-run ${catkin_LIBRARIES} ${PROJECT_NAME}-prm_controller)
target_link_libraries(${PROJECT_NAME}-run ${catkin_LIBRARIES} ${PROJECT_NAME}-ros_interface)
target_link_libraries(${PROJECT_NAME}-run ${catkin_LIBRARIES} ${PROJECT_NAME}-node)
target_link_libraries(${PROJECT_NAME}-run ${catkin_LIBRARIES} ${PROJECT_NAME}-prm)
target_link_libraries(${PROJECT_NAME}-run ${catkin_LIBRARIES} ${PROJECT_NAME}-transform)

## Add cmake target dependencies of the executable
add_dependencies(${PROJECT_NAME}-run ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# Now we make the gtests

set(GTEST_ROOT "/usr/src/gtest" CACHE PATH "Path to googletest")
find_package(GTest REQUIRED)

include_directories(${GTEST_INCLUDE_DIRS})

if(NOT GTEST_LIBRARY)
   message("GTest library not found")
endif()

add_executable(${PROJECT_NAME}-graph_test test/test_graph.cpp src/graph.cpp src/node.cpp)
target_link_libraries(${PROJECT_NAME}-graph_test ${GTEST_LIBRARIES} pthread)

add_executable(${PROJECT_NAME}-transform_test test/test_transform.cpp src/transform.cpp src/node.cpp)
target_link_libraries(${PROJECT_NAME}-transform_test ${GTEST_LIBRARIES} pthread)

add_executable(${PROJECT_NAME}-prm_build test/test_prm_build.cpp src/prm.cpp src/graph.cpp src/node.cpp src/transform.cpp)
target_link_libraries(${PROJECT_NAME}-prm_build ${GTEST_LIBRARIES} ${catkin_LIBRARIES} pthread)
