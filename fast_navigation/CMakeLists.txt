cmake_minimum_required(VERSION 3.0.2)
project(fast_navigation)

# set debug mode
# SET(CMAKE_BUILD_TYPE "Debug")
SET(CMAKE_BUILD_TYPE "RelWithDebInfo")

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -std=c++14")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  nav_msgs
  visualization_msgs 
  cv_bridge
  lazykinoprm
  nontrajopt
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
# find_package(Python3 REQUIRED COMPONENTS Development)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES fast_navigation
 CATKIN_DEPENDS roscpp rospy std_msgs lazykinoprm nontrajopt
#  DEPENDS system_lib
)

###########
## Build ##
###########
#message(${Eigen3_INCLUDE_DIRS})

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  /usr/local/include/osqp
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}/include/fast_navigation
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS} 
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
#   ${Python3_INCLUDE_DIRS}
)


## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/fast_navigation.cpp
# )
add_executable( fast_planner_node 
    src/fast_planner.cpp
    # src/LazyKinoPRM.cpp
    # src/LazyPRM.cpp
    # src/Polynomial.cpp
    # src/SampleOBVP.cpp
    # src/GridNode.cpp
    )

target_link_libraries(fast_planner_node 
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS})

    
add_executable( swift_planner_node 
    src/swift_planner.cpp
    )

target_link_libraries(swift_planner_node 
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    # ${Python3_LIBRARIES}
    ${OpenCV_LIBS})

add_executable( pub_goalpose 
    src/pub_goalpose.cpp
)

target_link_libraries(pub_goalpose 
    ${catkin_LIBRARIES})


#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_fast_navigation.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
