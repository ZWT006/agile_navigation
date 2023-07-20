cmake_minimum_required(VERSION 3.0.2)
project(nontrajopt)

SET(CMAKE_BUILD_TYPE "Debug")

set(CMAKE_CXX_FLAGS "-std=c++14 ${CMAKE_CXX_FLAGS} -O3 -Wall")

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(PCL REQUIRED)
find_package(OsqpEigen REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
## COMPONENTS Development is required for Python3_INCLUDE_DIRS and Python3_LIBRARIES
## 在寻找python3的时候,一般都还得找到python3的开发包,否则会找不到Python3_INCLUDE_DIRS和Python3_LIBRARIES
find_package(Python3 REQUIRED COMPONENTS Development)
set(CMAKE_PREFIX_PATH "/usr/local/lib/cmake/nlopt/" ${CMAKE_PREFIX_PATH})
find_package(NLopt REQUIRED) # 没啥卵用 还是得靠后边的include_directories
# if(NLopt_FOUND)
#     message(WARNING "NLopt found:")
#     message(WARNING "  Version: ${NLopt_VERSION}")
#     message(WARNING "  Include directories: ${nlopt_INCLUDE_DIRS}")
#     message(WARNING "  Libraries: ${nlopt_LIBRARIES}")
# else()
#     message(WARNING "NLopt not found") 
# endif()


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
  INCLUDE_DIRS include #${EIGEN3_INCLUDE_DIR}
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp
  DEPENDS EIGEN3
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  /usr/local/include/osqp
  SYSTEM
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Python3_INCLUDE_DIRS}
  ${nlopt_INCLUDE_DIRS}
)

add_library(nontrajopt
  src/traj_opt_perching.cc
)

target_link_libraries(nontrajopt
  ${catkin_LIBRARIES}
  OsqpEigen::OsqpEigen
)
# message(SEND_ERROR "Python3_INCLUDE_DIRS: ${Python3_INCLUDE_DIRS}")
# message(SEND_ERROR "Python3_LIBRARIES: ${Python3_LIBRARIES}")

# message(WARNING "Python3_INCLUDE_DIRS: ${Python3_INCLUDE_DIRS}")
# message(WARNING "Python3_LIBRARIES: ${Python3_LIBRARIES}")

add_executable(MPCExample test/MPCExample.cpp)
target_link_libraries(MPCExample OsqpEigen::OsqpEigen)

add_executable(PlotTest test/PlotTest.cpp)
target_link_libraries(PlotTest ${Python3_LIBRARIES})

add_executable(nlopt_test test/nlopt_test.cpp)
target_link_libraries(nlopt_test 
        ${PROJECT_NAME}
        ${catkin_LIBRARIES}
        nlopt)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/nontrajopt.cpp
# )



#############
## Install ##
#############



#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(nontraj_test test/nontraj_test.cpp)
# if(TARGET nontraj_test)
#   target_link_libraries(nontraj_test 
#                         ${PROJECT_NAME}
#                         ${catkin_LIBRARIES})
# endif()
# add_executable(nontraj_test test/nontraj_test.cpp)
# target_link_libraries(nontraj_test 
#                       ${PROJECT_NAME}
#                       ${catkin_LIBRARIES}
#                       ${OpenCV_LIBRARIES}
#                       gtest
#                       OsqpEigen::OsqpEigen
#                       ${Python3_LIBRARIES}
#                       nlopt)

add_executable(nlopt_TO test/nlopt_TO.cpp)
target_link_libraries(nlopt_TO 
                      ${PROJECT_NAME}
                      ${catkin_LIBRARIES}
                      ${OpenCV_LIBRARIES}
                      gtest
                      OsqpEigen::OsqpEigen
                      ${Python3_LIBRARIES}
                      nlopt)

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
