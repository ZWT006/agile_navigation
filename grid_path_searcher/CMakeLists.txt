cmake_minimum_required(VERSION 2.8.3)
project(grid_path_searcher)

# set debug mode
SET(CMAKE_BUILD_TYPE "RelWithDebInfo")

set(CMAKE_CXX_STANDARD 14)
find_package(catkin REQUIRED COMPONENTS 
    roscpp 
    std_msgs
    nav_msgs
    visualization_msgs 
)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)

# set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})

catkin_package(
  INCLUDE_DIRS include
)

include_directories(
    include 
    SYSTEM 
    third_party
    ${catkin_INCLUDE_DIRS} 
    ${EIGEN3_INCLUDE_DIR} 
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS} -O3 -Wall") # -Wextra -Werror
message(${OpenCV_INCLUDE_DIRS})




# target_link_libraries(demo_node 
#     ${catkin_LIBRARIES}
#     ${PCL_LIBRARIES})


add_executable ( image_map 
    src/image_map_generator.cpp )

target_link_libraries( image_map
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES} 
    ${OpenCV_LIBS}  )  