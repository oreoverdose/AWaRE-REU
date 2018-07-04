cmake_minimum_required(VERSION 2.8.3)
project(beginner_tutorials)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
)
find_package(Boost REQUIRED COMPONENTS thread)
include_directories(${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES beginner_tutorials
#  CATKIN_DEPENDS roscpp rospy
#  DEPENDS system_lib
)

include_directories("/usr/local/Aria/include")

add_executable(listener src/listener.cpp)
target_link_libraries(listener ${catkin_LIBRARIES} ${Boost_LIBRARIES} pthread /usr/local/Aria/lib/libAria.so dl rt)
#set_target_properties(listener PROPERTIES COMPILE_FLAGS "-fPIC")




###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

