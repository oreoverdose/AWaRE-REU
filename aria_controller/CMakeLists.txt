cmake_minimum_required(VERSION 2.8.3)
project(aria_controller)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES aria_controller
#  CATKIN_DEPENDS roscpp rospy
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  "/usr/local/Aria/include"
)


add_executable(aria_ros src/listener.cpp)

target_link_libraries(aria_ros ${catkin_LIBRARIES} pthread /usr/local/Aria/lib/libAria.so dl rt)

