cmake_minimum_required(VERSION 3.0.2)
project(human_sim)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES human_sim
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(supervisor_node src/supervisor_node.cpp)
add_dependencies(supervisor_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(supervisor_node ${catkin_LIBRARIES})


