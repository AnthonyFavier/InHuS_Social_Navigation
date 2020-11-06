cmake_minimum_required(VERSION 3.0.2)
project(move_human)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy tf
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(place_robot_node src/placeRobot_node.cpp)
add_dependencies(place_robot_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(place_robot_node ${catkin_LIBRARIES})

