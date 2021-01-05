cmake_minimum_required(VERSION 3.0.2)
project(move_human)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
)

add_message_files(
  FILES
  test_m.msg
)

add_service_files(
  FILES
  PlaceRobot.srv
  Test.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
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

add_executable(pov_node src/test_pov_node.cpp)
add_dependencies(pov_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pov_node ${catkin_LIBRARIES})

