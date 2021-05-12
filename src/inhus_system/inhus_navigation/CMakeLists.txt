cmake_minimum_required(VERSION 2.8.3)
project(inhus_navigation)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
)

add_service_files(
  FILES
  PlaceRobot.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp tf
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(placeRobot_node src/placeRobot_node.cpp)
add_dependencies(placeRobot_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(placeRobot_node ${catkin_LIBRARIES})

# install
foreach(dir maps launch/morse_pr2_launch config/morse_pr2_move_base_config launch/pr2_launch config/pr2_move_base_config launch/stage_pr2_launch config/stage_pr2_move_base_config)
   install(DIRECTORY ${dir}/
      DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
