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

add_executable(placeRobot_old_node src/placeRobot_node_old.cpp)
add_dependencies(placeRobot_old_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(placeRobot_old_node ${catkin_LIBRARIES})

add_executable(input_stage_node src/interface_stage_input_node.cpp)
add_dependencies(input_stage_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(input_stage_node ${catkin_LIBRARIES})

add_executable(output_stage_node src/interface_stage_output_node.cpp)
add_dependencies(output_stage_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(output_stage_node ${catkin_LIBRARIES})

add_executable(input_morse_node src/interface_morse_input_node.cpp)
add_dependencies(input_morse_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(input_morse_node ${catkin_LIBRARIES})

add_executable(output_morse_node src/interface_morse_output_node.cpp)
add_dependencies(output_morse_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(output_morse_node ${catkin_LIBRARIES})

# install
foreach(dir maps launch/stage_launch config/stage_pr2_move_base_config)
   install(DIRECTORY ${dir}/
      DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
