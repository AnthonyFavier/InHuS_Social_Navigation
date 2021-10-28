cmake_minimum_required(VERSION 2.8.3)
project(inhus_navigation)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  roslib
  cohan_msgs
)

add_service_files(
  FILES
  PlaceRobot.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  cohan_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp tf cohan_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(placeRobot_pub_node src/placeRobot_pub_node.cpp)
add_dependencies(placeRobot_pub_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(placeRobot_pub_node ${catkin_LIBRARIES})

add_executable(placeRobot_client_node src/placeRobot_client_node.cpp)
add_dependencies(placeRobot_client_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(placeRobot_client_node ${catkin_LIBRARIES})

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

add_executable(logManagerCohan_node src/logManagerCohan_node.cpp)
add_dependencies(logManagerCohan_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(logManagerCohan_node ${catkin_LIBRARIES})

add_executable(agentsBridge_node src/agents_bridge.cpp)
add_dependencies(agentsBridge_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(agentsBridge_node ${catkin_LIBRARIES})

# install
foreach(dir maps launch/stage_launch)
   install(DIRECTORY ${dir}/
      DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
