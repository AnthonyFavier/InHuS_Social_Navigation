cmake_minimum_required(VERSION 3.0.2)
project(human_sim)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  genmsg
  actionlib_msgs
  actionlib
  roslib
  move_base_msgs
  tf
)

add_message_files(
  FILES
  Goal.msg
)

add_service_files(
  FILES
  ComputePlan.srv
  ChooseGoal.srv
  SetGetGoal.srv
)

#add_action_files(DIRECTORY action 
#  FILES 
#  HAction.action
#)

generate_messages(
  DEPENDENCIES 
  actionlib_msgs
  geometry_msgs
  move_base_msgs
)


catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES human_sim
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs actionlib_msgs move_base_msgs message_runtime
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(supervisor_node src/supervisor_node.cpp src/task.cpp)
add_dependencies(supervisor_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(supervisor_node ${catkin_LIBRARIES})

add_executable(human_model_node src/humanModel_node.cpp)
add_dependencies(human_model_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(human_model_node ${catkin_LIBRARIES})

add_executable(task_planner_node src/taskPlanner_node.cpp src/task.cpp)
add_dependencies(task_planner_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(task_planner_node ${catkin_LIBRARIES})

add_executable(preprocessing_h_node src/preprocessing_node.cpp)
add_dependencies(preprocessing_h_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(preprocessing_h_node ${catkin_LIBRARIES})

add_executable(log_manager_node src/logManager_node.cpp)
add_dependencies(log_manager_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(log_manager_node ${catkin_LIBRARIES})

add_executable(controller_node src/controller_node.cpp)
add_dependencies(controller_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(controller_node ${catkin_LIBRARIES})

add_executable(place_robot_node src/placeRobot_node.cpp)
add_dependencies(place_robot_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(place_robot_node ${catkin_LIBRARIES})


