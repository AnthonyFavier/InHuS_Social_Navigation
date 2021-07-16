cmake_minimum_required(VERSION 3.0.2)
project(inhus)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  std_srvs
  geometry_msgs
  genmsg
  actionlib_msgs
  actionlib
  roslib
  move_base_msgs
  tf
  cmake_modules
)

  find_package(TinyXML REQUIRED)

add_message_files(
  FILES
  Goal.msg
  Action.msg
)

add_service_files(
  FILES
  ComputePlan.srv
  ActionBool.srv
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  geometry_msgs
  move_base_msgs
)


catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES inhus
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs actionlib_msgs move_base_msgs message_runtime
#  DEPENDS system_lib TinyXML
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(supervisor_node src/supervisor_node.cpp src/task.cpp)
add_dependencies(supervisor_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(supervisor_node ${catkin_LIBRARIES})

add_executable(humanBehaviorModel_node src/humanBehaviorModel_node.cpp)
add_dependencies(humanBehaviorModel_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(humanBehaviorModel_node ${catkin_LIBRARIES} ${TinyXML_LIBRARIES})

add_executable(taskPlanner_node src/taskPlanner_node.cpp src/task.cpp)
add_dependencies(taskPlanner_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(taskPlanner_node ${catkin_LIBRARIES})

add_executable(logManager_node src/logManager_node.cpp)
add_dependencies(logManager_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(logManager_node ${catkin_LIBRARIES})

add_executable(lowLevelController_node src/lowLevelController_node.cpp)
add_dependencies(lowLevelController_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(lowLevelController_node ${catkin_LIBRARIES})

add_executable(boss_node src/boss_node.cpp)
add_dependencies(boss_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(boss_node ${catkin_LIBRARIES} ${TinyXML_LIBRARIES})
