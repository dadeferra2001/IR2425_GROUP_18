cmake_minimum_required(VERSION 3.0.2)
project(assignment1)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  actionlib
  tf2
  actionlib_msgs
  geometry_msgs
  move_base_msgs
  sensor_msgs
  tf2_geometry_msgs
  message_generation
  message_runtime
  tiago_iaslab_simulation
  apriltag_ros
  trajectory_msgs
  control_msgs
  nav_msgs
)

add_action_files(
  DIRECTORY action
  FILES SearchIds.action
)

generate_messages(
  DEPENDENCIES actionlib_msgs std_msgs geometry_msgs move_base_msgs sensor_msgs tiago_iaslab_simulation apriltag_ros trajectory_msgs control_msgs nav_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp actionlib_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Build action server and client executables
add_executable(action_server src/action_server.cpp)
add_executable(action_client src/action_client.cpp)

# Link functions.cpp and robot_status.cpp for shared functionality
target_sources(action_server PRIVATE src/functions.cpp src/robot_status.cpp)
target_sources(action_client PRIVATE src/functions.cpp src/robot_status.cpp)

# Add dependencies
add_dependencies(action_server ${catkin_EXPORTED_TARGETS} ${assignment1_EXPORTED_TARGETS})
add_dependencies(action_client ${catkin_EXPORTED_TARGETS} ${assignment1_EXPORTED_TARGETS})

# Link libraries
target_link_libraries(action_server
  ${catkin_LIBRARIES}
)
target_link_libraries(action_client
  ${catkin_LIBRARIES}
)

# Install launch files
install(DIRECTORY launch/
        DESTINATION share/${PROJECT_NAME}/launch
        FILES_MATCHING PATTERN "*.launch"
)
