cmake_minimum_required(VERSION 3.0.2)
project(assignment1)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  actionlib_msgs
  std_msgs
  message_generation
  actionlib
)

add_message_files(
  FILES
  Obstacle.msg
)

add_action_files(
  DIRECTORY action
  FILES Pose.action
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime roscpp actionlib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# List of source files for poseServer
set(SOURCE_FILES_SERVER
  src/poseServer.cpp
  src/navigation_methods.cpp
  src/utils.cpp  
  src/scan_methods.cpp
)
# List of source files for poseClient
set(SOURCE_FILES_CLIENT
  src/poseClient.cpp
  src/utils.cpp  
)

# poseServer executable
add_executable(poseServer ${SOURCE_FILES_SERVER})
target_link_libraries(poseServer ${catkin_LIBRARIES})
add_dependencies(poseServer ${assignment1_EXPORTED_TARGETS})

# poseClient executable
add_executable(poseClient ${SOURCE_FILES_CLIENT})
target_link_libraries(poseClient ${catkin_LIBRARIES})
add_dependencies(poseClient ${assignment1_EXPORTED_TARGETS})
