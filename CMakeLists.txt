cmake_minimum_required(VERSION 3.0.2)
project(master)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  actionlib_msgs
  std_msgs
  message_generation
  actionlib
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

add_executable(poseServer src/poseServer.cpp src/navigation_methods.cpp)
target_link_libraries(poseServer ${catkin_LIBRARIES})

add_executable(poseClient src/poseClient.cpp src/navigation_methods.cpp)
target_link_libraries(poseClient ${catkin_LIBRARIES})

add_dependencies(poseServer ${master_EXPORTED_TARGETS})
add_dependencies(poseClient ${master_EXPORTED_TARGETS})
