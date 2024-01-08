cmake_minimum_required(VERSION 3.0.2)
project(assignment2)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  # Base
  roscpp
  std_msgs
  message_generation
  tiago_iaslab_simulation
  roslaunch

  # For actions
  actionlib
  actionlib_msgs

  # for Image Processing Part
  cv_bridge
  image_transport
  sensor_msgs
  control_msgs
  geometry_msgs
  apriltag_ros
  moveit_ros_planning_interface
  tf
  tf_conversions
)

find_package(OpenCV REQUIRED COMPONENTS
    opencv_highgui)

roslaunch_add_file_check(launch)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
add_message_files(
#   DIRECTORY msg
   FILES 
   Obstacle.msg 
   Object.msg
)

## Generate services in the 'srv' folder
add_service_files(
   FILES 
   Detection.srv
)


## Generate actions in the 'action' folder
add_action_files(
#  DIRECTORY action
  FILES 
  Pose.action 
  Arm.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  actionlib_msgs
  std_msgs
  geometry_msgs
  apriltag_ros
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################


catkin_package(
  CATKIN_DEPENDS message_runtime roscpp actionlib tiago_iaslab_simulation
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  SYSTEM
  ${catkin_INCLUDE_DIRS}
)

# List of source files for poseServerRevisited
set(SOURCE_FILES_SERVER_REVISITED
  src/poseServerRevisited.cpp
  src/navigation_methods.cpp
  src/utils.cpp  
)
# List of source files for clientNode
set(SOURCE_FILES_CLIENT_NODE
  src/clientNode.cpp
  src/utils.cpp  
)

# poseServerRevisited executable
add_executable(poseServerRevisited ${SOURCE_FILES_SERVER_REVISITED})
target_link_libraries(poseServerRevisited ${catkin_LIBRARIES})
add_dependencies(poseServerRevisited ${assignment2_EXPORTED_TARGETS})

#detectionNode executable
add_executable(detectionNode src/detectionNode.cpp src/utils.cpp)
target_link_libraries(detectionNode ${catkin_LIBRARIES}  ${OpenCV_LIBRARIES})
add_dependencies(detectionNode ${assignment2_EXPORTED_TARGETS})

#manipulationNode executable
add_executable(manipulationNode src/manipulationNode.cpp src/utils.cpp)
target_link_libraries(manipulationNode ${catkin_LIBRARIES}  ${OpenCV_LIBRARIES})
add_dependencies(manipulationNode ${assignment2_EXPORTED_TARGETS})


# clientNode executable
add_executable(clientNode ${SOURCE_FILES_CLIENT_NODE})
target_link_libraries(clientNode ${catkin_LIBRARIES})
add_dependencies(clientNode ${assignment2_EXPORTED_TARGETS})
