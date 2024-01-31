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

## Generate services in the 'msg' folder
add_message_files(
  FILES
  Obstacle.msg
  Object.msg
)

## Generate services in the 'srv' folder
add_service_files(
   FILES 
   Detection.srv
   Scan.srv
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

# List of source files for navigation_node
set(SOURCE_FILES_NAVIGATION
  src/navigation_node.cpp
  src/utils/navigation_methods.cpp
  src/utils/utils.cpp  
)
# List of source files for detection_node
set(SOURCE_FILES_DETECTION
  src/detection_node.cpp 
  src/utils/utils.cpp 
)

# List of source files for client_node
set(SOURCE_FILES_CLIENT
  src/client_node.cpp
  src/utils/navigation_methods.cpp
  src/utils/utils.cpp  
)

#List of source file for scan_node
set(SOURCE_FILES_SCAN
  src/scan_node.cpp 
  src/utils/utils.cpp 
  src/utils/scan_methods.cpp
)

#List of source file for manipulation_node
set(SOURCE_FILES_MANIPULATION
  src/manipulation_node.cpp 
  src/utils/utils.cpp 
)

# naigation_node executable
add_executable(navigation_node ${SOURCE_FILES_NAVIGATION})
target_link_libraries(navigation_node ${catkin_LIBRARIES})
add_dependencies(navigation_node ${assignment2_EXPORTED_TARGETS})

#detection_node executable
add_executable(detection_node ${SOURCE_FILES_DETECTION})
target_link_libraries(detection_node ${catkin_LIBRARIES}  ${OpenCV_LIBRARIES})
add_dependencies(detection_node ${assignment2_EXPORTED_TARGETS})

#manipulation_node executable
add_executable(manipulation_node ${SOURCE_FILES_MANIPULATION})
target_link_libraries(manipulation_node ${catkin_LIBRARIES}  ${OpenCV_LIBRARIES})
add_dependencies(manipulation_node ${assignment2_EXPORTED_TARGETS})

#scan_node executable
add_executable(scan_node ${SOURCE_FILES_SCAN} )
target_link_libraries(scan_node ${catkin_LIBRARIES}  ${OpenCV_LIBRARIES})
add_dependencies(scan_node ${assignment2_EXPORTED_TARGETS})

# client_node executable
add_executable(client_node ${SOURCE_FILES_CLIENT})
target_link_libraries(client_node ${catkin_LIBRARIES})
add_dependencies(client_node ${assignment2_EXPORTED_TARGETS})
