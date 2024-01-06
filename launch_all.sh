#!/bin/bash

# to run it:
#   chmod +x launch_all.sh
#   ./launch_all.sh

# Function to launch commands in separate tabs
launch_in_new_tab() {
    gnome-terminal --tab --title="$1" --command="bash -c '$2; exec bash'"
}

# Prerequisite commands
launch_in_new_tab "start_tiago" "start_tiago"
launch_in_new_tab "source_1" "source /opt/ros/noetic/setup.bash; sleep 1"
launch_in_new_tab "source_2" "source /tiago_public_ws/devel/setup.bash; sleep 1"
launch_in_new_tab "source_3" "source ~/catkin_ws/devel/setup.bash; sleep 1"

# Subsequent commands
commands=(
    "roslaunch tiago_iaslab_simulation start_simulation.launch world_name:=ias_lab_room_full_tables"
    "roslaunch tiago_iaslab_simulation apriltag.launch"
    "roslaunch tiago_iaslab_simulation navigation.launch"
    "rosrun tiago_iaslab_simulation human_node"
    "rosrun assignment2 poseServerRevisited"
    "rosrun assignment2 clientNode"
)

# Loop to run subsequent commands sequentially
for cmd in "${commands[@]}"; do
    launch_in_new_tab "start_tiago" "start_tiago"
    launch_in_new_tab "source_1" "source /opt/ros/noetic/setup.bash; sleep 1"
    launch_in_new_tab "source_2" "source /tiago_public_ws/devel/setup.bash; sleep 1"
    launch_in_new_tab "source_3" "source ~/catkin_ws/devel/setup.bash; sleep 1"
    launch_in_new_tab "$cmd" "$cmd"
    sleep 1  # Adjust this delay if needed
    read -p "Press Enter to continue to the next command..."
done

