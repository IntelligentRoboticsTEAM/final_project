# README #

GROUP 10
Nicola Calzone, nicola.calzone@studenti.unipd.it
Leonardo Da Re, leonardo.dare@studenti.unipd.it
Nicolò Tesser, nicolo.tesser@studenti.unipd.it

INSTRUCTIONS to run Assignment 2:
1. start_tiago
2. source /opt/ros/noetic/setup.bash
3. source /tiago_public_ws/devel/setup.bash
4. source ~/catkin_ws/devel/setup.bash
5. catkin build tiago_iaslab_simulation gazebo_ros_link_attacher
6. catkin build assignment2 
7. roslaunch tiago_iaslab_simulation start_simulation.launch
world_name:=ias_lab_room_full_tables
8. roslaunch tiago_iaslab_simulation apriltag.launch
9. roslaunch tiago_iaslab_simulation navigation.launch
10. rosrun tiago_iaslab_simulation human_node
11. rosrun assignment2 poseServerRevisited
12. rosrun assignment2 clientNode



Google Drive video: da_mettere
