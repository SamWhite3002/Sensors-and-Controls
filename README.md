# Sensors and Controls - Fetch Robot
Fetch robot to stack a red block ontop of a green block.

Future implmentation: Fetch robot to grasp a red coke can and place onto of a green coaster

# Installations:
Ubuntu 18.04

ROS Melodic

MATLAB R2022b

fetch_ros packages

fetch_gazebo packages

movit_python packages


# To Run:
1. Launch Environment
>$roslaunch fetch_gazebo project_environment.launch

2. Launch fetch_moveit
>$roslaunch fetch_moveit_config move_group.launch

4. Run startup
>$rosrun project startup.py

5. Run main MATLAB file

