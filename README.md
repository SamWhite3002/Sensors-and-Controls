w# Sensors and Controls - Fetch Robot
Current implementation: Fetch robot to stack a red block ontop of a green block.

Future implmentation: Fetch robot to grasp a red coke can and place onto of a green coaster


# Installations:

1. **Install Ubuntu 18.04 and ROS Melodic**
   - Useful documentation for installation:
     - [Virtual Machine Youtube Tutorial](https://www.youtube.com/watch?v=q4-vGmx_WZY&t=1300s)
     - [Portable USB Ubuntu Stick](https://www.partitionwizard.com/partitionmanager/install-ubuntu-on-usb.html)

2. **MATLAB R2022b**

3. **Create catkin_ws**
   - Useful Link: [ROS Catkin Workspace Tutorial](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)
   - Terminal lines:
     ```bash
     mkdir -p ~/catkin_ws/src
     cd ~/catkin_ws/
     catkin_make
     source devel/setup.bash
     echo $ROS_PACKAGE_PATH
     ```
     Output:
     ```
     /home/youruser/catkin_ws/src:/opt/ros/melodic/share
     ```

4. **fetch_ros packages & fetch_gazebo packages**
   ```bash
   sudo apt install ros-melodic-fetch-calibration ros-melodic-fetch-open-auto-dock \
   ros-melodic-fetch-navigation ros-melodic-fetch-tools -y
   cd ~/catkin_ws/src/
   git clone https://github.com/fetchrobotics/fetch_gazebo.git
   cd ~/catkin_ws
   catkin_make
   source devel/setup.bash



# Locations
1. From folders listed, download project folder and place into workspace src folder
2. Download environment launch file and place it in fetch_gazebo/launch folder
3. Download environment sdf file into fetch_gazebo/worlds folder


# To Run:
1. Start up MATLAB
2. Launch Environment
>$roslaunch fetch_gazebo launch_my_environment.launch

3. Launch fetch_moveit
>$roslaunch fetch_moveit_config move_group.launch

4. Run startup
>$rosrun project startup.py

5. Run main MATLAB file (projectMAIN.m)

# Code Logic
![My Image](CodeLogic.png)


# Video Demonstration
[Fetch Robot Grasping Task](https://youtu.be/ivZmcTA-lXA)

