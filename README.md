# Livox Ros Demo

Livox ROS demo is an application software running under ROS environment. It supports point cloud display using rviz. The Livox ROS demo includes two software packages, which are applicable when the host is connected to LiDAR sensors directly or when a Livox Hub is in use, respectively. This Livox ROS demo supports Ubuntu 14.04/Ubuntu 16.04, both x86 and ARM. It has been tested on Intel i7 and Nvidia TX2. 

## Livox ROS Demo User Guide

The Livox-SDK-ROS directory is organized in the form of ROS workspace, and is fully compatible with ROS workspace. Only a subfolder named src can be found under the Livox-SDK-ROS directory. Inside the src directory, there are two ROS software packages: display_lidar_points and display_hub_points.

1.	Download or clone the code from the Livox-SDK/Livox-SDK-ROS repository on GitHub. 
2.	Compile the ROS code package under the Livox-SDK-ROS directory by typing the following command in terminal:
  `catkin_make`
3.	Run the compiled ROS nodes:
  `rosrun display_lidar_points display_lidar_points_node`
  or
  `rosrun display_hub_points display_hub_points_node`
4.	Open a new terminal, and run roscore under the Livox-SDK-ROS directory:
  `roscore`
5.	Open another new terminal, and run rviz under the Livox-SDK-ROS directory:
  `rosrun rviz rviz`
6.	Set ROS RVIZ:
  1.	Create new visualization by display type, and select PointCloud;
  2.	Set the Fixed Frame to “sensor_frame” in Global Options and set Frame Rate to 20;
  3.	Select “/cloud” in Topic under the newly created PointCLoud.
