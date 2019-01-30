# Livox Ros Demo

Livox ROS demo is an application software running under ROS environment. It supports point cloud display using rviz. The Livox ROS demo includes two software packages, which are applicable when the host is connected to LiDAR sensors directly or when a Livox Hub is in use, respectively. This Livox ROS demo supports Ubuntu 14.04/Ubuntu 16.04, both x86 and ARM. It has been tested on Intel i7 and Nvidia TX2. 

## Livox ROS Demo User Guide

The Livox-SDK-ROS directory is organized in the form of ROS workspace, and is fully compatible with ROS workspace. Only a subfolder named src can be found under the Livox-SDK-ROS directory. Inside the src directory, there are two ROS software packages: display_lidar_points and display_hub_points.

### Compile&Install Livox-SDK 

1.Download or clone the code from the Livox-SDK/Livox-SDK repository on GitHub. 

2.Compile the Livox-SDK under the build directory by typing the following command in terminal:

```
   cmake ..
   make -j &(nproc)
   make install
```

 

### Run ros demo and display pointcloud by rviz 

1. Download or clone the code from the Livox-SDK/Livox-SDK-ROS repository on GitHub. 

2. Add broadcast code to broadcast_code_list on line 80 of main.cpp .

   How to find the broadcast code:

   1) the broadcast code will be printed in the terminal when run the ros demo.

   or

   2) The broadcast code is located on the QR code of the housing.

3. Compile the ROS code package under the Livox-SDK-ROS directory by typing the following command in terminal:
     `catkin_make`

4. Run the compiled ROS nodes:
     `rosrun display_lidar_points display_lidar_points_node`
     or
     `rosrun display_hub_points display_hub_points_node`

5. Open a new terminal, and run roscore under the Livox-SDK-ROS directory:
     `roscore`

6. Open another new terminal, and run rviz under the Livox-SDK-ROS directory:
     `rosrun rviz rviz`

7. Set ROS RVIZ:

  8. Create new visualization by display type, and select PointCloud;

  9. Set the Fixed Frame to “sensor_frame” in Global Options and set Frame Rate to 20;

  10. Select “/cloud” in Topic under the newly created PointCLoud.
