# Autonomous Driving ROS Project

This is the Autonomous Driving Project for the course "Introduction to ROS"

Team members: Dean Mercer, Lukas Evers, Mena Kuzman, Sammy Breen

## Installation

1. Install ROS noetic on Ubuntu 20.04 according to these instructions https://wiki.ros.org/noetic/Installation/Ubuntu. Please install ros-noetic-desktop-full.
2. Install python3-catkin-tools
```
sudo apt-get install python3-catkin-tools
```
3. Install external ROS packages used for the project
```
sudo apt install ros-noetic-move-base
sudo apt install ros-noetic-pointcloud-to-laserscan
sudo apt install ros-noetic-map-server
sudo apt install ros-noetic-teb-local-planner
```
4. Build the project with catkin build in the "Projects/AutonomousDriving/" folder
5. Download the [Unity Environment](https://github.com/BreenSammy/autonomous-driving-ros/releases/download/v.0.0.1/simulation.zip)
6. Unzip the Unity file and copy the files to .../devel/lib/simulation/ and make Car_build.x86_64 runnable
   
## Running the project

Do not forget to source the terminal with:
```
source devel/setup.bash
```

The entire software stack can be launched with a single launch file. To start the software use following command:

```
roslaunch car_bringup master.launch
```

The car will start driving and follows a path around the road network without colliding. It will stop at red lights.

Additionally to the visualization of the Unity simulation a rviz configuration can be used to visualize the object detection and planning. From the "Projects/AutonomousDriving/" folder launch rviz with:

```
rviz rviz -d src/car_bringup/rviz/nav.rviz
```
