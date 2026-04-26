# Autonomous Driving ROS Project

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
4. Build the project with catkin build in the "Projects/AutonomousDriving/" folder with 
```
catkin build
```
   
## Running the project

Do not forget to source the terminal with:
```
source devel/setup.bash
```

The entire driving stack can be launched with a single launch file. To start the software use following command:

```
roslaunch car_bringup master.launch
```

This will open the Unity simulation and Rviz to visualize the sensor outputs and driving stack.
The car will start driving and follows a path around the road network without colliding. It will stop at red lights.

# TODO's

- Increase planning performance
- Shorter turn cycles
- Lane keeping
