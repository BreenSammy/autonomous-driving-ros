
# Autonomous Driving ROS Project

This project is a complete autonomous driving software stack built with RO. It demonstrates an integrated system capable of autonomous navigation, obstacle avoidance, path planning, and traffic light recognition. The platform simulates a vehicle navigating through a road network in a Unity-based environment, with real-time visualization of sensor data and decision-making processes in RViz.

<img width="598" height="448" alt="drive_" src="https://github.com/user-attachments/assets/f1a583a6-69b8-40a1-a6ba-12c1d2972a51" />

<img width="598" height="448" alt="red-light(1)" src="https://github.com/user-attachments/assets/f0f7ee76-c78d-46ef-a8d6-8829496ae727" />

### Key Features

- **Path Planning & Navigation**: Dynamic path planning with collision avoidance and local path optimization
- **Traffic Light Detection**: Computer vision-based traffic light recognition and state interpretation
- **Sensor Integration**: Support for depth sensors and LiDAR-like sensor fusion for environmental awareness
- **Real-time Visualization**: RViz integration for monitoring sensor outputs and driving behavior
- **Simulation Environment**: Unity-based simulation for testing and validation

### Project Structure

- **car_bringup**: Main launch configurations and system initialization
- **controller_pkg**: Vehicle control and command execution
- **navigation**: Path planning and route management
- **traffic_light_detector**: Traffic signal recognition and processing
- **depth_image_conversion**: Sensor data processing and conversion
- **simulation**: Unity simulation interface and environment setup




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
