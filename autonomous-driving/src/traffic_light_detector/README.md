## Overview
- The package traffic_light_detector is a rosservice that detect traffic lights and their colour, using this info, a command is passed to the master node, where the service is called, to either continue moving if the traffic light is green or yellow, and stops if it is red.

## Content

- As the traffic light detector is a rosservice, the package includes a header file, where the dependencies are included , and subscribers and publishers are initialized
- A server that gets the info from the cameras, processes the image, and transfer it into a command, and a client that calls the service, in addition to the srv and msg files where the type of the message to be requested is specified, and a response is passed.

## Prerequisites to run the package

- In addition to building the package, the following dependencies are required.

`OpenCV, cv_bridge and image_transport`

## starting the service

## Method 1

- The service starts automatically when the master node is started using the following command

`rosrun controller_pkg master_node`

- this command is used after starting the simulation using

`roslaunch car_bringup master.launch`

## Method 2

- The service can be started and its output can be seen after starting the simulation and without starting the master_node using the following commands

`rosrun traffic_light_detector traffic_light_server`

`rosrun traffic_light_detector traffic_light_client`

