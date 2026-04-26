cmake_minimum_required(VERSION 2.8.3)
project(traffic_light_detector)


# Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  message_generation
)

# Find OpenCV
#find_package(OpenCV REQUIRED)

# Declare ROS messages and services
add_message_files(
  FILES
  TrafficLight.msg
)

add_service_files(
  FILES
  DetectTrafficLight.srv
)

# Generate messages and services
generate_messages(
  DEPENDENCIES
  sensor_msgs
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
  roscpp
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
  message_runtime
)

# Include directories
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# Add executables
add_executable(traffic_light_server src/traffic_light_server.cpp)
add_dependencies(traffic_light_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(traffic_light_server ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(traffic_light_client src/traffic_light_client.cpp)
add_dependencies(traffic_light_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(traffic_light_client ${catkin_LIBRARIES})

# Install targets if necessary
install(TARGETS traffic_light_server traffic_light_client
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

