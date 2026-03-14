cmake_minimum_required(VERSION 3.0.2)
project(controller_pkg)
add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  rospy 
  std_msgs 
  geometry_msgs 
  mav_msgs
  eigen_conversions
  tf_conversions
  message_generation
  simulation
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES controller_pkg
#  CATKIN_DEPENDS mav_msgs roscpp std_msgs
#  DEPENDS system_lib
  CATKIN_DEPENDS 
  simulation 
  roscpp 
  std_msgs
  mav_msgs 
  tf
  eigen_conversions
)

find_package(Eigen3)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


add_executable(controller_node src/controller_node.cpp)
add_dependencies(controller_node ${catkin_EXPORTED_TARGETS} simulation_generate_messages_cpp)
target_link_libraries(controller_node ${catkin_LIBRARIES})

