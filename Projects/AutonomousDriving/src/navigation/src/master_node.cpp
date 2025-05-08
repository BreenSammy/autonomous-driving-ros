#include <fstream>
#include <iostream>

#include <ros/ros.h>
#include <ros/console.h>
#include <nav_msgs/Odometry.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <geometry_msgs/Quaternion.h>
#include <math.h>
#include <std_msgs/Float64.h>
#include <eigen3/Eigen/Dense>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <traffic_light_detector/DetectTrafficLight.h>
#include <traffic_light_detector/TrafficLight.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class MasterNode{
  ros::NodeHandle nh_;
  ros::Subscriber current_state_subscriber_;
  ros::ServiceClient traffic_light_service_ ;
  MoveBaseClient ac_{"/move_base", true};

  Eigen::Vector3d position_;
  Eigen::Vector3d target_waypoint_;
  std::vector<Eigen::Vector3d> waypoints_;
  std::vector<geometry_msgs::Quaternion> orientations_;

  double waypoint_reached_threshold_ {15.0};
  int index_{-1};
  bool done_{false};
  int state_{0};
  int prev_state_{0};

  std::string file_path_;

public:
  MasterNode(const std::string& file_path) : file_path_(file_path) {
    ROS_INFO("Waypoint generator starting");

    loadWaypoints();

    while(!ac_.waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the move_base action server to come up");
    }
    current_state_subscriber_ = nh_.subscribe("/odom", 5, &MasterNode::onCurrentState, this);
    traffic_light_service_ = nh_.serviceClient<traffic_light_detector::DetectTrafficLight>("detect_traffic_light");
  }

  void loadWaypoints(){
    double x, y, z;
    double ox, oy, oz, ow;

    std::ifstream file(file_path_, std::ios_base::in);
    while(file >> x >> y >> z >> ox >> oy >> oz >> ow){
      std::cout << x << " " << y << std::endl;
      auto waypoint = Eigen::Vector3d(x, y, z);
      waypoints_.push_back(waypoint);

      geometry_msgs::Quaternion orientation;
      orientation.x = ox;
      orientation.y = oy;
      orientation.z = oz;
      orientation.w = ow;
      orientations_.push_back(orientation);
    }
    std::cout << waypoints_.size() << " waypoints loaded" << std::endl;
  }

  void start(std::string frame_id){
    ROS_INFO("Publishing first waypoint");
    index_ = 0;
    if(index_ < waypoints_.size()){
      target_waypoint_ = waypoints_[index_];

      move_base_msgs::MoveBaseGoal goal;
      goal.target_pose.header.frame_id = frame_id;
      goal.target_pose.header.stamp = ros::Time::now();
      goal.target_pose.pose.position.x = (double)target_waypoint_.x();
      goal.target_pose.pose.position.y = (double)target_waypoint_.y();
      goal.target_pose.pose.position.z = (double)target_waypoint_.z();
      
      goal.target_pose.pose.orientation = orientations_[0];

      ac_.sendGoal(goal);
    }
  }

  void next(std::string frame_id){
    ROS_INFO("Waypoint reached"); 
    index_++;
    if(index_ < waypoints_.size()){
      target_waypoint_ = waypoints_[index_];

      move_base_msgs::MoveBaseGoal goal;
      goal.target_pose.header.frame_id = frame_id;
      goal.target_pose.header.stamp = ros::Time::now();
      goal.target_pose.pose.position.x = (double)target_waypoint_.x();
      goal.target_pose.pose.position.y = (double)target_waypoint_.y();
      goal.target_pose.pose.position.z = (double)target_waypoint_.z();

      goal.target_pose.pose.orientation = orientations_[index_];

      ac_.sendGoal(goal);
    }
    else{
      done_ = true;
    }
  }

  void onCurrentState(const nav_msgs::Odometry& cur_state){
    if(done_){
      return;
    }
      
    position_ << cur_state.pose.pose.position.x,cur_state.pose.pose.position.y,cur_state.pose.pose.position.z;
    if(index_<0){
      start(cur_state.header.frame_id);
    }
    double distance = (target_waypoint_ - position_).norm();
    if(distance <= waypoint_reached_threshold_){
      next(cur_state.header.frame_id);
    }
    // Call the traffic light detection service
    traffic_light_detector::DetectTrafficLight srv;
    if (traffic_light_service_.call(srv)) {
      std::string traffic_light_status = srv.response.status;

      if (traffic_light_status == "STOP") {
        state_ = 1;
      } else if (traffic_light_status== "CAUTION") {
        state_ = 2;
      } else if (traffic_light_status == "GO") {
        state_ = 3;
      }

      if (state_ == prev_state_) {
        return;
      }
      ROS_INFO("Traffic Light Status: %s", traffic_light_status.c_str());

      if(state_ == 1){
        ROS_INFO("Red light detected, stopping...");
        ac_.cancelAllGoals(); // Cancel the current goal to stop the robot
      } 
      else if (state_ == 3 || state_ == 2) {
        ROS_INFO("No red light detected, resuming...");
        if (index_ < waypoints_.size()) {
          move_base_msgs::MoveBaseGoal goal;
          goal.target_pose.header.frame_id = cur_state.header.frame_id;
          goal.target_pose.header.stamp = ros::Time::now();
          goal.target_pose.pose.position.x = (double)target_waypoint_.x();
          goal.target_pose.pose.position.y = (double)target_waypoint_.y();
          goal.target_pose.pose.position.z = (double)target_waypoint_.z();
          goal.target_pose.pose.orientation = orientations_[index_];
          ac_.sendGoal(goal); // Resume navigation by sending the goal again
        }
      } 
        prev_state_ = state_;
    }
    else {
      ROS_ERROR("Failed to call service detect_traffic_light");
    }
  }
};

int main(int argc, char** argv){
  ros::init(argc, argv, "master_node");
  ROS_INFO_NAMED("master_node", "Master starting");

  if (argc < 2) {
        ROS_ERROR("No file path provided.");
        return -1;
  }

  std::string file_path = argv[1];
  ROS_INFO("File path is: %s", file_path.c_str());

  MasterNode n(file_path);
  ros::spin();
}
