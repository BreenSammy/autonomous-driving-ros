#include <ros/ros.h>
#include <stdio.h>
#include <string.h>

#include <ros/console.h>

#include <simulation/VehicleControl.h>

#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <mav_msgs/Actuators.h>
#include <nav_msgs/Odometry.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <math.h>
#include <std_msgs/Float64.h>

#include <eigen3/Eigen/Dense>

// If you choose to use Eigen, tf provides useful functions to convert tf
// messages to eigen types and vice versa, have a look to the documentation:
// http://docs.ros.org/melodic/api/eigen_conversions/html/namespacetf.html
#include <eigen_conversions/eigen_msg.h>

class controllerNode
{
  ros::NodeHandle nh_;

  ros::Subscriber cmd_vel_sub_;
  ros::Subscriber current_state_;
  ros::Subscriber teb_feedback_sub_;
  ros::Publisher car_commands_;
  ros::Timer timer_;

  // Controller internals (you will have to set them below)
  // Current state
  Eigen::Vector3d x_;     // current position of the cars c.o.m. in the world frame
  Eigen::Vector3d v_;     // current velocity of the cars c.o.m. in the world frame
  Eigen::Vector3d v_cvl_; // current velocity of the cars c.o.m. in the body frame
  Eigen::Matrix3d R_;     // current orientation of the car
  Eigen::Vector3d omega_; // current angular velocity of the car c.o.m. in the *body* frame

  // Desired state
  Eigen::Vector3d xd_; // desired position of the car c.o.m. in the world frame
  Eigen::Vector3d vd_; // desired velocity of the car c.o.m. in the world frame
  Eigen::Vector3d ad_; // desired acceleration of the car c.o.m. in the world frame

  double hz_;                        // frequency of the main control loop
  double integrator_value_;          // Integrator value for controller
  double previous_error_;            // Save previous control error for derivative part of controller
  double previous_steering_error_;   // Save previous control error for derivative part of controller
  double integrator_steering_value_; // Integrator value for steering controller
  double desired_angular_velocity_;

  ros::Time last_cmd_vel_time_; // Timestamp of last received cmd_vel
  double cmd_vel_timeout_;      // Timeout duration in seconds

  bool local_plan_feasible_;

public:
  controllerNode() : hz_(50.0)
  {
    // Get current state from the odometry used by move_base
    current_state_ = nh_.subscribe("odom", 1, &controllerNode::onCurrentState, this);
    cmd_vel_sub_ = nh_.subscribe("cmd_vel", 1, &controllerNode::cmdVelCallback, this);
    // car_commands_ = nh_.advertise<mav_msgs::Actuators>("car_commands", 1);

    car_commands_ = nh_.advertise<simulation::VehicleControl>("car_command", 1);

    timer_ = nh_.createTimer(ros::Rate(hz_), &controllerNode::controlLoop, this);

    integrator_value_ = 0;
    previous_error_ = 0;

    previous_steering_error_ = 0;
    integrator_steering_value_ = 0;

    // Initialize cmd_vel timeout (1.0 second)
    cmd_vel_timeout_ = 1.0;
    last_cmd_vel_time_ = ros::Time::now();
  }

  void cmdVelCallback(const geometry_msgs::Twist &cmd_vel)
  {
    // Set desired velocity and angular velocity from cmd_vel
    vd_ << cmd_vel.linear.x, 0, 0;
    desired_angular_velocity_ = -cmd_vel.angular.z;
    // Update timestamp of last received cmd_vel
    last_cmd_vel_time_ = ros::Time::now();
  }

  void onCurrentState(const nav_msgs::Odometry &cur_state)
  {

    x_ << cur_state.pose.pose.position.x, cur_state.pose.pose.position.y, cur_state.pose.pose.position.z;
    v_ << cur_state.twist.twist.linear.x, cur_state.twist.twist.linear.y, cur_state.twist.twist.linear.z;
    omega_ << cur_state.twist.twist.angular.x, cur_state.twist.twist.angular.y, cur_state.twist.twist.angular.z;
    Eigen::Quaterniond q;
    tf::quaternionMsgToEigen(cur_state.pose.pose.orientation, q);
    R_ = q.toRotationMatrix();
    // Rotate omega
    // omega_ = R_.transpose() * omega_;
    omega_ = -omega_;
    // v_cvl_ = R_.transpose() * v_;
    v_cvl_ = v_;
  }

  // Function to compute the steering angle from desired angular velocity
  double computeSteeringAngle(double omega, double linear_velocity)
  {
    double wheelbase = 2.63;
    // Check for zero linear velocity to avoid division by zero
    if (linear_velocity == 0)
    {
      return 0.0;
    }

    // Compute the steering angle
    double steering_angle = std::atan(omega * wheelbase / linear_velocity);
    return steering_angle;
  }

  void controlLoop(const ros::TimerEvent &t)
  {
    double braking = 0;

    // Check if cmd_vel has timed out (no command received for too long)
    ros::Time now = ros::Time::now();
    if ((now - last_cmd_vel_time_).toSec() > cmd_vel_timeout_)
    {
      // Timeout exceeded - stop the vehicle
      // ROS_WARN_THROTTLE(1.0, "controller: cmd_vel timeout! Stopping vehicle.");
      vd_.setZero();
      desired_angular_velocity_ = 0;
      braking = 1.0; // Apply full brake
    }

    // Get desired and current velocities from vectors
    double current_velocity = v_cvl_[0];
    double desired_velocity = vd_[0];

    // Acceleration controller based on simple PID controller
    double dt = 1 / hz_;
    double control_error = desired_velocity - current_velocity;

    // Control gains
    double k_p = 3.0;
    double k_i = 0.0;
    double k_d = 0.00; // Must be smaller than 1

    double derivative = (control_error - previous_error_) / dt;
    integrator_value_ = integrator_value_ + control_error * dt;

    // Calculate acceleration output of controller
    double acceleration_output = k_p * control_error + k_i * integrator_value_ + k_d * derivative;

    // Save current error for derivative calculation
    previous_error_ = control_error;

    // Steering Angle
    // Convert angular velocities to corresponding steering angles
    double current_angular_velocity = omega_[2];
    double current_steering_angle = computeSteeringAngle(current_angular_velocity, current_velocity);
    // double desired_steering_angle = computeSteeringAngle(desired_angular_velocity_, desired_velocity);
    double desired_steering_angle = desired_angular_velocity_;
    double steering_angle_error = desired_steering_angle - current_steering_angle;

    ROS_WARN_THROTTLE(1.0, "Current steering angle: %.4f, Desired: %.4f", current_steering_angle, desired_steering_angle);
    ROS_WARN_THROTTLE(1.0, "Current steering angle error: %.4f", steering_angle_error);
    // P controller for turning angle
    double K_p_omega = 8.0;
    double K_i_omega = 0.0;
    double K_d_omega = 0.0; // Must be smaller than 1

    double derivative_steering = (steering_angle_error - previous_steering_error_) / dt;

    previous_steering_error_ = steering_angle_error;
    integrator_steering_value_ = integrator_steering_value_ + steering_angle_error * dt;

    double turning_angle = K_p_omega * steering_angle_error + K_i_omega * integrator_steering_value_ + K_d_omega * derivative_steering;

    // mav_msgs::Actuators msg;

    simulation::VehicleControl msg;
    msg.Throttle = std::tanh(acceleration_output); // Throttle value from -1 to 1, this is the torque applied to the motors
    msg.Steering = turning_angle;                  // Steering value from -1 to 1, in which: positive value <=> turning right
    msg.Brake = braking;                           // Brake value from 0 to 1, this will apply brake torque to stop the car
    msg.Reserved = 0.0f;                           // Not used!

    // call the traffic light detector service
    // msg.angular_velocities.resize(4);
    // msg.angular_velocities[0] = ;  // Acceleration
    // msg.angular_velocities[1] = ;       // Turning angle
    // msg.angular_velocities[2] = ;            // Braking
    // msg.angular_velocities[3] = 0;                   // Other (not used)*/

    car_commands_.publish(msg);
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "controller_node");
  ROS_INFO_NAMED("controller", "Controller started!");
  controllerNode n;
  ros::spin();
}
