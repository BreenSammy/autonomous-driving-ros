// odom_rotator_node.cpp
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class OdomRotatorNode {
public:
    OdomRotatorNode() {
        ros::NodeHandle nh;
        sub_ = nh.subscribe("input_odom", 10, &OdomRotatorNode::odomCallback, this);
        pub_ = nh.advertise<nav_msgs::Odometry>("output_odom", 10);
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        // Rotate the orientation by 90 degrees around the z-axis
        tf2::Quaternion quat;
        tf2::fromMsg(msg->pose.pose.orientation, quat);
        tf2::Quaternion rot_quat;
        rot_quat.setRPY(0, 0, M_PI/2.0); // 90 degrees rotation around z-axis
        quat = quat * rot_quat;

        // Convert back to geometry_msgs::Quaternion
        geometry_msgs::Quaternion rotated_quat;
        rotated_quat = tf2::toMsg(quat);

        // Create a new message with rotated orientation
        nav_msgs::Odometry rotated_odom = *msg;
        rotated_odom.pose.pose.orientation = rotated_quat;

        // Publish the rotated message
        pub_.publish(rotated_odom);
    }

private:
    ros::Subscriber sub_;
    ros::Publisher pub_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "odom_rotator_node");
    OdomRotatorNode node;
    ros::spin();
    return 0;
}