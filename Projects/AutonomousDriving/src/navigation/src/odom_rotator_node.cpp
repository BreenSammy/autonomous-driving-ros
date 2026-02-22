// odom_rotator_node.cpp
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf2_msgs/TFMessage.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <string>

class OdomRotatorNode {
public:
    OdomRotatorNode() {
        ros::NodeHandle nh;
        ros::NodeHandle pnh("~");
        pnh.param("target_child_frame", target_child_frame_, std::string("OurCar/INS"));
        sub_ = nh.subscribe("input_odom", 10, &OdomRotatorNode::odomCallback, this);
        sub_tf_ = nh.subscribe("input_tf", 10, &OdomRotatorNode::tfCallback, this);
        pub_ = nh.advertise<nav_msgs::Odometry>("output_odom", 10);
    }
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        rotateAndPublish(*msg);
    }

    void tfCallback(const tf2_msgs::TFMessage::ConstPtr& msg) {
        if (msg->transforms.empty()) return;
        // Find the transform with the configured child_frame_id
        for (const auto& t : msg->transforms) {
            if (t.child_frame_id == target_child_frame_) {
                nav_msgs::Odometry odom;
                odom.header = t.header;
                odom.child_frame_id = t.child_frame_id;
                odom.pose.pose.position.x = t.transform.translation.x;
                odom.pose.pose.position.y = t.transform.translation.y;
                odom.pose.pose.position.z = t.transform.translation.z;
                odom.pose.pose.orientation = t.transform.rotation;
                rotateAndPublish(odom);
                return;
            }
        }
    }

    void rotateAndPublish(const nav_msgs::Odometry& in_msg) {
        tf2::Quaternion quat;
        tf2::fromMsg(in_msg.pose.pose.orientation, quat);
        tf2::Quaternion rot_quat;
        rot_quat.setRPY(0, 0, M_PI/2.0); // 90 degrees rotation around z-axis
        quat = quat * rot_quat;

        geometry_msgs::Quaternion rotated_quat = tf2::toMsg(quat);

        nav_msgs::Odometry rotated_odom = in_msg;
        rotated_odom.pose.pose.orientation = rotated_quat;

        pub_.publish(rotated_odom);
    }

private:
    ros::Subscriber sub_;
    ros::Subscriber sub_tf_;
    ros::Publisher pub_;
    std::string target_child_frame_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "odom_rotator_node");
    OdomRotatorNode node;
    ros::spin();
    return 0;
}