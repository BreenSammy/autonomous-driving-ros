#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Header.h>

class OdomPublisher
{
public:
    OdomPublisher()
    {
        // topics coming from Unity or other upstream nodes are stamped
        // we only care about the pose/twist inside the stamp
        pose_sub_ = nh_.subscribe("/pose", 10, &OdomPublisher::poseCallback, this);
        twist_sub_ = nh_.subscribe("/twist", 10, &OdomPublisher::twistCallback, this);

        odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/odom", 10);
    }

    void spin()
    {
        ros::Rate rate(50);

        while (ros::ok())
        {
            ros::spinOnce();

            if (!pose_received_ || !twist_received_)
            {
                rate.sleep();
                continue;
            }

            publishOdom();

            rate.sleep();
        }
    }

private:
    ros::NodeHandle nh_;

    ros::Subscriber pose_sub_;
    ros::Subscriber twist_sub_;

    ros::Publisher odom_pub_;

    geometry_msgs::Pose pose_;          // last received pose (un-stamped)
    geometry_msgs::Twist twist_;        // last received twist (un-stamped)
    std_msgs::Header last_pose_header_; // header from pose topic

    bool pose_received_ = false;
    bool twist_received_ = false;

    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        // preserve header so we can timestamp odom consistently
        last_pose_header_ = msg->header;
        pose_ = msg->pose;
        pose_received_ = true;
    }

    void twistCallback(const geometry_msgs::TwistStamped::ConstPtr &msg)
    {
        twist_ = msg->twist;
        twist_received_ = true;
    }

    void publishOdom()
    {
        // Create odometry message with consistent pose and twist
        nav_msgs::Odometry odom;

        odom.header.stamp = last_pose_header_.stamp;
        odom.header.frame_id = "odom";

        odom.child_frame_id = "base_link";

        // Publish pose and twist directly - they come from the same source (Unity)
        // so they're already consistent with each other
        odom.pose.pose = pose_;
        odom.twist.twist = twist_;

        odom_pub_.publish(odom);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "odom_publisher");
    ROS_INFO_NAMED("odom_publisher", "Odom publisher starting");

    OdomPublisher node;
    node.spin();

    return 0;
}