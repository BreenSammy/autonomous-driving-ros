#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Header.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> // for conversions
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>

class OdomPublisher
{
public:
    OdomPublisher() : tf_buffer_(), tf_listener_(tf_buffer_)
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

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

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
        try
        {
            // geometry_msgs::TransformStamped tf;
            // tf = tf_buffer_.lookupTransform(
            //     "odom",            // target frame
            //     "world",           // source frame
            //     ros::Time(0),      // latest available transform
            //     ros::Duration(1.0) // wait up to 1 second
            // );
            // geometry_msgs::TwistStamped twist_in, twist_out;
            // twist_in.header = last_pose_header_; // use pose timestamp for consistency
            // twist_in.twist = twist_;
            // twist_out = transformTwist(twist_in, tf);
            // Transform pose
            // geometry_msgs::PoseStamped pose_in, pose_out;
            // pose_in.header = last_pose_header_;
            // pose_in.pose = pose_;

            // pose_out = tf_buffer_.transform(pose_in, "world_map");

            // Build odometry message with transformed pose/twist
            tf2::Quaternion q;
            tf2::fromMsg(pose_.orientation, q);
            tf2::Matrix3x3 m(q);
            tf2::Vector3 world_vel(twist_.linear.x, twist_.linear.y, 0);
            tf2::Vector3 local_vel = m.inverse() * world_vel;

            nav_msgs::Odometry odom;

            odom.header.stamp = last_pose_header_.stamp;
            odom.header.frame_id = "odom";

            odom.child_frame_id = "base_link";

            odom.pose.pose = pose_;
            odom.twist.twist.linear.x = local_vel.x();
            odom.twist.twist.linear.y = local_vel.y();
            odom.twist.twist.angular.z = twist_.angular.z;

            odom_pub_.publish(odom);
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN_THROTTLE(1.0, "Could not transform pose to world frame: %s", ex.what());
            return;
        }
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