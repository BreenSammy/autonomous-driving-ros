/*
 * Based on the DepthImageToLaserScan.cpp from the ROS package ros-perception
 * https://github.com/ros-perception/depthimage_to_laserscan/blob/melodic-devel/src/DepthImageToLaserScan.cpp
*/

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Image.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/image_encodings.h>
#include <image_geometry/pinhole_camera_model.h>
#include <sstream>
#include <limits.h>
#include <math.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>


class OdomRotatorNode {
public:
    OdomRotatorNode() {
        ros::NodeHandle nh_;
        sub_semantic_ = nh_.subscribe("/unity_ros/OurCar/Sensors/SemanticCamera/image_raw", 2, &OdomRotatorNode::semanticCallback, this);
        sub_depth_ = nh_.subscribe("/realsense/depth/camera_info", 10, &OdomRotatorNode::depthInfoCallback, this);
        sub_depth_info_ = nh_.subscribe("/realsense/depth/image", 10, &OdomRotatorNode::depthCallback, this);
        pub_ = nh_.advertise<sensor_msgs::LaserScan>("scan", 2);
    }

    void semanticCallback(const sensor_msgs::Image& msg) {
        semantic_image_ = msg;
        semanitic_image_initialized_ = true;
    }

    void depthInfoCallback(const sensor_msgs::CameraInfo& info_msg) {
        info_msg_ = info_msg;
        info_msg_initialized_ = true;
    }

    void depthCallback(const sensor_msgs::Image& depth_msg) {
        if(!semanitic_image_initialized_ || !info_msg_initialized_){
            std::cout << "depth received but no semantic stored" << std::endl;
            return;
        }
        depth_image_ = depth_msg;
        // std::cout << "depth received" << std::endl;
        auto semantic_cv_ptr = cv_bridge::toCvCopy(semantic_image_, sensor_msgs::image_encodings::RGB8);
        auto depth_cv_ptr = cv_bridge::toCvCopy(depth_msg, "");
        generateLaserScan(depth_cv_ptr->image, semantic_cv_ptr->image, info_msg_);
    }

    double magnitude_of_ray(const cv::Point3d& ray) const{
        return sqrt(pow(ray.x, 2.0) + pow(ray.y, 2.0) + pow(ray.z, 2.0));
    }

    double angle_between_rays(const cv::Point3d& ray1, const cv::Point3d& ray2) const{
            const double dot_product = ray1.x*ray2.x + ray1.y*ray2.y + ray1.z*ray2.z;
            const double magnitude1 = magnitude_of_ray(ray1);
            const double magnitude2 = magnitude_of_ray(ray2);;
            return acos(dot_product / (magnitude1 * magnitude2));
    }

    void generateLaserScan(const cv::Mat& depth_image, const cv::Mat& semantic_image, const sensor_msgs::CameraInfo& info_msg) {
        cam_model_.fromCameraInfo(info_msg);
        int width = depth_image.cols;
        int height = depth_image.rows;
        const float center_x = cam_model_.cx();
        const float center_y = cam_model_.cy();
        const double unit_scaling = 0.001;
        const float constant_x = unit_scaling / cam_model_.fx();
        const float constant_y = unit_scaling / cam_model_.fx();

        // Calculate angle_min and angle_max by measuring angles between the left ray, right ray, and optical center ray
        cv::Point2d raw_pixel_left(0, cam_model_.cy());
        cv::Point2d rect_pixel_left = cam_model_.rectifyPoint(raw_pixel_left);
        cv::Point3d left_ray = cam_model_.projectPixelTo3dRay(rect_pixel_left);

        cv::Point2d raw_pixel_right(depth_image.cols-1, cam_model_.cy());
        cv::Point2d rect_pixel_right = cam_model_.rectifyPoint(raw_pixel_right);
        cv::Point3d right_ray = cam_model_.projectPixelTo3dRay(rect_pixel_right);

        cv::Point2d raw_pixel_center(cam_model_.cx(), cam_model_.cy());
        cv::Point2d rect_pixel_center = cam_model_.rectifyPoint(raw_pixel_center);
        cv::Point3d center_ray = cam_model_.projectPixelTo3dRay(rect_pixel_center);

        const double angle_max = angle_between_rays(left_ray, center_ray);
        const double angle_min = -angle_between_rays(center_ray, right_ray); // Negative because the laserscan message expects an opposite rotation of that from the depth image

        sensor_msgs::LaserScan scan_msg;
        scan_msg.header.stamp = ros::Time::now();
        scan_msg.header.frame_id = "depth_camera";
        scan_msg.header.seq = depth_image_.header.seq;

        scan_msg.angle_min = angle_min; 
        scan_msg.angle_max = angle_max;
        scan_msg.angle_increment = (angle_max - angle_min) / (width - 1);
        scan_msg.time_increment = 0.0;
        scan_msg.scan_time = 1.0/30.0;
        scan_msg.range_min = range_min_;
        scan_msg.range_max = range_max_;

        cv::Mat semantic_road_mask;
        cv::inRange(semantic_image, cv::Scalar(127, 127, 127), cv::Scalar(129, 129, 129), semantic_road_mask);

        scan_msg.ranges.resize(width);

        double fx =  cam_model_.fx();
        for(int u = 0; u < width; u++){
            double range= 10000.0;
            auto depth_col = depth_image.col(u);
            auto mask_col = semantic_road_mask.col(u);
            const double th = -atan2((double)(u - center_x) , fx);
            const int index = (th - angle_min) / scan_msg.angle_increment;

            for(int v = height*0.40; v < height; v++){
                auto depth = depth_col.at<uint16_t>(v);
                if(mask_col.at<uchar>(v) > 0 || depth <= 0 || !std::isfinite(depth)){
                    continue;
                }
                // Handle random miscategorized pixels
                int road_neighbors = 0;
                if(u > 0 && semantic_road_mask.at<uint16_t>(u-1, v) > 0){
                    road_neighbors++;
                }
                if(u < width-1 && semantic_road_mask.at<uint16_t>(u+1, v) > 0){
                    road_neighbors++;
                }
                if(v > 0  && mask_col.at<uint16_t>(v-1) > 0){
                    road_neighbors++;
                }
                if(v < height-1 && mask_col.at<uint16_t>(v+1) > 0){
                    road_neighbors++;
                }
                if(road_neighbors >= 3){
                    continue;
                }
                double r = depth; // Assign to pass through NaNs and Infs

                if (std::isfinite(depth) && depth != 0){ // Not NaN or Inf
                    // Calculate in XYZ
                    double x = (u - center_x) * depth * constant_x;
                    double z = depth * unit_scaling;

                    // Calculate actual distance
                    r = hypot(x, z);
                    range = std::min(range, r);
                }
            }
            if(range < 1000){
                scan_msg.ranges[index] = range;
            }
            else{
                scan_msg.ranges[index] = std::numeric_limits<float>::quiet_NaN();
            }
        }

        pub_.publish(scan_msg);
    }

private:
    bool semanitic_image_initialized_{false}; 
    bool info_msg_initialized_{false}; 
    ros::Subscriber sub_semantic_;
    ros::Subscriber sub_depth_;
    ros::Subscriber sub_depth_info_;
    ros::Publisher pub_;
    sensor_msgs::Image semantic_image_;
    sensor_msgs::Image depth_image_;
    sensor_msgs::CameraInfo info_msg_;
    image_geometry::PinholeCameraModel cam_model_;

    float range_min_{0.1}; 
    float range_max_{30.0};
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "semantic_depth_to_laserscan");
    OdomRotatorNode node;
    ros::spin();
    return 0;
}