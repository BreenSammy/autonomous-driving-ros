#ifndef TRAFFIC_LIGHT_SERVER_H
#define TRAFFIC_LIGHT_SERVER_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <std_msgs/String.h>
#include <unordered_map>
#include <image_transport/image_transport.h>
#include <traffic_light_detector/TrafficLight.h>
#include <traffic_light_detector/DetectTrafficLight.h>

class TrafficLightDetector {
public:
    TrafficLightDetector();
    bool detectTrafficLightService(traffic_light_detector::DetectTrafficLight::Request &req, traffic_light_detector::DetectTrafficLight::Response &res);

private:
    ros::NodeHandle nh_;
    ros::Subscriber rgb_sub_;
    ros::Subscriber semantic_sub_;
    image_transport::ImageTransport it_;
    image_transport::Publisher cropped_rgb_pub_;
    image_transport::Publisher cropped_semantic_pub_;
    cv_bridge::CvImagePtr rgb_cv_ptr_;
    cv_bridge::CvImagePtr semantic_cv_ptr_;
    bool rgb_received_;
    bool semantic_received_;

    void rgbCallback(const sensor_msgs::ImageConstPtr& msg);
    void semanticCallback(const sensor_msgs::ImageConstPtr& msg);
    void processImages();
    std::string detectTrafficLight(const cv::Mat& rgb_img, const cv::Mat& semantic_img);
};

#endif // TRAFFIC_LIGHT_SERVER_H

