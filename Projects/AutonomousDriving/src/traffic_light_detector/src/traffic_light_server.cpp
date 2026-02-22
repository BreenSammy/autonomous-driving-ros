#include "traffic_light_detector/traffic_light_server.h"

TrafficLightDetector::TrafficLightDetector() : nh_{"~"}, it_{nh_}, rgb_received_{false}, semantic_received_{false} {
    rgb_sub_ = nh_.subscribe("/Unity_ROS_message_Rx/OurCar/Sensors/RGBCameraLeft/image_raw", 1, &TrafficLightDetector::rgbCallback, this);
    semantic_sub_ = nh_.subscribe("/Unity_ROS_message_Rx/OurCar/Sensors/SemanticCamera/image_raw", 1, &TrafficLightDetector::semanticCallback, this);
    cropped_rgb_pub_ = it_.advertise("/cropped_rgb_image", 1);
    cropped_semantic_pub_ = it_.advertise("/cropped_semantic_image", 1);
    ROS_INFO("Traffic Light Detector Initialized");
}

void TrafficLightDetector::rgbCallback(const sensor_msgs::ImageConstPtr& msg) {
    try {
        rgb_cv_ptr_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

        // Define the central area of the screen
        int width = rgb_cv_ptr_->image.cols;
        int height = rgb_cv_ptr_->image.rows;
        int central_width = width / 2;
        int central_height = height;
        int central_x = (width - central_width) / 1.5;

        cv::Rect centre(central_x, 0, central_width, central_height);
        rgb_cv_ptr_->image = rgb_cv_ptr_->image(centre);

        sensor_msgs::ImagePtr cropped_rgb_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", rgb_cv_ptr_->image).toImageMsg();
        cropped_rgb_pub_.publish(cropped_rgb_msg);

        rgb_received_ = true;
        processImages();
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

void TrafficLightDetector::semanticCallback(const sensor_msgs::ImageConstPtr& msg) {
    try {
        semantic_cv_ptr_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
        int width = semantic_cv_ptr_->image.cols;
        int height = semantic_cv_ptr_->image.rows;
        int central_width = width / 2;
        int central_height = height ;
        int central_x = (width - central_width) / 1.5;

        cv::Rect centre(central_x, 0, central_width, central_height);
        semantic_cv_ptr_->image = semantic_cv_ptr_->image(centre);

        sensor_msgs::ImagePtr cropped_semantic_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", semantic_cv_ptr_->image).toImageMsg();
        cropped_semantic_pub_.publish(cropped_semantic_msg);
        
        semantic_received_ = true;
        processImages();
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

void TrafficLightDetector::processImages() {
    if (rgb_received_ && semantic_received_) {
        // If sizes differ, resize the semantic image to match the RGB image
        if (rgb_cv_ptr_->image.size() != semantic_cv_ptr_->image.size()) {
            // Need to resize image
            cv::Mat resized_semantic;
            cv::resize(semantic_cv_ptr_->image, resized_semantic, rgb_cv_ptr_->image.size(), 0, 0, cv::INTER_NEAREST);
            semantic_cv_ptr_->image = resized_semantic;
        }

        // Clear flags so next pair can be processed
        rgb_received_ = false;
        semantic_received_ = false;
    }
}

std::string TrafficLightDetector::detectTrafficLight(const cv::Mat& rgb_img, const cv::Mat& semantic_img) {
    cv::Mat traffic_light_mask;
    cv::inRange(semantic_img, cv::Scalar(200), cv::Scalar(255), traffic_light_mask);
    cv::Mat masked_rgb;
    rgb_img.copyTo(masked_rgb, traffic_light_mask);
    
    cv::Mat red_mask, yellow_mask, green_mask;
    cv::inRange(masked_rgb, cv::Scalar(0, 0, 250), cv::Scalar(120, 120, 255), red_mask);
    cv::inRange(masked_rgb, cv::Scalar(62, 250, 250), cv::Scalar(116, 255, 255), yellow_mask);
    cv::inRange(masked_rgb, cv::Scalar(0, 250, 0), cv::Scalar(120, 255, 140), green_mask);

    int red_pixels = cv::countNonZero(red_mask);
    int yellow_pixels = cv::countNonZero(yellow_mask);
    int green_pixels = cv::countNonZero(green_mask);

    if (red_pixels > 0 && red_pixels > green_pixels && red_pixels > yellow_pixels) {
        ROS_INFO("Detected RED light");
        return "STOP";
    } else if (green_pixels > 0 && green_pixels > red_pixels && green_pixels > yellow_pixels) {
        ROS_INFO("Detected GREEN light");
        return "GO";
    } else if (yellow_pixels > 1) {
        ROS_INFO("Detected YELLOW light");
        return "CAUTION";
    }

    ROS_INFO("No traffic light detected");
    return "GO";
}

bool TrafficLightDetector::detectTrafficLightService(traffic_light_detector::DetectTrafficLight::Request &req, traffic_light_detector::DetectTrafficLight::Response &res) {
    if (rgb_cv_ptr_ && semantic_cv_ptr_) {
        // Ensure semantic mask matches RGB image size. If not, resize semantic to RGB.
        cv::Mat rgb_img = rgb_cv_ptr_->image;
        cv::Mat semantic_img = semantic_cv_ptr_->image;
        if (rgb_img.size() != semantic_img.size()) {
            cv::Mat resized_semantic;
            cv::resize(semantic_img, resized_semantic, rgb_img.size(), 0, 0, cv::INTER_NEAREST);
            semantic_img = resized_semantic;
        }

        std::string command = detectTrafficLight(rgb_img, semantic_img);
        res.status = command;
        return true;
    } else {
        return false;
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "traffic_light_server");
    TrafficLightDetector detector;
    ros::NodeHandle nh;

    ros::ServiceServer service = nh.advertiseService("detect_traffic_light", &TrafficLightDetector::detectTrafficLightService, &detector);
    ROS_INFO("Traffic Light Detector Service Ready.");
    ros::spin();

    return 0;
}

