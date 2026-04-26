#include <ros/ros.h>
#include <traffic_light_detector/DetectTrafficLight.h>
#include <traffic_light_detector/TrafficLight.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "traffic_light_client");
    ros::NodeHandle nh;

    // Create a service client to the 'detect_traffic_light' service
    ros::ServiceClient client = nh.serviceClient<traffic_light_detector::DetectTrafficLight>("detect_traffic_light");

    // Create a service request and response object
    traffic_light_detector::DetectTrafficLight srv;

    ros::Rate loop_rate(50);

    while (ros::ok()) {
        // Call the service
        if (client.call(srv)) {
            // Log the response from the service
            ROS_INFO("Traffic Light Status: %s", srv.response.status.c_str());
        } else {
            // Log an error if the service call fails
            ROS_ERROR("Failed to call service detect_traffic_light");
        }

        loop_rate.sleep();
    }

    return 0;
}
