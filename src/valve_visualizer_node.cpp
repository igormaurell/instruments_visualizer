#include "ros/ros.h"

#include <opencv2/opencv.hpp>

#include <cv_bridge/cv_bridge.h>

#include "std_msgs/Header.h"

#include "sensor_msgs/Image.h"

#include "instruments_visualizer/VisualizeValve.h"

int NUM_READS = 50;

void readImage(const sensor_msgs::Image::ConstPtr& msg_image, cv::Mat &image)
{
    cv_bridge::CvImageConstPtr cv_image;
    cv_image = cv_bridge::toCvShare(msg_image, "bgr8");
    cv_image->image.copyTo(image);
}

bool visualizeValve(instruments_visualizer::VisualizeValve::Request &req, instruments_visualizer::VisualizeValve::Response &res)
{
    ROS_INFO("Reading manometer state...");
    
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "valve_visualizer_node");

    ros::NodeHandle nh;

    ros::ServiceServer service = nh.advertiseService("/instruments_visualizer/visualize_valve", visualizeValve);

    ros::spin();
    return 0;
}