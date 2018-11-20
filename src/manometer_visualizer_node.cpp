#include "ros/ros.h"

#include <opencv2/opencv.hpp>

#include "std_msgs/Header.h"

#include "sensor_msgs/Image.h"

#include "instruments_visualizer/VisualizeManometer.h"

#include "AnalogMeterDetectorCLT.h"

std_msgs::Header state_header;

int NUM_READS = 50;

void readImage(const sensor_msgs::Image::ConstPtr& msg_image, cv::Mat &image)
{
    cv_bridge::CvImageConstPtr cv_image;
    cv_image = cv_bridge::toCvShare(msg_image, msg_image->encoding);
    cv_image->image.copyTo(image);
}

bool visualizeManometer(instruments_visualizer::VisualizeManometer::Request &req, instruments_visualizer::VisualizeManometer::Response &res)
{
    ROS_INFO("Reading manometer state...");
    
    instruments_visualizer::ManometerState &manometer_state = res.manometer_state;

    sensor_msgs::Image::ConstPtr image_ptr;

    cv::Mat image;

    for(int i = 0 ; i < NUM_READS ; i++) {
        image_ptr = ros::topic::waitForMessage<sensor_msgs::Image>("/usb_cam/image_raw", ros::Duration(10)); 
        readImage(image_ptr, image);

        
    }

    state_header.stamp = ros::Time::now();

    return true;
}


int main(int argc, char **argv)
{
    ros::NodeHandle nh;
    nh.init(argc, argv, "manometer_visualizer_node");
    

    VisionSense::AnalogMeterDetectorCLT amd;

    ros::ServiceServer service = nh.advertiseService("/instruments_visualizer/visualize_manometer", visualizeManometer);

    ros::spin();
    return 0;
}