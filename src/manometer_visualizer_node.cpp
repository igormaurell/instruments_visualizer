#include "ros/ros.h"
#include "math.h"

#include <opencv2/opencv.hpp>

#include <cv_bridge/cv_bridge.h>

#include "std_msgs/Header.h"

#include "sensor_msgs/Image.h"

#include "instruments_visualizer/VisualizeManometer.h"

#include "AnalogMeterDetectorCLT.h"

#define _USE_MATH_DEFINES

std_msgs::Header state_header;

VisionSense::AnalogMeterDetectorCLT amd;

int NUM_READS = 50;

void readImage(const sensor_msgs::Image::ConstPtr& msg_image, cv::Mat &image)
{
    cv_bridge::CvImageConstPtr cv_image;
    cv_image = cv_bridge::toCvShare(msg_image, "bgr8");
    cv_image->image.copyTo(image);
}

bool visualizeManometer(instruments_visualizer::VisualizeManometer::Request &req, instruments_visualizer::VisualizeManometer::Response &res)
{
    ROS_INFO("Reading manometer state...");
    
    instruments_visualizer::ManometerState &manometer_state = res.manometer_state;

    sensor_msgs::Image image_ptr;

    cv::Mat image;

    std::vector<VisionSense::AnalogMeter> analogs;

    for(int i = 0 ; ; i++) {
        image_ptr = *(ros::topic::waitForMessage<sensor_msgs::Image>("/usb_cam/image_raw", ros::Duration(1))); 
        sensor_msgs::Image::ConstPtr image_const_ptr( new sensor_msgs::Image(image_ptr));
        readImage(image_const_ptr, image);

        analogs = amd.detect(image);

        if(analogs.size() > 0){
            ROS_INFO("Measure: %lf", analogs[0].pointerAngle);
            cv::circle(image, analogs[0].center, analogs[0].radius, cv::Scalar(0, 0, 255), 3);
            cv::Point p2(analogs[0].center.x + analogs[0].radius*sin(analogs[0].pointerAngle*M_PI/180.0), analogs[0].center.y + analogs[0].radius*cos(analogs[0].pointerAngle*M_PI/180.0));
            cv::arrowedLine(image, analogs[0].center, p2, cv::Scalar(0, 0, 255), 2);
            cv::imshow("MANOMETER", image);
            cv::waitKey(1);
        }

    }

    state_header.stamp = ros::Time::now();

    return true;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "manometer_visualizer_node");
    
    ros::NodeHandle nh;



    ros::ServiceServer service = nh.advertiseService("/instruments_visualizer/visualize_manometer", visualizeManometer);

    ros::spin();
    return 0;
}