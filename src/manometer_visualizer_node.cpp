#include "ros/ros.h"
#include <cv_bridge/cv_bridge.h>
#include "std_msgs/Header.h"
#include "sensor_msgs/Image.h"
#include "instruments_visualizer/VisualizeManometer.h"

#include <vector>
#include <algorithm>
#include <string>
#include <algorithmmath.h>

#include <opencv2/opencv.hpp>

#include "AnalogMeterDetector/analog_meter_detector.h"

#define _USE_MATH_DEFINES

AnalogMeterDetector amd;

int NUM_READS = 50;

bool DEBUG = true;

double MIN_MEASURE = 0.0, MIN_MEASURE_ANGLE = 314.0;
double MAX_MEASURE = 10.0, MAX_MEASURE_ANGLE = 39.1;

std::string camera_topic;

int seq;

void readImage(const sensor_msgs::Image::ConstPtr& msg_image, cv::Mat &image)
{
    cv_bridge::CvImageConstPtr cv_image;
    cv_image = cv_bridge::toCvShare(msg_image, "bgr8");
    cv_image->image.copyTo(image);
}

double doStatistics(std::vector<double> measures)
{
    std::sort(measures.begin(), measures.end());

    double value;

    if(measures.size()%2 == 0) {
        value = (measures[measures.size()/2] + measures[measures.size()/2 - 1])/2; 
    }
    else{
        value = measures[measures.size()/2];
    }

    return value;
}

bool visualizeManometer(instruments_visualizer::VisualizeManometer::Request &req, instruments_visualizer::VisualizeManometer::Response &res)
{
    ROS_INFO("READING MANOMETER STATE...");
    
    instruments_visualizer::ManometerState &manometer_state = res.manometer_state;

    sensor_msgs::Image image_msg;

    cv::Mat image;

    std::vector<AnalogMeter> manometers;
    std::vector<AnalogMeter>::iterator it;

    std::vector<double> measures;

    for(int i = 0 ; i < NUM_READS ; i++) {
        image_msg = *(ros::topic::waitForMessage<sensor_msgs::Image>("/usb_cam/image_raw", ros::Duration(1))); 
        sensor_msgs::Image::ConstPtr image_const_ptr( new sensor_msgs::Image(image_msg));
        readImage(image_const_ptr, image);

        manometers = amd.detect(image);

        AnalogMeter manometer;
        manometer.radius = 0;
        ROS_INFO("%d", manometers.size());
        if(manometers.size() > 0){
            for(it = manometers.begin() ; it != manometers.end() ; it++) {
                if(it->radius > manometer.radius) {
                    manometer = *it;
                }
            }

            ROS_INFO("POINTER ANGLE: %lf", manometer.pointer_angle);
            measures.push_back(manometer.pointer_angle);
            if(DEBUG){
                cv::circle(image, manometer.center, manometer.radius, cv::Scalar(0, 0, 255), 3);
                cv::Point p2(manometer.center.x + manometer.radius*sin(manometer.pointer_angle*M_PI/180.0), manometer.center.y + manometer.radius*cos(manometer.pointer_angle*M_PI/180.0));
                cv::arrowedLine(image, manometer.center, p2, cv::Scalar(0, 0, 255), 2);
                cv::imshow("MANOMETER", image);
                cv::waitKey(30);
            }
        }
        else{
            ROS_ERROR("IMAGE HAS NO MANOMETERS!");
        }

    }

    if(measures.size() == 0) {
        ROS_ERROR("NO MEASUREMENTS WERE TAKEN!");
        return false;
    }

    cv::destroyAllWindows();

    double measure;

    measure = doStatistics(measures);
    
    ROS_INFO("POINTER FINAL ANGLE: %lf", measure);

    measure = ((measure - MIN_MEASURE_ANGLE)*(MAX_MEASURE))/(MAX_MEASURE_ANGLE-MIN_MEASURE_ANGLE);
    measure += MIN_MEASURE;

    ROS_INFO("MEASURE: %lf", measure);

    res.manometer_state.header.seq = seq++;
    res.manometer_state.header.stamp = ros::Time::now();
    res.manometer_state.state = measure;

    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "manometer_visualizer_node");
    
    ros::NodeHandle nh;

    std::string manometer_visualizer_service;

    node_handle.param("/instuments_visualizer/subscribers/image_raw/topic", camera_topic, std::string("/usb_cam/image_raw"));
    node_handle.param("/instuments_visualizer/servers/manometer_visualizer/service", manometer_visualizer_service, std::string("/instruments_visualizer/visualize_manometer"));
    node_handle.param("/instuments_visualizer/num_reads", NUM_READS, 50);
    node_handle.param("/instuments_visualizer/debug", DEBUG, false);
    node_handle.param("/instuments_visualizer/min_measure", MIN_MEASURE, (double) 0.0);
    node_handle.param("/instuments_visualizer/min_measure_angle", MIN_MEASURE_ANGLE, (double) 314.0);
    node_handle.param("/instuments_visualizer/max_measure", MAX_MEASURE, (double) 10.0);
    node_handle.param("/instuments_visualizer/max_measure_angle", MAX_MEASURE_ANGLE, (double) 39.1);
    node_handle.param("/instuments_visualizer/analog_meter_detector/border_ratio", amd.border_ratio, (double) 0.8);
    node_handle.param("/instuments_visualizer/analog_meter_detector/resolution", amd.resolution, (double) 0.1);
    node_handle.param("/instuments_visualizer/analog_meter_detector/circle_detector/use_gaussian_filter", amd.circle_detector.use_gaussian_filter, true);
    node_handle.param("/instuments_visualizer/analog_meter_detector/circle_detector/gaussian_kernel_size", amd.circle_detector.gaussian_kernel_size, 5);
    node_handle.param("/instuments_visualizer/analog_meter_detector/circle_detector/dp", amd.circle_detector.dp, (double) 2.0);
    node_handle.param("/instuments_visualizer/analog_meter_detector/circle_detector/min_dist_div", amd.circle_detector.min_dist_div, (double)  4.0);
    node_handle.param("/instuments_visualizer/analog_meter_detector/circle_detector/hough_param1", amd.circle_detector.hough_param1, 200);
    node_handle.param("/instuments_visualizer/analog_meter_detector/circle_detector/hough_param2", amd.circle_detector.hough_param2, 300);
    node_handle.param("/instuments_visualizer/analog_meter_detector/circle_detector/min_radius", amd.circle_detector.min_radius, 0);
    node_handle.param("/instuments_visualizer/analog_meter_detector/circle_detector/max_radius", amd.circle_detector.max_radius, 0);

    ros::ServiceServer service = nh.advertiseService(manometer_visualizer_service, visualizeManometer);

    ros::spin();
    return 0;
}