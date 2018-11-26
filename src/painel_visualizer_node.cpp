#include "ros/ros.h"
#include <cv_bridge/cv_bridge.h>
#include "std_msgs/Header.h"
#include "sensor_msgs/Image.h"
#include "instruments_visualizer/VisualizePainel.h"

#include <vector>
#include <string>

#include <opencv2/opencv.hpp>

#include "PainelDetector/painel_detector.h"

PainelDetector pd;

int NUM_READS = 50;

bool DEBUG = true;

std::string camera_topic;

int seq;

void readImage(const sensor_msgs::Image::ConstPtr& msg_image, cv::Mat &image)
{
    cv_bridge::CvImageConstPtr cv_image;
    cv_image = cv_bridge::toCvShare(msg_image, "bgr8");
    cv_image->image.copyTo(image);
}

std::vector<bool> doStatistics(std::vector<std::vector<bool> > measures)
{
    std::vector<bool> measure;

    std::vector<int> on_counter(0, measures[0].size());
    std::vector<std::vector<bool> >::iterator it;
    std::vector<int>::iterator jt;
    for(it = measures.begin(), jt = on_counter.begin() ; it != measures.end() ; it++, jt++) {

    }

    return measure;
}

bool visualizePainel(instruments_visualizer::VisualizePainel::Request &req, instruments_visualizer::VisualizePainel::Response &res)
{
    ROS_INFO("READING PAINEL STATE...");
    
    instruments_visualizer::PainelState &painel_state = res.painel_state;

    sensor_msgs::Image image_msg;

    cv::Mat image;

    std::pair<std::vector<std::pair<cv::Vec3f, cv::Vec3f> >, std::vector<bool> > detection;
    std::vector<std::pair<cv::Vec3f, cv::Vec3f> > circuits;
    std::vector<std::vector<bool> > measures;

    std::vector<std::pair<cv::Vec3f, cv::Vec3f> >::iterator it;
    for(int i = 0 ; i < NUM_READS ; i++) {
        /*image_msg = *(ros::topic::waitForMessage<sensor_msgs::Image>(camera_topic, ros::Duration(1))); 
        sensor_msgs::Image::ConstPtr image_const_ptr( new sensor_msgs::Image(image_msg));
        readImage(image_const_ptr, image);*/

        image = cv::imread("/home/igormaurell/Workspace/rcb/catkin_ws/src/instruments_visualizer/images/painel.png");


        detection = pd.detect(image);

        circuits = detection.first;
        /*if(DEBUG){
            int i;
            for(it = circuits.begin(), i = 0 ; it != circuits.end() ; it++, i++){
                cv::Vec3f c1, c2;
                c1 = it->first;
                c2 = it->second;
                cv::circle(image, cv::Point(c1[0], c1[1]), c1[2], cv::Scalar(255, 0, 0), 3);
                cv::putText(image, std::to_string(i), cv::Point(c1[0], c1[1]), cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(255, 0, 0));
                cv::circle(image, cv::Point(c2[0], c2[1]), c2[2], cv::Scalar(255, 0, 0), 3);
                cv::putText(image, std::to_string(i), cv::Point(c2[0], c2[1]), cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(255, 0, 0));
            }
            cv::imshow("PAINEL", image);
            cv::waitKey(30);
        }*/

        measures.push_back(detection.second);
    }

    if(measures.size() == 0) {
        ROS_ERROR("NO MEASUREMENTS WERE TAKEN!");
        return false;
    }

    cv::destroyAllWindows();
    /*
    std::vector<bool> measure;

    //measure = doStatistics(measures);
    
    ROS_INFO("POINTER FINAL ANGLE: %lf", measure);

    measure = ((measure - MIN_MEASURE_ANGLE)*(MAX_MEASURE))/(MAX_MEASURE_ANGLE-MIN_MEASURE_ANGLE);
    measure += MIN_MEASURE;

    ROS_INFO("MEASURE: %lf", measure);

    res.manometer_state.header.seq = seq++;
    res.manometer_state.header.stamp = ros::Time::now();
    res.manometer_state.state = measure;*/

    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "painel_visualizer_node");
    
    ros::NodeHandle node_handle;

    std::string painel_visualizer_service;

    node_handle.param("/instruments_visualizer/painel_visualizer/subscribers/image_raw/topic", camera_topic, std::string("/usb_cam/image_raw"));
    node_handle.param("/instruments_visualizer/painel_visualizer/servers/painel_visualizer/service", painel_visualizer_service, std::string("/instruments_visualizer/visualize_painel"));
    node_handle.param("/instruments_visualizer/painel_visualizer/num_reads", NUM_READS, 50);
    node_handle.param("/instruments_visualizer/painel_visualizer/debug", DEBUG, true);
    node_handle.param("/instruments_visualizer/painel_visualizer/painel_detector/circle_detector/use_gaussian_filter", pd.circle_detector.use_gaussian_filter, true);
    node_handle.param("/instruments_visualizer/painel_visualizer/painel_detector/circle_detector/gaussian_kernel_size", pd.circle_detector.gaussian_kernel_size, 5);
    node_handle.param("/instruments_visualizer/painel_visualizer/painel_detector/circle_detector/dp", pd.circle_detector.dp, (double) 1.0);
    node_handle.param("/instruments_visualizer/painel_visualizer/painel_detector/circle_detector/min_dist_div", pd.circle_detector.min_dist_div, (double)  8.0);
    node_handle.param("/instruments_visualizer/painel_visualizer/painel_detector/circle_detector/hough_param1", pd.circle_detector.hough_param1, 200.0);
    node_handle.param("/instruments_visualizer/painel_visualizer/painel_detector/circle_detector/hough_param2", pd.circle_detector.hough_param2, 100.0);
    node_handle.param("/instruments_visualizer/painel_visualizer/painel_detector/circle_detector/min_radius", pd.circle_detector.min_radius, 0);
    node_handle.param("/instruments_visualizer/painel_visualizer/painel_detector/circle_detector/max_radius", pd.circle_detector.max_radius, 0);

    ros::ServiceServer service = node_handle.advertiseService(painel_visualizer_service, visualizePainel);

    ros::spin();
    return 0;
}