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

std::vector<uint8_t> doStatistics(std::vector<std::vector<uint8_t> >& measures)
{
    std::vector<uint8_t> measure;
    
    std::vector<int> on_counter(measures[0].size(), 0);
    std::vector<std::vector<uint8_t> >::iterator it;
    std::vector<uint8_t>::iterator kt;
    std::vector<int>::iterator jt;
    for(it = measures.begin() ; it != measures.end() ; it++) {
        for(kt = it->begin(), jt = on_counter.begin() ; kt != it->end() ; kt++, jt++){
            if(*kt) (*jt)++;
        }
    }

    for(jt = on_counter.begin() ; jt != on_counter.end() ; jt++){
        if(*jt >= (measures.size()/2))
            measure.push_back(true);
        else 
            measure.push_back(false);
    }

    return measure;
}

bool visualizePainel(instruments_visualizer::VisualizePainel::Request &req, instruments_visualizer::VisualizePainel::Response &res)
{
    ROS_INFO("READING PAINEL STATE...");
    
    instruments_visualizer::PainelState &painel_state = res.painel_state;

    sensor_msgs::Image image_msg;

    cv::Mat image;

    std::vector<std::pair<cv::Vec3f, uint8_t> > detection;
    std::vector<std::vector<uint8_t> > measures;
    std::vector<uint8_t> measure;
    std::vector<std::pair<cv::Vec3f, uint8_t> >::iterator it;
    for(int i = 0 ; i < NUM_READS ; i++) {
        image_msg = *(ros::topic::waitForMessage<sensor_msgs::Image>(camera_topic, ros::Duration(1))); 
        sensor_msgs::Image::ConstPtr image_const_ptr( new sensor_msgs::Image(image_msg));
        readImage(image_const_ptr, image);

        detection = pd.detect(image);

        if(DEBUG){
            int i;
            for(it = detection.begin(), i = 0 ; it != detection.end() ; it++, i++){
                cv::Vec3f c1;
                c1 = it->first;
                measure.push_back(it->second);
                cv::circle(image, cv::Point(c1[0], c1[1]), c1[2], cv::Scalar(255, 0, 0), 3);
                cv::putText(image, std::to_string(i), cv::Point(c1[0], c1[1]), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 0, 0));
            }
            cv::imshow("PAINEL", image);
            cv::waitKey(30);
        }

        measures.push_back(measure);
        measure.clear();
    }

    if(measures.size() == 0) {
        ROS_ERROR("NO MEASUREMENTS WERE TAKEN!");
        return false;
    }

    cv::destroyAllWindows();

    measure = doStatistics(measures);
    

    res.painel_state.header.seq = seq++;
    res.painel_state.header.stamp = ros::Time::now();
    res.painel_state.state = measure;

    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "painel_visualizer_node");
    
    ros::NodeHandle node_handle;

    std::string painel_visualizer_service;

    node_handle.param("/instruments_visualizer/subscribers/image_raw/topic", camera_topic, std::string("/usb_cam/image_raw"));
    node_handle.param("/instruments_visualizer/servers/painel_visualizer/service", painel_visualizer_service, std::string("/instruments_visualizer/visualize_painel"));
    node_handle.param("/instruments_visualizer/painel_visualizer/num_reads", NUM_READS, 10);
    node_handle.param("/instruments_visualizer/painel_visualizer/debug", DEBUG, true);
    node_handle.param("/instruments_visualizer/painel_visualizer/painel_detector/circuits_number", pd.circuits_number, 3);
    node_handle.param("/instruments_visualizer/painel_visualizer/painel_detector/min_radius", pd.min_radius, 10.0);
    node_handle.param("/instruments_visualizer/painel_visualizer/painel_detector/max_radius", pd.max_radius, 80.0);
    node_handle.param("/instruments_visualizer/painel_visualizer/painel_detector/upper_state", pd.upper_state, std::string("on"));
    node_handle.param("/instruments_visualizer/painel_visualizer/painel_detector/on_color_hsv", pd.on_color_hsv, std::vector<int>{89, 187, 56});
    node_handle.param("/instruments_visualizer/painel_visualizer/painel_detector/on_color_thresh", pd.on_color_thresh, std::vector<int>{15, 67, 56});
    node_handle.param("/instruments_visualizer/painel_visualizer/painel_detector/off_color_hsv", pd.off_color_hsv, std::vector<int>{23, 212, 106});
    node_handle.param("/instruments_visualizer/painel_visualizer/painel_detector/off_color_thresh", pd.off_color_thresh, std::vector<int>{23, 42, 22});
    node_handle.param("/instruments_visualizer/painel_visualizer/painel_detector/led_on_v", pd.led_on_v, 237);
    node_handle.param("/instruments_visualizer/painel_visualizer/painel_detector/led_on_thresh", pd.led_on_thresh, 17);
    node_handle.param("/instruments_visualizer/painel_visualizer/painel_detector/closing_kernel_size", pd.closing_kernel_size, 9);

    ros::ServiceServer service = node_handle.advertiseService(painel_visualizer_service, visualizePainel);

    ros::spin();
    return 0;
}