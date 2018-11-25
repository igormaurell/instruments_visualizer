#include "ros/ros.h"
#include <cv_bridge/cv_bridge.h>
#include "std_msgs/Header.h"
#include "sensor_msgs/Image.h"
#include "instruments_visualizer/VisualizeValve.h"

#include <iostream>
#include <vector>
#include <string>
#include <math.h>
#include <algorithm>

#include <opencv2/opencv.hpp>

#include "ValveDetector/valve_detector.h"

ValveDetector vd;

std_msgs::Header state_header;

int NUM_READS = 50;

int OPEN_THRESH = 30;

bool DEBUG = true;

void readImage(const sensor_msgs::Image::ConstPtr& msg_image, cv::Mat &image)
{
    cv_bridge::CvImageConstPtr cv_image;
    cv_image = cv_bridge::toCvShare(msg_image, "bgr8");
    cv_image->image.copyTo(image);
}

bool doStatistics(std::vector<bool> measures)
{
    int trues = std::count(measures.begin(), measures.end(), true);
    int falses = std::count(measures.begin(), measures.end(), false);

    return trues>=falses;
}

bool visualizeValve(instruments_visualizer::VisualizeValve::Request &req, instruments_visualizer::VisualizeValve::Response &res)
{
    ROS_INFO("Reading valve state...");
    
    sensor_msgs::Image image_msg;

    cv::Mat image;

    bool open;
    std::vector<bool> measures;
    for(int i = 0 ; ; i++) {
        image_msg = *(ros::topic::waitForMessage<sensor_msgs::Image>("/usb_cam/image_raw", ros::Duration(1))); 
        sensor_msgs::Image::ConstPtr image_const_ptr( new sensor_msgs::Image(image_msg));
        readImage(image_const_ptr, image);

        std::pair<cv::RotatedRect, cv::RotatedRect> rects;
        rects = vd.detect(image);

        cv::RotatedRect rect_bod = rects.first;
        cv::RotatedRect rect_mob = rects.second;

        double angle_mob, angle_bod;

        if(rect_mob.size.width < rect_mob.size.height)
            angle_mob = rect_mob.angle - 90;
        else
            angle_mob = rect_mob.angle;

        if(rect_bod.size.width < rect_bod.size.height)
            angle_bod = rect_bod.angle - 90;
        else
            angle_bod = rect_bod.angle;

        open = (abs(angle_bod - angle_mob) < OPEN_THRESH);

        ROS_INFO("BODY ANGLE : %lf", angle_bod);
        ROS_INFO("MOBILE ANGLE: %lf", angle_mob);
        ROS_INFO("OPEN: %d", open);

        if(DEBUG){
            cv::Point2f vertices1[4];
            rect_mob.points(vertices1);
            for (int i = 0; i < 4; i++)
            cv::line(image, vertices1[i], vertices1[(i+1)%4], cv::Scalar(0,255,0), 2);

            cv::Point2f vertices2[4];
            rect_bod.points(vertices2);
            for (int i = 0; i < 4; i++){
                cv::line(image, vertices2[i], vertices2[(i+1)%4], cv::Scalar(0,0,255), 2);
            }

            
            double angle_mob, angle_bod;

            if(rect_mob.size.width < rect_mob.size.height)
                angle_mob = rect_mob.angle - 90;
            else
                angle_mob = rect_mob.angle;

            if(rect_bod.size.width < rect_bod.size.height)
                angle_bod = rect_bod.angle - 90;
            else
                angle_bod = rect_bod.angle;

            ROS_INFO("MOB: %lf", angle_mob);
            ROS_INFO("BOD: %lf", angle_bod);

            cv::imshow("image", image);
            cv::waitKey(30);
        }

        measures.push_back(open);
    
    }

    if(measures.size() == 0) {
        ROS_ERROR("NO MEASUREMENTS WERE TAKEN!");
        return false;
    }

    open = doStatistics(measures);

    state_header.stamp = ros::Time::now();

    res.valve_state.header = state_header;
    res.valve_state.state = open;

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