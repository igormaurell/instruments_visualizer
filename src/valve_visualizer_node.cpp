#include "ros/ros.h"

#include <vector>

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

bool verifyValveState(cv::Mat& image)
{
    cv::Mat img, image_gray, image_hsv;
    image.copyTo(img);

    cv::cvtColor(image, image_gray, CV_BGR2GRAY);
    cv::cvtColor(image, image_hsv, CV_BGR2HSV);

    int hsv[] = {120, 190};
    int thresh_hsv[] = {20, 30};

    cv::Scalar min_hsv = cv::Scalar(hsv[0] - thresh_hsv[0], hsv[1] - thresh_hsv[1], 0); 
    cv::Scalar max_hsv = cv::Scalar(hsv[0] + thresh_hsv[0], hsv[1] + thresh_hsv[1], 255);

    cv::Mat mask_hsv, result_hsv, result_gray, th1;
    cv::inRange(image_hsv, min_hsv, max_hsv, mask_hsv);
    cv::bitwise_and(image_hsv, image_hsv, result_hsv, mask_hsv);
 
    cv::cvtColor(result_hsv, result_gray, CV_HSV2BGR);
    cv::cvtColor(result_gray, result_gray, CV_BGR2GRAY);

    cv::threshold(result_gray, th1, 1, 255, CV_THRESH_BINARY);

    std::vector<std::vector<cv::Point> > contours1;
    std::vector<cv::Point> contour1;

    cv::findContours(th1, contours1, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);

    std::vector<std::vector<cv::Point> >::iterator it;

    for(it = contours1.begin() ; it != contours1.end() ; it++) {
        if(contour1.size() == 0 || cv::contourArea(*it) > cv::contourArea(contour1)) {
            contour1 = *it;
        }
    }

    contours1 = std::vector<std::vector<cv::Point> >{contour1};

    cv::drawContours(th1, contours1, 0, 255, -1);

    cv::RotatedRect rect_mob;
    rect_mob = cv::minAreaRect(contour1);
    cv::Point2f vertices1[4];
    rect_mob.points(vertices1);
    for (int i = 0; i < 4; i++)
        cv::line(img, vertices1[i], vertices1[(i+1)%4], cv::Scalar(0,255,0), 2);

    
    cv::Mat blur, th2, th1_inv, kernel;
    cv::GaussianBlur(image_gray, blur, cv::Size(5, 5), 0);

    cv::threshold(blur, th2, 0, 255, CV_THRESH_OTSU);

    cv::bitwise_not(th2, th2);

    cv::bitwise_not(th1, th1_inv);
    cv::bitwise_and(th2, th1_inv, th2);

    kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(11, 11));
    cv::morphologyEx(th2, th2, cv::MORPH_OPEN, kernel);

    std::vector<std::vector<cv::Point> > contours2;
    std::vector<cv::Point> contour2;
    cv::findContours(th2, contours2, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

    std::vector<std::vector<cv::Point> >::iterator jt;
    for(jt = contours2.begin() ; jt != contours2.end() ; jt++) {
        if(contour2.size() == 0 || cv::contourArea(*jt) > cv::contourArea(contour2)) {
            contour2 = *jt;
        }
    }

    cv::RotatedRect rect_bod;
    std::vector<cv::Point> box_bod;
    rect_bod = cv::minAreaRect(contour2);
    cv::Point2f vertices2[4];
    rect_bod.points(vertices2);
    for (int i = 0; i < 4; i++)
        cv::line(img, vertices2[i], vertices2[(i+1)%4], cv::Scalar(0,0,255), 2);

    cv::imshow("image", img);
    cv::waitKey(0);
}

bool visualizeValve(instruments_visualizer::VisualizeValve::Request &req, instruments_visualizer::VisualizeValve::Response &res)
{
    ROS_INFO("Reading valve state...");
    
    sensor_msgs::Image image_msg;

    cv::Mat image;

    image = cv::imread("/home/igormaurell/Workspace/rcb/catkin_ws/src/instruments_visualizer/valvulas/valvula2.jpg");
    verifyValveState(image);

    /*for(int i = 0 ; ; i++) {
        image_msg = *(ros::topic::waitForMessage<sensor_msgs::Image>("/usb_cam/image_raw", ros::Duration(1))); 
        sensor_msgs::Image::ConstPtr image_const_ptr( new sensor_msgs::Image(image_msg));
        readImage(image_const_ptr, image);

        

    }*/

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