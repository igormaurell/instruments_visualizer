#include "CircleDetector/circle_detector.h"

CircleDetector::CircleDetector():
use_gaussian_filter(true),
gaussian_kernel_size(5),
dp(2.0f),
min_dist_div(4.0f),
hough_param1(200),
hough_param2(300),
min_radius(0),
max_radius(0)
{
    
}

std::vector<cv::Vec3f> CircleDetector::detect(const cv::Mat &image)
{
    cv::Mat gray;
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    if(use_gaussian_filter)
        cv::medianBlur(gray, gray, gaussian_kernel_size);
    
    std::vector<cv::Vec3f> circles;

    HoughCircles(gray, circles, cv::HOUGH_GRADIENT, dp,
                gray.rows/min_dist_div,
                hough_param1, hough_param2, 
                min_radius, max_radius);
    return circles;
}