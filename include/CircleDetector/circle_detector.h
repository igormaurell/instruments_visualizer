#include <vector>

#include "opencv2/opencv.hpp"

class CircleDetector {
    public:
        CircleDetector();

        std::vector<cv::Vec3f> detect(const cv::Mat &image);


        bool use_gaussian_filter;
        int gaussian_kernel_size;

        double dp;
        double min_dist_div;
        double hough_param1;
        double hough_param2;
        int min_radius;
        int max_radius;
};