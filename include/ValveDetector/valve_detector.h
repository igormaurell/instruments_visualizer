#include <vector>

#include <opencv2/opencv.hpp>

class ValveDetector {
    public:
        ValveDetector();

        std::vector<int> mobile_hs;
        std::vector<int> mobile_hs_thresh;

        bool use_gaussian_filter;
        int gaussian_kernel_size;

        int opening_kernel_size;


        std::pair<cv::RotatedRect, cv::RotatedRect> detect(const cv::Mat& image);
};