#include <vector>

#include <opencv2/opencv.hpp>

class ValveDetector {
    public:
        ValveDetector();

        /*std::vector<int> mobile_hs;
        std::vector<int> mobile_hs_thresh;*/
        double extent_thresh;

        bool use_gaussian_filter;
        int gaussian_kernel_size;

        int closing_kernel_size;

        std::pair<std::vector<cv::Point>, bool> detect(const cv::Mat& image);

        //std::pair<cv::RotatedRect, cv::RotatedRect> detect(const cv::Mat& image);
};