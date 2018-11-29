#include <vector>
#include <string>

#include <opencv2/opencv.hpp>

class ValveDetector {
    public:
        ValveDetector();

        std::string type;  

        std::vector<int> min_vec_hsv;
        std::vector<int> max_vec_hsv;

        bool use_gaussian_filter;
        int gaussian_kernel_size;

        int canny_thresh;
        int erode_times;
        int dilate_times;
    
        int closing_kernel_size;

        void contourByTriangle(const cv::Mat& image, std::vector<cv::Point>& contour);
        void contourByHsv(const cv::Mat& image, std::vector<cv::Point>& contour);
        void contourByCanny(const cv::Mat& image, std::vector<cv::Point>& contour);

        std::pair<std::vector<cv::Point>, double> detect(const cv::Mat& image);

        //std::pair<cv::RotatedRect, cv::RotatedRect> detect(const cv::Mat& image);
};