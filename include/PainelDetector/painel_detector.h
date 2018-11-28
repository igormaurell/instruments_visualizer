#include <algorithm>
#include <vector>
#include <string>

#include <opencv2/opencv.hpp>

#include "CircleDetector/circle_detector.h"

class PainelDetector {
    public:
        PainelDetector();

        int circuits_number;

        double min_radius;
        double max_radius;

        bool upper_state;

        std::vector<int> on_color_hsv;
        std::vector<int> on_color_thresh;

        std::vector<int> off_color_hsv;
        std::vector<int> off_color_thresh;

        int led_on_v;
        int led_on_thresh;

        int closing_kernel_size;

        //CircleDetector circle_detector;

        void calculateCircles(const cv::Mat& image, std::vector<cv::Vec3f>& circles, std::string type);
        std::vector<std::pair<cv::Vec3f, uint8_t> >  detect(const cv::Mat &image);
};