#include <vector>

#include <opencv2/opencv.hpp>

#include "CircleDetector/circle_detector.h"

class PainelDetector {
    public:
        PainelDetector();

        std::vector<int> on_color_hs;
        std::vector<int> on_color_thresh;

        std::vector<int> off_color_hs;
        std::vector<int> off_color_thresh;

        int led_on_v;
        int led_on_thresh;

        CircleDetector circle_detector;

        std::vector<bool> detect(const cv::Mat &image);
}