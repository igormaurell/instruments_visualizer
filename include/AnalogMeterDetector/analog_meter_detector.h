#include <vector>

#include <opencv2/opencv.hpp>

#include "CircleDetector/circle_detector.h"

struct AnalogMeter {
	double measure;
	double pointer_angle;
	cv::Point center;
    double radius;
};

class AnalogMeterDetector {
    public:
        AnalogMeterDetector();
    
        double border_ratio;
	    
        double resolution;

        CircleDetector circle_detector;

        std::vector<AnalogMeter> detect(const cv::Mat &image);
    
    private:
        double getAngleBetween(cv::Point p1, cv::Point p2);
};