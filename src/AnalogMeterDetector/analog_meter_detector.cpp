#include "AnalogMeterDetector/analog_meter_detector.h"

AnalogMeterDetector::AnalogMeterDetector():
border_ratio(0.8),
resolution(0.1)
{

}

double AnalogMeterDetector::getAngleBetween(cv::Point p1, cv::Point p2)
{
    double angle;
    angle = (atan2(p2.y - p1.y, p2.x - p1.x) * 180.0f / CV_PI);

	if (angle < .0f){
		angle = angle + 360.0f;
	}
    return angle;
}

std::vector<AnalogMeter> AnalogMeterDetector::detect(const cv::Mat &image)
{
    std::vector<AnalogMeter> analog_meters;

    cv::Mat gray;
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);

    std::vector<cv::Vec3f> circles;

    circles = circle_detector.detect(image);

    std::vector<cv::Vec3f>::iterator it;
    for(it = circles.begin() ; it != circles.end() ; it++) {
        cv::Point top_left((*it)[0] - (*it)[2],(*it)[1] - (*it)[2]);
        cv::Point botton_right((*it)[0] + (*it)[2],(*it)[1] + (*it)[2]);
        if(top_left.x>=0 && top_left.y>=0 && botton_right.x<=image.cols && botton_right.y<=image.rows){
            cv::Rect ROI(top_left, botton_right);
            cv::Mat crop = gray(ROI);

            int radius = (*it)[2];
            cv::Point new_center(radius, radius);
            double radius2 = pow(border_ratio*radius, 2); 
            double angle;
            int positions = int(360.0 / resolution);
            std::vector<int> intensity(positions, 0);
            std::vector<int> count(intensity.size(), 0);

            for (int i = 0 ; i < crop.cols ; i++){
                for (int j = 0 ; j < crop.rows ; j++){
                    angle = getAngleBetween(new_center, cv::Point(i, j));
                    int index = (int)(angle/resolution);
                    if (pow(new_center.x - i, 2) + pow(new_center.y - j, 2) <= radius2)
                    {
                        int pixel_intensity = crop.at<unsigned char>(i, j);
                        intensity[index] = intensity[index] + (255-pixel_intensity);
                        count[index]++;
                    }
                }
            }

            int max_value=-1;
            int max_index=-1;
            for(unsigned int i=0;i<intensity.size()-1;i++){
                if(count[i]>5){
                    if(intensity[i]/count[i]>max_value){
                        max_value=intensity[i]/count[i];
                        max_index=i;
                    }
                }
            }

            AnalogMeter m;
            m.center=cv::Point((*it)[0],(*it)[1]);
            m.radius=radius;
            m.pointer_angle=max_index*resolution;
            analog_meters.push_back(m);
        }
    }
    return analog_meters;

}