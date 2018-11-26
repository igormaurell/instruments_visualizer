#include "PainelDetector/painel_detector.h"

PainelDetector::PainelDetector():
//param inicialization
{

}

bool compByWidth(const cv::Vec3f& vec1, const cv::Vec3f& vec2)
{
    return vec1[0] < vec2[0] || (vec1[0] == vec2[0] && vec1[1] < vec2[1]);
}

bool compByRadius(const cv::Vec3f& vec1, const cv::Vec3f& vec2)
{
    return vec1[2] < vec2[2] || (vec1[2] == vec2[2] && vec1[0] < vec2[0]);
}

int PainelDetector::meanCircleValue(const cv::Mat& image_hsv, const cv::Vec3f& circle)
{
    int mean_value = 0;
    cv::Point top_left(circle[0] - circle[2], circle[1] - circle[2]);
    cv::Point botton_right(circle[0] + circle[2], circle[1] + circle[2]);
    if(top_left.x>=0 && top_left.y>=0 && botton_right.x<=image.cols && botton_right.y<=image.rows){
        cv::Rect ROI(top_left, botton_right);
        cv::Mat crop = image_hsv(ROI);

        for (int i = 0 ; i < crop.cols ; i++){
            for (int j = 0 ; j < crop.rows ; j++){
                mean_value += crop.at<unsigned char>(i, j)[2];
            }
        }
        mean_value /= (crop.cols*crop.rows);
    }
    return mean_value;
}


    
}

std::vector<bool> PainelDetector::detect(const cv::Mat& image)
{
    std::vector<bool> states(false, circuits_number);
    std::vector<cv::Vec3f> circles;
    circles = circle_detector.detect(image);

    if(circles.size() > circuits_number*2){
        std::sort(circles.rend(), circles.rbegin(), compByRadius);        

        circles.erase(circles.begin() + circuits_number*2, circles.end());
    }

    std::sort(circles.begin(), circles.end(), compByWidth);
    std::vector<std::pair<cv::Vec3f, cv::Vec3f> > circle_pairs;
    
    std::vector<cv::Vec3f>::iterator it;
    if(circles.size() == circuits_number*2){
        for(it = circles.begin() ; it != circles.end() ; it += 2){
            if((*it)[1] < (*(it+1)[1])
                circle_pairs.push_back(std::make_pair(*it, *(it+1)));
            else
               circle_pairs.push_back(std::make_pair(*(it+1), *it)); 
        }
    }
    else {
        //rotina de recuperacao caso não sejam detectados todos os circulos, pode ser usado um metodo baseado inteiramente em cor
    }

    cv::Mat image_hsv;
    cv::cvtColor(image, image_hsv, cv::COLOR_BGR2HSV);

    //circulos separados por circuito
    if(upper_state == 'on') {
        
    }
    else if(upper_state == 'off') {

    }
    else {

    }
}