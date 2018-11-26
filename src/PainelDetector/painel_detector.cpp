#include "PainelDetector/painel_detector.h"

#include <iostream>

PainelDetector::PainelDetector():
circuits_number(3),
upper_state("on")
//on_color_hs;
//on_color_thresh;
//off_color_hs;
//off_color_thresh;
//led_on_v,
//led_on_thresh
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
    if(top_left.x>=0 && top_left.y>=0 && botton_right.x<=image_hsv.cols && botton_right.y<=image_hsv.rows){
        cv::Rect ROI(top_left, botton_right);
        cv::Mat crop = image_hsv(ROI);

        for (int i = 0 ; i < crop.cols ; i++){
            for (int j = 0 ; j < crop.rows ; j++){
                mean_value += crop.at<cv::Vec3b>(i, j)[2];
            }
        }
        mean_value /= (crop.cols*crop.rows);
    }
    return mean_value;
}


std::pair<std::vector<std::pair<cv::Vec3f, cv::Vec3f> >, std::vector<bool> > PainelDetector::detect(const cv::Mat& image)
{
    std::vector<bool> states(false, circuits_number);
    std::vector<cv::Vec3f> circles;
    circles = circle_detector.detect(image);

    std::cout<<circles.size()<<std::endl;

    std::vector<cv::Vec3f>::iterator ct;     
    for(ct = circles.begin() ; ct != circles.end() ; ct++){
        cv::Vec3f c1;
        c1 = *ct;
        cv::circle(image, cv::Point(c1[0], c1[1]), c1[2], cv::Scalar(255, 0, 0), 3);
        //cv::putText(image, std::to_string(i), cv::Point(c1[0], c1[1]), cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(255, 0, 0));
    }
    cv::imshow("PAIsNEL", image);
    cv::waitKey(0);

    if(circles.size() > circuits_number*2){
        std::sort(circles.rend(), circles.rbegin(), compByRadius);        

        circles.erase(circles.begin() + circuits_number*2, circles.end());
    }

    

    std::sort(circles.begin(), circles.end(), compByWidth);
    std::vector<std::pair<cv::Vec3f, cv::Vec3f> > circle_pairs;
    
    std::vector<cv::Vec3f>::iterator it;
    if(circles.size() == circuits_number*2){
        for(it = circles.begin() ; it != circles.end() ; it += 2){
            if((*it)[1] < ((*(it+1)))[1])
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

    std::vector<std::pair<cv::Vec3f, cv::Vec3f> >::iterator jt;

    //circulos separados por circuito
    int i;
    for(jt = circle_pairs.begin(), i = 0 ; jt != circle_pairs.end() ; jt++, i++) {
        int lower_value, upper_value;
        lower_value = meanCircleValue(image_hsv, jt->first);
        upper_value = meanCircleValue(image_hsv, jt->second);
        states[i] = upper_value>lower_value;
    }
    if(upper_state == "on") {
        return std::make_pair(circle_pairs, states);
    }
    else if(upper_state == "off") {
        std::vector<bool>::iterator kt;
        for(kt = states.begin() ; kt != states.end() ; kt++){
            *kt != *kt;
        }
        return std::make_pair(circle_pairs, states);
    }
    else {
        //implementar para quando não temos certeza de qual cor ta emcima
    }
}