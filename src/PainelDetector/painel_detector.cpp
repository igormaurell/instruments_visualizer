#include "PainelDetector/painel_detector.h"

#include <iostream>

PainelDetector::PainelDetector():
circuits_number(3),
min_radius(10.0),
max_radius(80.0),
upper_state("on"),
closing_kernel_size(11)
{
    on_color_hsv = std::vector<int>{89, 187, 56};
    on_color_thresh = std::vector<int>{15, 67, 56};
    off_color_hsv = std::vector<int>{23, 212, 106};
    off_color_thresh = std::vector<int>{23, 42, 22};
    led_on_v = 237;
    led_on_thresh = 17;
}

bool compPairByWidth(const std::pair<cv::Vec3f, uint8_t>& pair1, const std::pair<cv::Vec3f, uint8_t>& pair2)
{
    cv::Vec3f vec1, vec2;
    vec1 = pair1.first;
    vec2 = pair2.first;
    return vec1[0] < vec2[0] || (vec1[0] == vec2[0] && vec1[1] < vec2[1]);
}

bool compByWidth(const cv::Vec3f& vec1, const cv::Vec3f& vec2)
{
    return vec1[0] < vec2[0] || (vec1[0] == vec2[0] && vec1[1] < vec2[1]);
}

bool compByHeight(const cv::Vec3f& vec1, const cv::Vec3f& vec2)
{
    return vec1[1] < vec2[1] || (vec1[1] == vec2[1] && vec1[0] < vec2[0]);
}

bool compByRadius(const cv::Vec3f& vec1, const cv::Vec3f& vec2)
{
    return vec1[2] < vec2[2] || (vec1[2] == vec2[2] && vec1[0] < vec2[0]);
}

int meanCircleValue(const cv::Mat& image_hsv, const cv::Vec3f& circle)
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

void contours2Circles(std::vector<std::vector<cv::Point> >& contours, std::vector<cv::Vec3f>& circles)
{
    std::vector<std::vector<cv::Point> >::iterator it;

    for(it = contours.begin() ; it != contours.end() ; it++) {
        cv::Point2f center;
        float radius;
        cv::minEnclosingCircle(*it, center, radius);

        circles.push_back(cv::Vec3f(center.x, center.y, radius));
    }
}

void PainelDetector::calculateCircles(const cv::Mat& image, std::vector<cv::Vec3f>& circles, std::string type)
{
    cv::Scalar min_hsv, max_hsv;

    if(type == "on") {
        min_hsv = cv::Scalar(on_color_hsv[0] - on_color_thresh[0], on_color_hsv[1] - on_color_thresh[1], on_color_hsv[2] - on_color_thresh[2]); 
        max_hsv = cv::Scalar(on_color_hsv[0] + on_color_thresh[0], on_color_hsv[1] + on_color_thresh[1], on_color_hsv[2] + on_color_thresh[2]);
    }
    else if(type == "off") {
        min_hsv = cv::Scalar(off_color_hsv[0] - off_color_thresh[0], off_color_hsv[1] - off_color_thresh[1], off_color_hsv[2] - off_color_thresh[2]); 
        max_hsv = cv::Scalar(off_color_hsv[0] + off_color_thresh[0], off_color_hsv[1] + off_color_thresh[1], off_color_hsv[2] + off_color_thresh[2]);
    }
    else {
        min_hsv = cv::Scalar(0, 0, led_on_v - led_on_thresh); 
        max_hsv = cv::Scalar(180, 255, led_on_v + led_on_thresh);
    }

    cv::Mat mask_hsv, result_hsv, result_gray, th1, kernel;
    cv::inRange(image, min_hsv, max_hsv, mask_hsv);
    cv::bitwise_and(image, image, result_hsv, mask_hsv);

    cv::cvtColor(result_hsv, result_gray, CV_HSV2BGR);
    cv::cvtColor(result_gray, result_gray, CV_BGR2GRAY);

    cv::threshold(result_gray, th1, 1, 255, CV_THRESH_BINARY);

    kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(closing_kernel_size, closing_kernel_size));
    cv::morphologyEx(th1, th1, cv::MORPH_CLOSE, kernel);

    std::vector<std::vector<cv::Point> > contours;

    cv::findContours(th1, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
    
    contours2Circles(contours, circles);

    std::vector<cv::Vec3f>::iterator it = circles.begin();

    if(max_radius==0) return;

    while(it != circles.end()) {
        std::cout<<(*it)[2]<<std::endl;
        if((*it)[2] < min_radius || (*it)[2] > max_radius) {
            it = circles.erase(it);
        }
        else {
            it++;
        }
    }
}

std::vector<std::pair<cv::Vec3f, uint8_t> > PainelDetector::detect(const cv::Mat& image)
{
    cv::Mat image_hsv;

    cv::cvtColor(image, image_hsv, CV_BGR2HSV);

    std::vector<cv::Vec3f> circles;

    calculateCircles(image_hsv, circles, "default");

    //std::cout<<circles.size()<<std::endl;
    if(circles.size() > circuits_number) {
        std::cout<<"DETECCAO DE MUITOS CIRCULOS"<<std::endl;
        std::sort(circles.rbegin(), circles.rend(), compByRadius);        

        circles.erase(circles.begin() + circuits_number, circles.end());
    }
   // std::cout<<circles.size()<<std::endl;


    std::vector<cv::Vec3f> line1;
    std::vector<cv::Vec3f> line2;
    std::vector<cv::Vec3f>::iterator it;
    for(it = circles.begin() ; it != circles.end() ; it++) {
        if(line1.size() == 0) {
            line1.push_back(*it);
        }
        else if(line1[0][1] + 2*line1[0][2] > (*it)[1] && line1[0][1] - 2*line1[0][2] < (*it)[1]) {
            line1.push_back(*it);
        }
        else {
            line2.push_back(*it);
        }
    }

    //std::cout<<"L1: "<<line1.size()<<std::endl;
    //std::cout<<"L2: "<<line2.size()<<std::endl;

    std::vector<std::pair<cv::Vec3f, uint8_t> > states;

    if(line1.size() == 0 || line2.size() == 0){
        std::cout<<"CASO ALINHADO"<<std::endl;

        std::vector<cv::Vec3f> circles_on, circles_off;

        calculateCircles(image_hsv, circles_on, "on");

        if(circles_on.size() > circuits_number) {
            std::cout<<"DETECCAO DE MUITOS CIRCULOS: on"<<std::endl;
            std::sort(circles_on.rbegin(), circles_on.rend(), compByRadius);        
            circles_on.erase(circles_on.begin() + circuits_number, circles_on.end());
        }

        if(circles_off.size() > circuits_number) {
            std::cout<<"DETECCAO DE MUITOS CIRCULOS: off"<<std::endl;
            std::sort(circles_off.rbegin(), circles_off.rend(), compByRadius);   
            circles_off.erase(circles_off.begin() + circuits_number, circles_off.end());
        }
        


        std::vector<cv::Vec3f> line;

        if(line1.size() > 0) line = line1;
        else line = line2;
        int on_rate = 0, off_rate = 0;
        std::vector<cv::Vec3f>::iterator c_it1;
        std::vector<cv::Vec3f>::iterator c_it2;
        for(c_it1 = line.begin() ; c_it1 != line.end() ; c_it1++) {
            for(c_it2 = circles_on.begin() ; c_it2 != circles_on.end() ; c_it2++){
                if(((*c_it1)[0] + 2*(*c_it1)[2] > (*c_it2)[0] && (*c_it1)[0] - 2*(*c_it1)[2] < (*c_it2)[0]) 
                && ((*c_it1)[1] + 2*(*c_it1)[2] < (*c_it2)[1] || (*c_it1)[1] - 2*(*c_it1)[2] > (*c_it2)[1])) {
                    off_rate++;
                }
            }
        }

        for(c_it1 = line.begin() ; c_it1 != line.end() ; c_it1++) {
            for(c_it2 = circles_off.begin() ; c_it2 != circles_off.end() ; c_it2++){
                if(((*c_it1)[0] + 2*(*c_it1)[2] > (*c_it2)[0] && (*c_it1)[0] - 2*(*c_it1)[2] < (*c_it2)[0]) 
                && ((*c_it1)[1] + 2*(*c_it1)[2] < (*c_it2)[1] || (*c_it1)[1] - 2*(*c_it1)[2] > (*c_it2)[1])) {
                    on_rate++;
                }
            }
        }

        if(on_rate>=off_rate) {
            for(it = line.begin() ; it != line.end() ; it++) {
                states.push_back(std::make_pair(*it, true));
            }
        }
        else {
            for(it = line.begin() ; it != line.end() ; it++) {
                states.push_back(std::make_pair(*it, false));
            }
        }
    }
    else {
        if(line1[0][1] < line2[0][1]){
            for(it = line1.begin() ; it != line1.end() ; it++) {
                states.push_back(std::make_pair(*it, true));
            }
            for(it = line2.begin() ; it != line2.end() ; it++) {
                states.push_back(std::make_pair(*it, false));
            }
        }
        else {
            for(it = line1.begin() ; it != line1.end() ; it++) {
                states.push_back(std::make_pair(*it, false));
            }
            for(it = line2.begin() ; it != line2.end() ; it++) {
                states.push_back(std::make_pair(*it, true));
            }
        }
    }

    std::sort(states.begin(), states.end(), compPairByWidth);
    return states;
}

































/*std::pair<std::vector<std::pair<cv::Vec3f, cv::Vec3f> >, std::vector<uint8_t> > PainelDetector::detect(const cv::Mat& image)
{
    std::vector<uint8_t> states(false, circuits_number);
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
        std::vector<uint8_t>::iterator kt;
        for(kt = states.begin() ; kt != states.end() ; kt++){
            *kt != *kt;
        }
        return std::make_pair(circle_pairs, states);
    }
    else {
        //implementar para quando não temos certeza de qual cor ta emcima
    }
}*/