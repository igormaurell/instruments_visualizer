#include "ValveDetector/valve_detector.h"

ValveDetector::ValveDetector():
extent_thresh(0.7),
use_gaussian_filter(true),
gaussian_kernel_size(5),
closing_kernel_size(7)
{
    /*mobile_hs = std::vector<int>{120, 190};
    mobile_hs_thresh = std::vector<int>{90, 30};*/
}

std::pair<std::vector<cv::Point>, bool> ValveDetector::detect(const cv::Mat& image)
{
    cv::Mat image_gray, th2, kernel;

    if(use_gaussian_filter)
        cv::GaussianBlur(image_gray, image_gray, cv::Size(gaussian_kernel_size, gaussian_kernel_size), 0);

    cv::cvtColor(image, image_gray, CV_BGR2GRAY);

    cv::threshold(image_gray, th2, 0, 255, CV_THRESH_OTSU);

    cv::bitwise_not(th2, th2);    

    kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(closing_kernel_size, closing_kernel_size));
    cv::morphologyEx(th2, th2, cv::MORPH_CLOSE, kernel);

    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Point> contour;

    cv::findContours(th2, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

    std::vector<std::vector<cv::Point> >::iterator it;

    for(it = contours.begin() ; it != contours.end() ; it++) {
        if(contour.size() == 0 || cv::contourArea(*it) > cv::contourArea(contour)) {
            contour = *it;
        }
    }


    cv::Rect rec;
    double area = cv::contourArea(contour);
    rec = cv::boundingRect(contour);
    double rec_area = rec.width*rec.height;
    double extent = float(area)/rec_area;

    return std::make_pair(contour, extent>=extent_thresh);
}

/*std::pair<cv::RotatedRect, cv::RotatedRect> ValveDetector::detect(const cv::Mat& image)
{
    cv::Mat image_gray, image_hsv, th2;

    cv::cvtColor(image, image_gray, CV_BGR2GRAY);
    cv::cvtColor(image, image_hsv, CV_BGR2HSV);

    cv::threshold(image_gray, th2, 0, 255, CV_THRESH_OTSU);

    cv::imshow("dasdas", th2);
    cv::waitKey(0);

    cv::Scalar min_hsv = cv::Scalar(mobile_hs[0] - mobile_hs_thresh[0], mobile_hs[1] - mobile_hs_thresh[1], 0); 
    cv::Scalar max_hsv = cv::Scalar(mobile_hs[0] + mobile_hs_thresh[0], mobile_hs[1] + mobile_hs_thresh[1], 255);

    cv::Mat mask_hsv, result_hsv, result_gray, th1;
    cv::inRange(image_hsv, min_hsv, max_hsv, mask_hsv);
    cv::bitwise_and(image_hsv, image_hsv, result_hsv, mask_hsv);
 
    cv::cvtColor(result_hsv, result_gray, CV_HSV2BGR);
    cv::cvtColor(result_gray, result_gray, CV_BGR2GRAY);

    cv::threshold(result_gray, th1, 1, 255, CV_THRESH_BINARY);

    std::vector<std::vector<cv::Point> > contours1;
    std::vector<cv::Point> contour1;

    cv::findContours(th1, contours1, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);

    if(contours1.size() == 0){
        cv::RotatedRect rect(cv::Point2f(-1, -1), cv::Size2f(0, 0), 0);
        return std::make_pair(rect, rect);
    }

    std::vector<std::vector<cv::Point> >::iterator it;

    for(it = contours1.begin() ; it != contours1.end() ; it++) {
        if(contour1.size() == 0 || cv::contourArea(*it) > cv::contourArea(contour1)) {
            contour1 = *it;
        }
    }

    contours1 = std::vector<std::vector<cv::Point> >{contour1};

    cv::drawContours(th1, contours1, 0, 255, -1);

    cv::RotatedRect rect_mob;
    rect_mob = cv::minAreaRect(contour1);

    
    cv::Mat th1_inv, kernel;
    if(use_gaussian_filter)
        cv::GaussianBlur(image_gray, image_gray, cv::Size(gaussian_kernel_size, gaussian_kernel_size), 0);

    cv::threshold(image_gray, th2, 0, 255, CV_THRESH_OTSU);

    cv::imshow("dasdas", th2);
    cv::waitKey(0);

    cv::bitwise_not(th2, th2);

    cv::bitwise_not(th1, th1_inv);
    cv::bitwise_and(th2, th1_inv, th2);

    kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(opening_kernel_size, opening_kernel_size));
    cv::morphologyEx(th2, th2, cv::MORPH_OPEN, kernel);

    std::vector<std::vector<cv::Point> > contours2;
    std::vector<cv::Point> contour2;
    cv::findContours(th2, contours2, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

    if(contours2.size() == 0){
        cv::RotatedRect rect(cv::Point2f(-1, -1), cv::Size2f(0, 0), 0);
        return std::make_pair(rect, rect);
    }

    std::vector<std::vector<cv::Point> >::iterator jt;
    for(jt = contours2.begin() ; jt != contours2.end() ; jt++) {
        if(contour2.size() == 0 || cv::contourArea(*jt) > cv::contourArea(contour2)) {
            contour2 = *jt;
        }
    }

    cv::RotatedRect rect_bod;
    std::vector<cv::Point> box_bod;
    rect_bod = cv::minAreaRect(contour2);

    return std::make_pair(rect_bod, rect_mob);
}*/