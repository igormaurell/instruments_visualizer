#ifndef LUMINANCEWRAP_H
#define LUMINANCEWRAP_H

#include "VisionSensors.h"

namespace VisionSense {

template<class TObj>
class LuminanceWrap : public VisionCore::Tracker<cv::Mat,TObj>
{
public:
    LuminanceWrap(VisionCore::Tracker<cv::Mat,TObj>* baseSensor);

    ~LuminanceWrap();

    void update(const VisionCore::Frame<cv::Mat>& frame);

    const TObj& getLastTrack();

    bool lostTrack() const;

    /// Get the region in which luminance is analised.
    cv::Rect getRoi() const;

    /// Set the region in which luminance is analised.
    void setRoi(const cv::Rect &value);

    /// A frame is considered with good luminance if average pixels intensity is bigger than this threshold.
    double luminanceThreshold;

    /// After a bad frame is detected, lostTrack() will return true until this much frames are detected with no good luminance.
    int framesLumThreshold;

    /// Indicates if a region of interest has been defined. Luminance will be analized only inside this region.
    bool roiSet;

private:
    /// Base sensor.
    VisionCore::Tracker<cv::Mat,TObj>* baseSensor;

    /// Region in which luminance is analised.
    cv::Rect roi;

    bool luminance;

    /// Last good measurement.
    TObj lastTrack;

    /// Number of frames with good luminance
    int luminanceCount;
};

template<class TObj>
LuminanceWrap<TObj>::LuminanceWrap(VisionCore::Tracker<cv::Mat,TObj>* baseSensor)
    :baseSensor(baseSensor)
    ,roiSet(false)
    ,luminance(false)
    ,luminanceThreshold(50)
    ,framesLumThreshold(60)
{

}

template<class TObj>
LuminanceWrap<TObj>::~LuminanceWrap()
{
    if(baseSensor)
        delete baseSensor;
}

template<class TObj>
void LuminanceWrap<TObj>::setRoi(const cv::Rect &value)
{
    roi = value;
    roiSet=true;
}

template<class TObj>
cv::Rect LuminanceWrap<TObj>::getRoi() const
{
return roi;
}

template<class TObj>
bool LuminanceWrap<TObj>::lostTrack() const
{
    // returns true if luminance is bad or baseSensor lost track
    if(baseSensor){
        return (luminanceCount < framesLumThreshold) || baseSensor->lostTrack();
    }
    else{
        return false;
    }
}

template<class TObj>
const TObj &LuminanceWrap<TObj>::getLastTrack()
{
   //Lock in the lastTrack if no luminance detected.
   if(this->lostTrack())
        return lastTrack;
   else
       return baseSensor->getLastTrack();
}

template<class TObj>
void LuminanceWrap<TObj>::update(const VisionCore::Frame<cv::Mat> &frame)
{
    if(baseSensor){
        baseSensor->update(frame);

        //If base sensor is tracking, compute luminance
        if(baseSensor->lostTrack()==false ){

            // Convert image to grayscale and get region of interest
            const cv::Mat& img = frame.getImg();
            cv::Mat gray, roiImg;
            if(img.channels()>1)
                cvtColor(img, gray, CV_BGR2GRAY);
            else
                gray=img;
            if(roiSet){
                const cv::Rect imgRect(0,0,img.cols-1,img.rows-1);
                const cv::Rect r(roi&imgRect); //intersection
                roiImg = gray(r);
            }
            else{
                roiImg=gray;
            }
            //Process luminance
            const cv::Scalar lum = cv::mean(roiImg);
            if(lum[0]>luminanceThreshold){
                luminance = true;
                luminanceCount++;
            }
            else{
                luminance=false;
                luminanceCount=0;
            }
        }
        // Keeps the last good track
        if(!(this->lostTrack()))
            lastTrack=baseSensor->getLastTrack();
    }
}

} //namespace

#endif // LUMINANCEWRAP_H
