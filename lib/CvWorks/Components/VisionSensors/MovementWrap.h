#ifndef MOVEMENTWRAP_H
#define MOVEMENTWRAP_H
#include "VisionSensors.h"

namespace VisionSense {

template<class TObj>
class MovementWrap : public VisionCore::Tracker<cv::Mat,TObj>
{
public:
    MovementWrap(VisionCore::Tracker<cv::Mat,TObj>* baseSensor);

    ~MovementWrap();

    void update(const VisionCore::Frame<cv::Mat>& frame);

    const TObj& getLastTrack();

    bool lostTrack() const;

    /// Get the region in which movement is analised.
    cv::Rect getRoi() const;

    /// Set the region in which movement is analised.
    void setRoi(const cv::Rect &value);

    /// Indicates if a region of interest has been defined. Movement will be analized only inside this region.
    bool roiSet;

    /// Any pixel with difference bigger than this threshold is concidered as movement.
    int pixelDiffThreshold;

    /// A frame is considered with movement is the ratio of pixels that moved is bigger than this threshold.
    double moveRatioThreshold;

    /// After movement is detected, lostTrack() will return true until this much frames are detected with no movement.
    int noMovementThreshold;

    bool hasMovement() const;

private:
    /// Base sensor.
    VisionCore::Tracker<cv::Mat,TObj>* baseSensor;

    /// Previous image. Used to compute movement between frames.
    cv::Mat prevImg;

    /// Region in which movement is analised.
    cv::Rect roi;

    bool movement;
    TObj lastTrack;

    /// Number of frames without movement.
    int noMovementCount;
};

template<class TObj>
MovementWrap<TObj>::MovementWrap(VisionCore::Tracker<cv::Mat,TObj>* baseSensor)
    :baseSensor(baseSensor)
    ,roiSet(false)
    ,movement(false)
    ,pixelDiffThreshold(255)
    ,moveRatioThreshold(0.5)
    ,noMovementThreshold(1)
{

}

template<class TObj>
MovementWrap<TObj>::~MovementWrap()
{
    if(baseSensor)
        delete baseSensor;
}

template<class TObj>
void MovementWrap<TObj>::setRoi(const cv::Rect &value)
{
    roi = value;
    //se tamanho da roi mudou, resize prevImg
    if(prevImg.empty()==false){
    if(prevImg.rows!=roi.height || prevImg.cols!=roi.width){
        cv::Mat temp(prevImg);
        cv::Size s(roi.width,roi.height);
        cv::resize(temp,prevImg,s);
    }
    }
    roiSet=true;
}

template<class TObj>
bool MovementWrap<TObj>::hasMovement() const
{
    return (noMovementCount < noMovementThreshold);
}

template<class TObj>
cv::Rect MovementWrap<TObj>::getRoi() const
{
return roi;
}

template<class TObj>
bool MovementWrap<TObj>::lostTrack() const
{
    if(baseSensor){
        // if movement was detected or baseSensor is not tracking, return true
        return hasMovement() || baseSensor->lostTrack();
    }
    else{
        return false;
    }
}

template<class TObj>
const TObj &MovementWrap<TObj>::getLastTrack()
{
   //Lock in the lastTrack if movement detected.
   if(this->lostTrack())
        return lastTrack;
   else
       return baseSensor->getLastTrack();
}

template<class TObj>
void MovementWrap<TObj>::update(const VisionCore::Frame<cv::Mat> &frame)
{
    if(baseSensor){
        baseSensor->update(frame);

        //pega imagem ou sub-imagem onde vai ser analisado movimento
        const cv::Mat& img = frame.getImg();
        cv::Mat roiImg;
        if(roiSet){
            const cv::Rect imgRect(0,0,img.cols-1,img.rows-1);
            const cv::Rect r(roi&imgRect); //intersection
            roiImg=img(r);
        }
        else{
            roiImg=img;
        }

        //If base sensor is tracking, compute movement
        if(baseSensor->lostTrack()==false && prevImg.empty()==false){
            //se tamanho da roi mudou, resize prevImg
            if(prevImg.rows!=roiImg.rows || prevImg.cols!=roiImg.cols){
                cv::Mat temp(prevImg);
                cv::Size s(roiImg.cols,roiImg.rows);
                cv::resize(temp,prevImg,s);
            }
            //Detect movement
            cv::Mat diff,moveImg;
            cv::absdiff(prevImg, roiImg, diff);
            cv::threshold(diff, moveImg, pixelDiffThreshold, 255, CV_THRESH_BINARY);
            //cv::imshow("diff",diff);
            //cv::waitKey(1);

            //Average pixel movement
            const cv::Scalar moveRatio = cv::mean(moveImg)/255;
            const double meanMovement = (moveRatio[0]+moveRatio[1]+moveRatio[2])/3;
            //const double moveRatio = ((double)countMovement/(roi.height*roi.width));

            //Decide if there is movement
            if(meanMovement>moveRatioThreshold){
                movement = true;
                noMovementCount = 0; //number of frames without movement
            }
            else{
                movement=false;
                noMovementCount++;
            }
        }
        prevImg=roiImg;

        //If is ok (no movement), store base sensor result
        if(!(this->lostTrack()))
            lastTrack=baseSensor->getLastTrack();
    }
}

} //namespace
#endif // MOVEMENTWRAP_H
