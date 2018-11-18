/*******************************************************************************\
Copyright (c) 2016, FURG - Universidade Federal do Rio Grande
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Universidade Federal do Rio Grande nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL UNIVERSIDADE FEDERAL DO RIO GRANDE BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

/*******************************************************************************/

#include "TemperatureTrackerThermo.h"

namespace VisionSense {

TemperatureTrackerThermo::TemperatureTrackerThermo()
    :roiOk(false)
    ,minColorOk(false)
    ,maxColorOk(false)
    ,minTemp(0)
    ,maxTemp(100)
    ,temperature(0)
    ,computeReferenceEveryFrame(false)
{

}

TemperatureTrackerThermo::~TemperatureTrackerThermo()
{

}

void TemperatureTrackerThermo::update(const VisionCore::Frame<cv::Mat> &frame)
{
    if(configured()){
        // Get img in Lab color space
        const cv::Mat& img = frame.getImg();
        cv::Mat imgLab;
        cv::cvtColor(img,imgLab,CV_BGR2Lab);        

        // Cria uma sub-imagem contendo a "region of interest"
        const cv::Rect imgRect(0,0,imgLab.cols-1,imgLab.rows-1);
        cv::Rect r(roi&imgRect); //intersection
        outOfFrame = (r!=roi);
        const cv::Mat imgRoi = imgLab(r);

        //Extract mean color
        cv::Scalar meanColor = extractStickerColor(imgRoi);

        if(this->computeReferenceEveryFrame){
            // Compute min reference color
            r = (imgRect&minRect);
            const bool of1 = (r!=minRect);  //out of frame?
            if(!of1){
                cv::Scalar c1 = extractStickerColor(imgLab(minRect));
                setMinColor(c1);
            }

            //Compute max color
            r = (imgRect&maxRect);
            const bool of2 = (r!=maxRect);  //out of frame?
            if(!of2){
                cv::Scalar c2 = extractStickerColor(imgLab(maxRect));
                setMaxColor(c2);
            }
            outOfFrame = outOfFrame || of1 || of2;
        }

        temperature = inferTemperature(meanColor);
    }

}

const double &TemperatureTrackerThermo::getLastTrack()
{
    return temperature;

}

bool TemperatureTrackerThermo::lostTrack() const
{
    return !(configured()) || outOfFrame;
}

void TemperatureTrackerThermo::setMinimumReference(const cv::Mat &img,const cv::Rect &r)
{
    cv::Mat imgLab;
    cv::cvtColor(img,imgLab,CV_BGR2Lab);
    cv::Scalar color = extractStickerColor(imgLab);
    setMinColor(color);
    minRect=r;
}

void TemperatureTrackerThermo::setMaximumReference(const cv::Mat &img,const cv::Rect &r)
{
    cv::Mat imgLab;
    cv::cvtColor(img,imgLab,CV_BGR2Lab);
    cv::Scalar color = extractStickerColor(imgLab);
    setMaxColor(color);
    maxRect=r;
}

void TemperatureTrackerThermo::setRegion(const cv::Rect &roi)
{
    this->roi=roi;
    roiOk=true;
}
cv::Scalar TemperatureTrackerThermo::getMinColor() const
{
    return minColor;
}

void TemperatureTrackerThermo::setMinColor(const cv::Scalar &value)
{
    minColor = value;
    minColorOk = true;
}
cv::Scalar TemperatureTrackerThermo::getMaxColor() const
{
    return maxColor;
}

void TemperatureTrackerThermo::setMaxColor(const cv::Scalar &value)
{
    maxColor = value;
    maxColorOk = true;
}
cv::Rect TemperatureTrackerThermo::getRoi() const
{
    return roi;
}




cv::Scalar TemperatureTrackerThermo::extractStickerColor(const cv::Mat &img)
{
    //Compute the mean for pixels inside a circle
    const cv::Point center(img.cols/2,img.rows/2);
    const int radius = std::min(img.cols/2,img.rows/2);
    //radius*=0.8;
    const int radius2 = radius*radius;
    cv::Scalar_<long> color;
    long pixelCount=0;
    for (int i = 0; i < img.cols; i++){
        for (int j = 0; j < img.rows; j++){
            if (pow(center.x - i, 2) + pow(center.y - j, 2) <= radius2)
            {
                cv::Scalar intensity = img.at<cv::Vec3b>(j, i);
                color+=intensity;
                pixelCount++;
            }
        }
    }
    color/=pixelCount;
    return color;
}

double TemperatureTrackerThermo::inferTemperature(const cv::Scalar &color) const
{
    //translate axis to minColor
    const cv::Scalar cT = color-minColor;
    const cv::Scalar maxT  = maxColor-minColor;

    //project cT in the maxT vector
    const double proj = cT[0]*maxT[0]+cT[1]*maxT[1]+cT[2]*maxT[2];
    const double maxTLen = maxT[0]*maxT[0]+maxT[1]*maxT[1]+maxT[2]*maxT[2];

    const double normProj= proj/maxTLen;
    const double temperature = normProj*(maxTemp-minTemp) + minTemp;

    return temperature;
}

bool TemperatureTrackerThermo::configured() const
{
    return minColorOk && maxColorOk && roiOk;
}
}
