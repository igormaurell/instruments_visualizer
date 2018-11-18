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

#include "TemperatureTrackerTCam.h"

namespace VisionSense {


TemperatureTrackerTCam::TemperatureTrackerTCam()
    :configured(false)
    ,minTemp(20)
    ,maxTemp(50)
    ,temp(20)
{

}

TemperatureTrackerTCam::~TemperatureTrackerTCam()
{

}

void TemperatureTrackerTCam::update(const VisionCore::Frame<cv::Mat> &frame)
{
    if(configured){
        const cv::Mat& img = frame.getImg();
        cv::Mat gray;
        /// Convertendo a imagem para tons de cinza
        if(img.channels()>1)
            cvtColor(img, gray, CV_BGR2GRAY);
        else
            gray=img;
        temp=minTemp+(maxTemp-minTemp)*gray.at<uchar>(point)/255.0;
    }
}

const double &TemperatureTrackerTCam::getLastTrack()
{
    return temp;
}

bool TemperatureTrackerTCam::lostTrack() const
{
    return !configured;
}



cv::Point TemperatureTrackerTCam::getPoint() const
{
    return point;
}

void TemperatureTrackerTCam::setPoint(const cv::Point &value)
{
    point = value;
    configured=true;
}
}
