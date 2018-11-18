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

#ifndef TemperatureTrackerThermo_H
#define TemperatureTrackerThermo_H
#include "VisionSensors.h"

namespace VisionSense {

class TemperatureTrackerThermo :
        public VisionCore::Tracker<cv::Mat,double>

{
public:
    TemperatureTrackerThermo(void);
    virtual ~TemperatureTrackerThermo(void);

    void update(const VisionCore::Frame<cv::Mat>& frame);

    const double& getLastTrack();

    bool lostTrack() const;

    /// Sets the minimum temperature (sticker not active) reference image.
    void setMinimumReference(const cv::Mat& img,const cv::Rect &r = cv::Rect());

    /// Sets the maximum temperature (sticker active) reference image.
    void setMaximumReference(const cv::Mat& img,const cv::Rect &r = cv::Rect());

    /// Sets the region were the temperature will be measured.
    void setRegion(const cv::Rect& roi);

    double minTemp;
    double maxTemp;
    cv::Rect minRect;
    cv::Rect maxRect;

    bool computeReferenceEveryFrame;

    cv::Scalar getMinColor() const;
    void setMinColor(const cv::Scalar &value);

    cv::Scalar getMaxColor() const;
    void setMaxColor(const cv::Scalar &value);

    cv::Rect getRoi() const;


private:
    double temperature;
    cv::Scalar minColor;
    cv::Scalar maxColor;
    cv::Rect roi;

    bool roiOk;
    bool minColorOk;
    bool maxColorOk;
    bool outOfFrame;

    /// Extract the mean color from a segmented sticker image.
    static cv::Scalar extractStickerColor(const cv::Mat& img);

    /// Infer the temperature given a color;
    double inferTemperature(const cv::Scalar& color) const;

    /// Return true if all parameters where configured
    bool configured() const;

};

}

#endif // TemperatureTrackerThermo_H
