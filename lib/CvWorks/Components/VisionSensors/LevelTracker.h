/*
Copyright (c) 2015, FURG - Universidade Federal do Rio Grande
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
*/

#pragma once

#include "VisionSensors.h"

namespace VisionSense {

/// Rastreador de nível de um líquido ou sólido dentro de um container.
/** Um container é definido na imagem por um retângulo através de um ponto esquerdo-superior e outro ponto 
 * direito inferior.
 *
 * A inferência do nível de preenchimento do container é feita através da difração da luz ao passar entre a
 * transição entre ar/líquido.
 *
 * \ingroup VisionSensors
*/
class LevelTracker :
    public VisionCore::Tracker<cv::Mat,double>
{
public:
	LevelTracker(void);
	virtual ~LevelTracker(void);

    void update(const VisionCore::Frame<cv::Mat>& frame);

	const double& getLastTrack();

    bool lostTrack() const;

    /// Sets the negative level (lack of liquid) based on a reference image.
	void setNegativeReference(const cv::Mat& img);

    /// Sets the positive level (liquid is present) based on a reference image.
	void setPositiveReference(const cv::Mat& img);

    /// Sets the negative level (lack of liquid) based on a color.
    void setNegativeReferenceC(const cv::Scalar& color);

    /// Sets the positive level (liquid is present) based on a color.
    void setPositiveReferenceC(const cv::Scalar& color);


    /// Sets the region were the level will be measured.
    void setRegion(const cv::Point& p0_,const cv::Point& p1_,const cv::Point& p2_);

    /// Extract a sub-image given a rotated rectangle.
    /**
     * The rectangle is defined by three points in counter-clockwise direction starting
     * from the top-left point.
     * \returns Boolean value indicating whether the rotated rectangle extrapolated the image region.
     */
    static bool extractRotatedRoi(const cv::Mat& src,cv::Mat& dst,const cv::Point& p0_,const cv::Point& p1_,const cv::Point& p2_);

	cv::Scalar positiveColorLab;
	cv::Scalar negativeColorLab;

    cv::Mat covPositiveColor;
    cv::Mat covNegativeColor;

    /// Does not use L component of Lab color to compute distances. May be more robust to light variation.
    bool ignoreLcomponent;

    int method;

    cv::Point p0;
    cv::Point p1;
    cv::Point p2;


private:
    double level;
    bool posColOk;
    bool negColOk;
    bool regOk;
    bool outOfFrame;

    cv::Mat extractRoiImg(const cv::Mat &img);

    double colorDistance(const cv::Scalar c1,cv::Scalar c2);
};

} //namespace VisionSense
