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

#include "PresenceTracker.h"

namespace VisionSense {

PresenceTracker::PresenceTracker(void)
    :presence(false)
    ,negConfigOk(false)
    ,posConfigOk(false)
    ,outOfFrame(false)
{
}


PresenceTracker::~PresenceTracker(void)
{
}

void PresenceTracker::update(const VisionCore::Frame<cv::Mat>& frame){
    if(negConfigOk && posConfigOk){
        // Cria uma sub-imagem contendo a "region of interest"
        cv::Rect roi(topLeft,bottonRight);
        const cv::Mat& originalImg = frame.getImg();
        const cv::Rect imgRect(0,0,originalImg.cols-1,originalImg.rows-1);
        const cv::Rect r(roi&imgRect); //intersection
        outOfFrame = (r!=roi);
        const cv::Mat img = originalImg(r);

        // Calcula intensidade m�dia do pixel no espa�o de cor Lab
        cv::Mat imgLab;
        cv::cvtColor(img,imgLab,CV_BGR2Lab);
        const cv::Scalar meanColor = cv::mean(imgLab);

        // Calcula a diferen�a para as refer�ncias positivas e negativas
        const double posDist = cv::norm(meanColor,positiveColorLab,cv::NORM_L2);
        const double negDist = cv::norm(meanColor,negativeColorLab,cv::NORM_L2);
        presence = posDist > negDist;
    }

}

const bool& PresenceTracker::getLastTrack(){
	return presence;
}

/// Set the negative presence (lack of object) based on a reference image.
void PresenceTracker::setNegativeReference(const cv::Mat& img){
	// Calcula intensidade m�dia do pixel no espa�o de cor Lab
	cv::Mat imgLab;
	cv::cvtColor(img,imgLab,CV_BGR2Lab);
	negativeColorLab = cv::mean(imgLab);
    negConfigOk=true;
}

/// Set the positive presence (object is present) based on a reference image.
void PresenceTracker::setPositiveReference(const cv::Mat& img){
	// Calcula intensidade m�dia do pixel no espa�o de cor Lab
	cv::Mat imgLab;
	cv::cvtColor(img,imgLab,CV_BGR2Lab);
    positiveColorLab = cv::mean(imgLab);
    posConfigOk=true;
}

void PresenceTracker::setNegativeReferenceC(const cv::Scalar &color)
{
    negativeColorLab=color;
    negConfigOk=true;
}

void PresenceTracker::setPositiveReferenceC(const cv::Scalar &color)
{
    positiveColorLab=color;
    posConfigOk=true;
}

}


bool VisionSense::PresenceTracker::lostTrack() const
{
    return !negConfigOk || !posConfigOk || outOfFrame;
}
