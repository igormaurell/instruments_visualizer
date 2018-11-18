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

#include "LevelTrackerAuto.h"

namespace VisionSense {

LevelTrackerAuto::LevelTrackerAuto()
    :level(0.0)
    ,regOk(false)
{

}

LevelTrackerAuto::~LevelTrackerAuto()
{

}

void LevelTrackerAuto::update(const VisionCore::Frame<cv::Mat> &frame)
{
    // Cria uma sub-imagem contendo a "region of interest"
    if(regOk){
        cv::Rect roi(topLeft,bottonRight);
        const cv::Mat& originalImg = frame.getImg();
        cv::Mat img = originalImg(roi);

        // Converte para espaço de cor Lab
        cv::Mat imgLab;
        cv::cvtColor(img,imgLab,CV_BGR2Lab);

        // Para cada linha, calcula "divergência" (distância entre cor da região superior e inferior)
        std::vector<double> diverg(imgLab.rows);
        double maxDiv=std::numeric_limits<double>::min();
        int maxDivIndex=0;
        diverg[0]=0.0;
        diverg[imgLab.rows-1]=0.0;
        for(int i=1;i<imgLab.rows-1;i++){
            const cv::Rect top = cv::Rect(0,0,imgLab.cols,i);
            const cv::Mat& imgTop= imgLab(top);
            const cv::Scalar meanTop = cv::mean(imgTop);
            const cv::Rect botton = cv::Rect(0,i,imgLab.cols,imgLab.rows-i);
            const cv::Scalar meanBotton = cv::mean(imgLab(botton));
            diverg[i] = cv::norm(meanTop,meanBotton,cv::NORM_L2);
            if(diverg[i]>maxDiv){
                maxDiv=diverg[i];
                maxDivIndex=i;
            }
        }
        level=(imgLab.rows-maxDivIndex)/(double)imgLab.rows;
    }

}

const double &LevelTrackerAuto::getLastTrack()
{
    return level;
}

bool LevelTrackerAuto::lostTrack() const
{
    return !regOk;

}

void LevelTrackerAuto::setRegion(const cv::Rect &roi)
{
    topLeft=roi.tl();
    bottonRight=roi.br();
    if(topLeft.x<bottonRight.x && topLeft.y<bottonRight.y)
        regOk=true;
}

}
