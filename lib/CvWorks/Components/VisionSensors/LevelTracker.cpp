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

#include "LevelTracker.h"

namespace VisionSense {


LevelTracker::LevelTracker(void)
    : regOk(false)
    , negColOk(false)
    , posColOk(false)
    , outOfFrame(false)
    , ignoreLcomponent(false)
    , method(1)
    , covNegativeColor(cv::Mat::eye(3, 3, CV_64F))
    , covPositiveColor(cv::Mat::eye(3, 3, CV_64F))
{
}

LevelTracker::~LevelTracker(void)
{
}

void LevelTracker::update(const VisionCore::Frame<cv::Mat>& frame){

    if(negColOk && posColOk && regOk){
        // Cria uma sub-imagem contendo a "region of interest"
        cv::Mat img = this->extractRoiImg(frame.getImg());

        // Converte para espaço de cor Lab
        cv::Mat imgLab;
        cv::cvtColor(img,imgLab,CV_BGR2Lab);

        // Para cada linha, calcula distâncias para referências
        std::vector<double> distPos(img.rows);  //distância cumulativa para a referência em cada linha
        std::vector<double> distNeg(img.rows);
        double distPosCum=0;
        double distNegCum=0;
        for(int i=0;i<imgLab.rows;i++){
            if(method==1){ //distância euclidiana de cada linha
                cv::Mat row=imgLab.row(i);
                cv::Scalar meanColor = cv::mean(row);
                distPos[i]=distPosCum+colorDistance(meanColor,positiveColorLab);
                distNeg[i]=distNegCum+colorDistance(meanColor,negativeColorLab);
                distPosCum=distPos[i];
                distNegCum=distNeg[i];
            }
            if(method==2){
                double distPosRow=0.0;
                double distNegRow=0.0;
                for(int j=0;j<imgLab.cols;j++){
                    const cv::Vec3b p = imgLab.at<cv::Vec3b>(i,j);
                    const cv::Scalar ps(p[0],p[1],p[2]);
                    distPosRow+=colorDistance(ps,positiveColorLab);
                    distNegRow+=colorDistance(ps,negativeColorLab);
                }
                distPos[i]=distPosCum+distPosRow;
                distNeg[i]=distNegCum+distNegRow;
                distPosCum=distPos[i];
                distNegCum=distNeg[i];
            }
            if(method==3){
                double distPosRow=0.0;
                double distNegRow=0.0;
                for(int j=0;j<imgLab.cols;j++){
                    const cv::Vec3f p = imgLab.at<cv::Vec3b>(i,j);
                    const cv::Vec3f m(positiveColorLab[0],positiveColorLab[1],positiveColorLab[2]);
                    const cv::Vec3f pm = p-m;
                    auto r1 = pm*covPositiveColor;
                    auto r = r1*(pm.t());
                    distPosRow+=r(0,0);
                    distNegRow+=distPosRow;
                    //distNegRow+=cv::norm(ps,negativeColorLab,cv::NORM_L2);
                }
                distPos[i]=distPosCum+distPosRow;
                distNeg[i]=distNegCum+distNegRow;
                distPosCum=distPos[i];
                distNegCum=distNeg[i];
            }
        }

        // Decide nível. Calcula o erro de decisão para cada linha.
        double minError=std::numeric_limits<double>::max();
        int minErrorIndex=0;
        for(int i=0;i<imgLab.rows;i++){
            double error=distNeg[i]+distPos[imgLab.rows-1]-distPos[i];
            if(error<minError){
                minError=error;
                minErrorIndex=i;
            }
        }
        level=(imgLab.rows-minErrorIndex)/(double)imgLab.rows;
    }

    // True se algo não está configurado
    m_lostTrack = !negColOk || !posColOk || !regOk || outOfFrame;
}

const double& LevelTracker::getLastTrack() {
	return level;

}

bool LevelTracker::lostTrack() const
{
    return m_lostTrack;
}

/// Set the negative presence (lack of object) based on a reference image.
void LevelTracker::setNegativeReference(const cv::Mat& img){
	// Calcula intensidade média do pixel no espaço de cor Lab
    if(!img.empty()){
        cv::Mat imgLab;
        cv::cvtColor(img,imgLab,CV_BGR2Lab);
        this->setNegativeReferenceC(cv::mean(imgLab));
    }
}

/// Set the positive presence (object is present) based on a reference image.
void LevelTracker::setPositiveReference(const cv::Mat& img){
	// Calcula intensidade média do pixel no espaço de cor Lab
    if(!img.empty()){
        cv::Mat imgLab;
        cv::cvtColor(img,imgLab,CV_BGR2Lab);
        this->setPositiveReferenceC(cv::mean(imgLab));

        cv::Mat data(img.rows*img.cols,3,CV_8UC1);
        for(int i=0;i<img.rows;i++){
            for(int j=0;j<img.cols;j++){
                const cv::Vec3b &p = imgLab.at<cv::Vec3b>(i,j);
                data.at<uchar>(i*img.cols+j,0)=p[0];
                data.at<uchar>(i*img.cols+j,1)=p[1];
                data.at<uchar>(i*img.cols+j,2)=p[2];
            }
        }
        cv::Mat mean(1,3,CV_64F);
        cv::calcCovarMatrix(data,covPositiveColor,mean,CV_COVAR_NORMAL | CV_COVAR_ROWS);
    }
}

void LevelTracker::setNegativeReferenceC(const cv::Scalar &color)
{
    negativeColorLab=color;
    negColOk=true;
}

void LevelTracker::setPositiveReferenceC(const cv::Scalar &color)
{
    positiveColorLab=color;
    posColOk=true;
}

void LevelTracker::setRegion(const cv::Point &p0_, const cv::Point &p1_, const cv::Point &p2_)
{
    p0=p0_;
    p1=p1_;
    p2=p2_;
    if(p0_.x<p2_.x && p0_.y<p2_.y)
        regOk=true;
}

cv::Mat LevelTracker::extractRoiImg(const cv::Mat& img)
{
    cv::Mat roiImg;
    outOfFrame=LevelTracker::extractRotatedRoi(img,roiImg,p0,p1,p2);
    return roiImg;
}

double LevelTracker::colorDistance(const cv::Scalar c1, cv::Scalar c2)
{
    double dist=0.0;
    if(ignoreLcomponent){
        cv::Scalar t1(c1);
        cv::Scalar t2(c2);
        t1[0]=0;
        t2[0]=0;
        dist = cv::norm(t1,t2,cv::NORM_L2);
    }
    else{
        dist = cv::norm(c1,c2,cv::NORM_L2);
    }
    return dist;
}

bool LevelTracker::extractRotatedRoi(const cv::Mat &src, cv::Mat &dst, const cv::Point &p0_, const cv::Point &p1_, const cv::Point &p2_)
{
    //Referências:
    //https://en.wikipedia.org/wiki/Transformation_matrix#Affine_transformations
    //http://answers.opencv.org/question/497/extract-a-rotatedrect-area/

    //cv::RotatedRect rotRect(p0_,p1_,p2_);
    const float height = sqrt(pow(p0_.x-p1_.x,2)+pow(p0_.y-p1_.y,2));
    const float width = sqrt(pow(p1_.x-p2_.x,2)+pow(p1_.y-p2_.y,2));
    const float a = atan(((float)(p1_.x)-p0_.x)/(p1_.y-p0_.y));
    const float tx = -(p0_.x);
    const float ty = -(p0_.y);
    const float cosa = cos(a);
    const float sina = sin(a);
    const cv::Mat M = (cv::Mat_<float>(2,3) << cosa, -sina, cosa*tx-sina*ty,
                                            sin(a), cos(a), sina*tx+cosa*ty);
    const cv::Size s(width,height);
    cv::warpAffine(src,dst,M,s);
    //cv::imshow("rot",dst);
    //cv::waitKey(1);

    //Verifica se saiu da imagem
    const cv::Rect roi(p0_,p2_);
    const cv::Rect imgRect(0,0,src.cols-1,src.rows-1);
    const cv::Rect r(roi&imgRect); //intersection
    return (r!=roi);
}

}
