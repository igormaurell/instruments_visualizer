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

#ifndef _CIRCLETRACKERHTCF_H
#define _CIRCLETRACKERHTCF_H

#include "VisionImplementationCv.h"
#include "CircleDetectorHTCF.h"

namespace Viscv
{

/// Rastreador baseado na classe CircleDetectorHTCF.
/** O rastreamento � feito limitando a regi�o de interesse (ROI) de aplica��o do detector em cada frame e filtrando o tamanho dos circulos. 
Ou seja, assume-se que houve apenas varia��o local da posi��o e tamanho do circulo

\ingroup Viscv
.*/
class CircleTrackerHTCF
    : public VisionCore::Tracker<cv::Mat,Circle<>>
{
private:

    Circle<> m_lastTrack;
    bool m_lostTrack;


public:
    /// Constructor method.
    CircleTrackerHTCF(int x, int y, int radius);
    /// Destructor method.
    virtual ~CircleTrackerHTCF();
    const Circle<>& getLastTrack();
    bool lostTrack() const;
    /// Given an image (i.e. video frame), update the tracked objects.
    void update(const VisionCore::Frame<cv::Mat>& frame);
};


inline bool CircleTrackerHTCF::lostTrack() const
{
    return m_lostTrack;
}

}
#endif // ndef _CIRCLETRACKERHTCF_H
