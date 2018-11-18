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

#ifndef LEVELTRACKERAUTO_H
#define LEVELTRACKERAUTO_H

#include "VisionSensors.h"

namespace VisionSense {

/// Rastreador de nível de um líquido ou sólido dentro de um container com ajuste automatico.
/** Um container é definido na imagem por um retângulo através de um ponto esquerdo-superior e outro ponto
 * direito inferior.
 *
 * O método infere o nível de preenchimento do container através da procura por um nível que melhor separa
 * o container em duas partes homogeneas e semelhantes.
 *
 * \ingroup VisionSensors
*/
class LevelTrackerAuto :
    public VisionCore::Tracker<cv::Mat,double>
{
public:
    LevelTrackerAuto(void);
    virtual ~LevelTrackerAuto(void);

    void update(const VisionCore::Frame<cv::Mat>& frame);

    const double& getLastTrack();

    bool lostTrack() const;

    /// Sets the region were the level will be measured.
    void setRegion(const cv::Rect& roi);

    cv::Point topLeft;
    cv::Point bottonRight;

private:
    double level;
    bool regOk;

};

}

#endif // LEVELTRACKERAUTO_H
