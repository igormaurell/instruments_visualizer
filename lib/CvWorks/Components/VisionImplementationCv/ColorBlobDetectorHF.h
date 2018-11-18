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

#include "VisionImplementationCv.h"

namespace Viscv
{

/// Implementa um detector de blobs baseado em cor.
/**
O detector executa os seguintes passos:
1) Aplica um filtro de cor, baseado em um m�nimo e m�ximo.
2) Aplica dilata��o e eros�o para fechar buracos e eliminar pequenos blobs.
3) Encontra blobs
4) Filtra blobs baseado em caracter�sticas (tamanho e forma)

Obs: A OpenCv possui uma implementa��o de detector de blobs na classe cv::SimpleBlobDetector.

\ingroup Viscv
*/
class ColorBlobDetectorHF :
    public VisionCore::Detector<cv::Mat,std::vector<cv::Point>>
{
public:
	/// Construtor default.
	ColorBlobDetectorHF(void);

	/// Destrutor.
	virtual ~ColorBlobDetectorHF(void);

	/// Detecta blobs em uma imagem.
    std::vector<std::vector<cv::Point>> detect(const cv::Mat& img) const;

	/// Define a cor m�nima que ser� filtrada.
	void setMinHsv(const cv::Scalar& minHsv_);

	/// Define a cor m�xima que ser� filtrada.
	void setMaxHsv(const cv::Scalar& maxHsv_);

	/// Retorna a cor m�nima que ser� filtrada
	const cv::Scalar& getMinHsv() const;

	/// Retorna a cor m�xima que ser� filtrada
	const cv::Scalar& getMaxHsv() const;

	/// Di�metro do kernel (circular) usado na eros�o.
	int erodeSize;

	/// Di�metro do kernel (circular) usado na dilata��o.
	int dilateSize;

	/// Define se as imagens intermedi�rias de processamento devem ser mostradas.
	bool showProcessingImgs;

private:
	cv::Scalar minHsv;
	cv::Scalar maxHsv;
};

}
