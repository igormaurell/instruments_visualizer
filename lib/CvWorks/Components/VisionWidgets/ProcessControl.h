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

#ifndef PROCESSCONTROL_H
#define PROCESSCONTROL_H

#include <QGroupBox>
#include <QLabel>
#include <QPainter>
#include "VisionCore.h"
#include "ProcessWidget.h"
#include "FrameServerControlWidget.h"

/// Esta classe � respons�vel por controlar a execu��o de um algoritmo de vis�o computacional.
/** A classe n�o especifica como o algoritmo � executado. Ela apenas serve de base para outras classes e prove acesso a um FrameServerControlWidget
 * (que fornece uma fonte de video) e um widget com funcionalidades b�sicas de controle (fechar, pausar, definir cor de desenho).
 *
 *  A classe tamb�m fornece m�todos para adi��o de widgets que facilitam a entrada de dados pelo usu�rio (ex. addLeftClickHandler, addNumericParameter).
 *
 * \ingroup VisionGUI
 */
class ProcessControl
{
public:
    /// Construtor padr�o.
    ProcessControl();

    /// Construtor.
    ProcessControl(FrameServerControlWidget* frameServerControl_);

    /// Destrutor
    virtual ~ProcessControl();

    /// Retorna o widget (parte gr�fica) do processo.
    ProcessWidget* getWidget() const;

    /// Retorna a cor padr�o para desenhar os resultados
    QColor getDrawColor() const;

    /// Retorna o nome do processo.
    QString getName() const;

    /// Altera o nome do processo.
    void setName(const QString &value);

    /// Pause process.
    virtual void pause() = 0;

    /// Start process.
    virtual void start() = 0;

    /// Sincroniza o m�todo com o v�deo (a captura de novos frames espera o processamento).
    virtual void sync(bool s) = 0;

    /// Adiciona um widget para um par�metro do tipo double. Quando o par�metro � alterado a fun��o setParam � chamada.
    void addNumericParameter(const QString name, const std::function<void (double)> setParam, const double min=0.0, const double max=1.0,const double initial=0.5, const int steps=100, const int style=0);

    /// Adiciona um widget para um par�metro do tipo double. Quando o par�metro � alterado o valor da vari�vel param � alterado.
    void addNumericParameter(const QString name, double& param, const double min=0.0, const double max=1.0,const double initial=0.5, const int steps=100, const int style=0);

    /// Adiciona um widget que executa a fun��o handlerFcn quando um ponto da imagem � clicado.
    void addLeftClickHandler(const QString name, std::function<void (QPoint)> handlerFcn,bool enabled_=false);

    /// Adiciona um widget que captura numOfPoints cliques na imagem e chama a fun��o handlerFcn repassando os pontos clicados.
    void addMultiClickHandler(const QString name, const unsigned int numOfPoints, std::function<void (std::vector<QPoint>)> handlerFcn,bool enabled_=false);

    /// Adiciona um widget que captura uma sub-imagem baseada na sele��o do usu�rio.
    void addSubImageHandler(const QString name, std::function<void (cv::Mat img)> handlerFcn, bool enabled_=false);

    /// Adiciona um widget que captura uma sub-imagem baseada na sele��o do usu�rio.
    void addSubImageAndRectHandler(const QString name, std::function<void (cv::Mat img,cv::Rect r)> handlerFcn, bool enabled_=false);

    /// Adiciona um widget que possibilita a defini��o do path de um arquivo. Repassa o path do arquivo a fun��o handlerFcn.
    void addFileChoice(const QString name, std::function<void (const std::string)> handlerFcn);

    /// Adiciona um widget para um par�metro do tipo int. Quando o par�metro � alterado o valor da vari�vel param � alterado.
    void addNumericParameter(const QString name, int &param, const int min=0, const int max=100, const int initial=50);

    /// Adiciona um widget para um par�metro do tipo int. Quando o par�metro � alterado a fun��o setParam � chamada.
    void addNumericParameter(const QString name, const std::function<void (int)> setParam, const int min=0, const int max=100, const int initial=50);

    /// Adiciona um widget para um par�metro do tipo bool.
    void addBoolParameter(const QString name, bool &param);

    /// Adiciona um widget para um par�metro do tipo bool.
    void addBoolParameter(const QString name, const std::function<void(bool)> setParam,const bool initial);

    /// Exports the log data to file.
    virtual bool exportLog(std::string fileName);

    /// Fun��o que reseta o log.
    virtual void resetLog();

    /// Serializa o processo para um data stream.
    virtual bool save(QDataStream& s);

    /// Carrega o processo de um data stream.
    virtual bool load(QDataStream& s);

    /// Retorna o tipo do processo.
    virtual QString type() const ;

    FrameServerControlWidget *getFrameServerControl() const;

protected:
    /// Objeto que representa uma fonte de video.
    FrameServerControlWidget* frameServerControl;

    /// Widget com funcionalidades b�sicas de controle (fechar, pausar, definir cor de desenho).
    ProcessWidget* widget;

    /// Vetor com conex�es do Qt que devem ser desconectadas no destrutor.
    std::vector<QMetaObject::Connection> connections;

    /// Nome de exibi��o do processo em execu��o.
    QString name;

    /// Define status de execu��o do processo.
    void setStatus(const QString& status);


private:
    /// Fun��o que desenha os resultados. Chamada quando uma nova imagem do frameServerControl estiver disponivel.
    virtual void drawResults(QPainter* painter);


};

#endif // PROCESSCONTROL_H
