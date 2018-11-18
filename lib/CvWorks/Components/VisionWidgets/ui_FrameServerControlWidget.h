/********************************************************************************
** Form generated from reading UI file 'FrameServerControlWidget.ui'
**
** Created by: Qt User Interface Compiler version 5.11.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_FRAMESERVERCONTROLWIDGET_H
#define UI_FRAMESERVERCONTROLWIDGET_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QDoubleSpinBox>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSlider>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_FrameServerControlWidget
{
public:
    QVBoxLayout *verticalLayout;
    QGroupBox *groupBox;
    QVBoxLayout *verticalLayout_2;
    QHBoxLayout *horizontalLayout;
    QPushButton *closeBtn;
    QPushButton *stopBtn;
    QPushButton *startBtn;
    QHBoxLayout *horizontalLayout_2;
    QLabel *label_2;
    QDoubleSpinBox *fpsInput;
    QLabel *label;
    QSlider *scaleSlider;
    QCheckBox *showImageCheckBox;

    void setupUi(QWidget *FrameServerControlWidget)
    {
        if (FrameServerControlWidget->objectName().isEmpty())
            FrameServerControlWidget->setObjectName(QStringLiteral("FrameServerControlWidget"));
        FrameServerControlWidget->resize(277, 202);
        QSizePolicy sizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(FrameServerControlWidget->sizePolicy().hasHeightForWidth());
        FrameServerControlWidget->setSizePolicy(sizePolicy);
        FrameServerControlWidget->setMinimumSize(QSize(180, 0));
        FrameServerControlWidget->setMaximumSize(QSize(277, 16777215));
        verticalLayout = new QVBoxLayout(FrameServerControlWidget);
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        groupBox = new QGroupBox(FrameServerControlWidget);
        groupBox->setObjectName(QStringLiteral("groupBox"));
        QSizePolicy sizePolicy1(QSizePolicy::Preferred, QSizePolicy::Expanding);
        sizePolicy1.setHorizontalStretch(0);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(groupBox->sizePolicy().hasHeightForWidth());
        groupBox->setSizePolicy(sizePolicy1);
        groupBox->setAutoFillBackground(true);
        verticalLayout_2 = new QVBoxLayout(groupBox);
        verticalLayout_2->setObjectName(QStringLiteral("verticalLayout_2"));
        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setObjectName(QStringLiteral("horizontalLayout"));
        closeBtn = new QPushButton(groupBox);
        closeBtn->setObjectName(QStringLiteral("closeBtn"));

        horizontalLayout->addWidget(closeBtn);

        stopBtn = new QPushButton(groupBox);
        stopBtn->setObjectName(QStringLiteral("stopBtn"));

        horizontalLayout->addWidget(stopBtn);

        startBtn = new QPushButton(groupBox);
        startBtn->setObjectName(QStringLiteral("startBtn"));
        sizePolicy.setHeightForWidth(startBtn->sizePolicy().hasHeightForWidth());
        startBtn->setSizePolicy(sizePolicy);

        horizontalLayout->addWidget(startBtn);


        verticalLayout_2->addLayout(horizontalLayout);

        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setObjectName(QStringLiteral("horizontalLayout_2"));
        label_2 = new QLabel(groupBox);
        label_2->setObjectName(QStringLiteral("label_2"));

        horizontalLayout_2->addWidget(label_2);

        fpsInput = new QDoubleSpinBox(groupBox);
        fpsInput->setObjectName(QStringLiteral("fpsInput"));
        fpsInput->setDecimals(1);
        fpsInput->setMinimum(0.1);
        fpsInput->setMaximum(100);
        fpsInput->setValue(30);

        horizontalLayout_2->addWidget(fpsInput);


        verticalLayout_2->addLayout(horizontalLayout_2);

        label = new QLabel(groupBox);
        label->setObjectName(QStringLiteral("label"));

        verticalLayout_2->addWidget(label);

        scaleSlider = new QSlider(groupBox);
        scaleSlider->setObjectName(QStringLiteral("scaleSlider"));
        QSizePolicy sizePolicy2(QSizePolicy::Preferred, QSizePolicy::Preferred);
        sizePolicy2.setHorizontalStretch(0);
        sizePolicy2.setVerticalStretch(0);
        sizePolicy2.setHeightForWidth(scaleSlider->sizePolicy().hasHeightForWidth());
        scaleSlider->setSizePolicy(sizePolicy2);
        scaleSlider->setMinimum(10);
        scaleSlider->setMaximum(200);
        scaleSlider->setSliderPosition(100);
        scaleSlider->setOrientation(Qt::Horizontal);
        scaleSlider->setTickPosition(QSlider::TicksBelow);
        scaleSlider->setTickInterval(10);

        verticalLayout_2->addWidget(scaleSlider);

        showImageCheckBox = new QCheckBox(groupBox);
        showImageCheckBox->setObjectName(QStringLiteral("showImageCheckBox"));
        showImageCheckBox->setChecked(true);

        verticalLayout_2->addWidget(showImageCheckBox);


        verticalLayout->addWidget(groupBox);


        retranslateUi(FrameServerControlWidget);

        QMetaObject::connectSlotsByName(FrameServerControlWidget);
    } // setupUi

    void retranslateUi(QWidget *FrameServerControlWidget)
    {
        FrameServerControlWidget->setWindowTitle(QApplication::translate("FrameServerControlWidget", "Form", nullptr));
        groupBox->setTitle(QApplication::translate("FrameServerControlWidget", "GroupBox", nullptr));
        closeBtn->setText(QApplication::translate("FrameServerControlWidget", "Close", nullptr));
        stopBtn->setText(QApplication::translate("FrameServerControlWidget", "Stop", nullptr));
        startBtn->setText(QApplication::translate("FrameServerControlWidget", "Start", nullptr));
        label_2->setText(QApplication::translate("FrameServerControlWidget", "Frames per second:", nullptr));
        label->setText(QApplication::translate("FrameServerControlWidget", "Zoom:", nullptr));
        showImageCheckBox->setText(QApplication::translate("FrameServerControlWidget", "Show image", nullptr));
    } // retranslateUi

};

namespace Ui {
    class FrameServerControlWidget: public Ui_FrameServerControlWidget {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_FRAMESERVERCONTROLWIDGET_H
