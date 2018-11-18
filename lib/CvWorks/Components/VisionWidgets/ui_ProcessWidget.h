/********************************************************************************
** Form generated from reading UI file 'ProcessWidget.ui'
**
** Created by: Qt User Interface Compiler version 5.11.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_PROCESSWIDGET_H
#define UI_PROCESSWIDGET_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QFormLayout>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QRadioButton>
#include <QtWidgets/QSpinBox>
#include <QtWidgets/QVBoxLayout>

QT_BEGIN_NAMESPACE

class Ui_ProcessWidget
{
public:
    QVBoxLayout *verticalLayout;
    QGroupBox *groupBox;
    QVBoxLayout *verticalLayout_4;
    QFormLayout *formLayout;
    QLabel *label_5;
    QLineEdit *nameEdit;
    QHBoxLayout *horizontalLayout;
    QPushButton *colorBtn;
    QPushButton *startBtn;
    QPushButton *pauseBtn;
    QPushButton *closeBtn;
    QFormLayout *formLayout_4;
    QLabel *label_4;
    QLabel *statusLabel;
    QLabel *label_3;
    QLabel *maxProcessingFpsLabel;
    QLabel *label;
    QLabel *procTimeLabel;
    QCheckBox *syncCheckBox;
    QCheckBox *keepLogCheckBtn;
    QHBoxLayout *horizontalLayout_3;
    QVBoxLayout *verticalLayout_2;
    QPushButton *exportLogBtn;
    QPushButton *resetLogBtn;
    QVBoxLayout *verticalLayout_3;
    QRadioButton *extortCsvRadioBtn;
    QRadioButton *exportXmlRadioBtn;
    QHBoxLayout *horizontalLayout_4;
    QCheckBox *drawCheckBtn;
    QLabel *label_2;
    QSpinBox *drawHistSize;
    QGroupBox *methodParametersGroup;
    QVBoxLayout *verticalLayout_5;

    void setupUi(QGroupBox *ProcessWidget)
    {
        if (ProcessWidget->objectName().isEmpty())
            ProcessWidget->setObjectName(QStringLiteral("ProcessWidget"));
        ProcessWidget->resize(220, 329);
        QSizePolicy sizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(ProcessWidget->sizePolicy().hasHeightForWidth());
        ProcessWidget->setSizePolicy(sizePolicy);
        ProcessWidget->setMinimumSize(QSize(220, 0));
        ProcessWidget->setMaximumSize(QSize(250, 16777215));
        verticalLayout = new QVBoxLayout(ProcessWidget);
        verticalLayout->setSpacing(6);
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        groupBox = new QGroupBox(ProcessWidget);
        groupBox->setObjectName(QStringLiteral("groupBox"));
        verticalLayout_4 = new QVBoxLayout(groupBox);
        verticalLayout_4->setObjectName(QStringLiteral("verticalLayout_4"));
        formLayout = new QFormLayout();
        formLayout->setObjectName(QStringLiteral("formLayout"));
        label_5 = new QLabel(groupBox);
        label_5->setObjectName(QStringLiteral("label_5"));

        formLayout->setWidget(0, QFormLayout::LabelRole, label_5);

        nameEdit = new QLineEdit(groupBox);
        nameEdit->setObjectName(QStringLiteral("nameEdit"));

        formLayout->setWidget(0, QFormLayout::FieldRole, nameEdit);


        verticalLayout_4->addLayout(formLayout);

        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setObjectName(QStringLiteral("horizontalLayout"));
        colorBtn = new QPushButton(groupBox);
        colorBtn->setObjectName(QStringLiteral("colorBtn"));

        horizontalLayout->addWidget(colorBtn);

        startBtn = new QPushButton(groupBox);
        startBtn->setObjectName(QStringLiteral("startBtn"));

        horizontalLayout->addWidget(startBtn);

        pauseBtn = new QPushButton(groupBox);
        pauseBtn->setObjectName(QStringLiteral("pauseBtn"));
        pauseBtn->setMaximumSize(QSize(30, 16777215));

        horizontalLayout->addWidget(pauseBtn);

        closeBtn = new QPushButton(groupBox);
        closeBtn->setObjectName(QStringLiteral("closeBtn"));
        closeBtn->setMaximumSize(QSize(30, 16777215));

        horizontalLayout->addWidget(closeBtn);


        verticalLayout_4->addLayout(horizontalLayout);

        formLayout_4 = new QFormLayout();
        formLayout_4->setObjectName(QStringLiteral("formLayout_4"));
        label_4 = new QLabel(groupBox);
        label_4->setObjectName(QStringLiteral("label_4"));

        formLayout_4->setWidget(0, QFormLayout::LabelRole, label_4);

        statusLabel = new QLabel(groupBox);
        statusLabel->setObjectName(QStringLiteral("statusLabel"));

        formLayout_4->setWidget(0, QFormLayout::FieldRole, statusLabel);

        label_3 = new QLabel(groupBox);
        label_3->setObjectName(QStringLiteral("label_3"));

        formLayout_4->setWidget(1, QFormLayout::LabelRole, label_3);

        maxProcessingFpsLabel = new QLabel(groupBox);
        maxProcessingFpsLabel->setObjectName(QStringLiteral("maxProcessingFpsLabel"));

        formLayout_4->setWidget(1, QFormLayout::FieldRole, maxProcessingFpsLabel);

        label = new QLabel(groupBox);
        label->setObjectName(QStringLiteral("label"));

        formLayout_4->setWidget(2, QFormLayout::LabelRole, label);

        procTimeLabel = new QLabel(groupBox);
        procTimeLabel->setObjectName(QStringLiteral("procTimeLabel"));

        formLayout_4->setWidget(2, QFormLayout::FieldRole, procTimeLabel);


        verticalLayout_4->addLayout(formLayout_4);

        syncCheckBox = new QCheckBox(groupBox);
        syncCheckBox->setObjectName(QStringLiteral("syncCheckBox"));

        verticalLayout_4->addWidget(syncCheckBox);

        keepLogCheckBtn = new QCheckBox(groupBox);
        keepLogCheckBtn->setObjectName(QStringLiteral("keepLogCheckBtn"));
        keepLogCheckBtn->setChecked(true);

        verticalLayout_4->addWidget(keepLogCheckBtn);

        horizontalLayout_3 = new QHBoxLayout();
        horizontalLayout_3->setObjectName(QStringLiteral("horizontalLayout_3"));
        verticalLayout_2 = new QVBoxLayout();
        verticalLayout_2->setObjectName(QStringLiteral("verticalLayout_2"));
        exportLogBtn = new QPushButton(groupBox);
        exportLogBtn->setObjectName(QStringLiteral("exportLogBtn"));

        verticalLayout_2->addWidget(exportLogBtn);

        resetLogBtn = new QPushButton(groupBox);
        resetLogBtn->setObjectName(QStringLiteral("resetLogBtn"));

        verticalLayout_2->addWidget(resetLogBtn);


        horizontalLayout_3->addLayout(verticalLayout_2);

        verticalLayout_3 = new QVBoxLayout();
        verticalLayout_3->setObjectName(QStringLiteral("verticalLayout_3"));
        extortCsvRadioBtn = new QRadioButton(groupBox);
        extortCsvRadioBtn->setObjectName(QStringLiteral("extortCsvRadioBtn"));
        extortCsvRadioBtn->setChecked(true);

        verticalLayout_3->addWidget(extortCsvRadioBtn);

        exportXmlRadioBtn = new QRadioButton(groupBox);
        exportXmlRadioBtn->setObjectName(QStringLiteral("exportXmlRadioBtn"));
        exportXmlRadioBtn->setEnabled(false);

        verticalLayout_3->addWidget(exportXmlRadioBtn);


        horizontalLayout_3->addLayout(verticalLayout_3);


        verticalLayout_4->addLayout(horizontalLayout_3);

        horizontalLayout_4 = new QHBoxLayout();
        horizontalLayout_4->setObjectName(QStringLiteral("horizontalLayout_4"));
        drawCheckBtn = new QCheckBox(groupBox);
        drawCheckBtn->setObjectName(QStringLiteral("drawCheckBtn"));
        QSizePolicy sizePolicy1(QSizePolicy::Preferred, QSizePolicy::Preferred);
        sizePolicy1.setHorizontalStretch(0);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(drawCheckBtn->sizePolicy().hasHeightForWidth());
        drawCheckBtn->setSizePolicy(sizePolicy1);
        drawCheckBtn->setChecked(true);
        drawCheckBtn->setTristate(false);

        horizontalLayout_4->addWidget(drawCheckBtn);

        label_2 = new QLabel(groupBox);
        label_2->setObjectName(QStringLiteral("label_2"));

        horizontalLayout_4->addWidget(label_2);

        drawHistSize = new QSpinBox(groupBox);
        drawHistSize->setObjectName(QStringLiteral("drawHistSize"));
        drawHistSize->setMinimum(1);
        drawHistSize->setMaximum(50);

        horizontalLayout_4->addWidget(drawHistSize);


        verticalLayout_4->addLayout(horizontalLayout_4);

        keepLogCheckBtn->raise();
        syncCheckBox->raise();

        verticalLayout->addWidget(groupBox);

        methodParametersGroup = new QGroupBox(ProcessWidget);
        methodParametersGroup->setObjectName(QStringLiteral("methodParametersGroup"));
        methodParametersGroup->setEnabled(true);
        verticalLayout_5 = new QVBoxLayout(methodParametersGroup);
        verticalLayout_5->setObjectName(QStringLiteral("verticalLayout_5"));

        verticalLayout->addWidget(methodParametersGroup);


        retranslateUi(ProcessWidget);

        QMetaObject::connectSlotsByName(ProcessWidget);
    } // setupUi

    void retranslateUi(QGroupBox *ProcessWidget)
    {
        ProcessWidget->setWindowTitle(QApplication::translate("ProcessWidget", "GroupBox", nullptr));
        ProcessWidget->setTitle(QApplication::translate("ProcessWidget", "GroupBox", nullptr));
        groupBox->setTitle(QApplication::translate("ProcessWidget", "Process", nullptr));
        label_5->setText(QApplication::translate("ProcessWidget", "Name:", nullptr));
        colorBtn->setText(QString());
        startBtn->setText(QString());
        pauseBtn->setText(QApplication::translate("ProcessWidget", "PushButton", nullptr));
        closeBtn->setText(QApplication::translate("ProcessWidget", "PushButton", nullptr));
        label_4->setText(QApplication::translate("ProcessWidget", "Status:", nullptr));
        statusLabel->setText(QApplication::translate("ProcessWidget", "TextLabel", nullptr));
        label_3->setText(QApplication::translate("ProcessWidget", "Max fps: ", nullptr));
        maxProcessingFpsLabel->setText(QApplication::translate("ProcessWidget", "TextLabel", nullptr));
        label->setText(QApplication::translate("ProcessWidget", "Time / frame:", nullptr));
        procTimeLabel->setText(QApplication::translate("ProcessWidget", "TextLabel", nullptr));
        syncCheckBox->setText(QApplication::translate("ProcessWidget", "Sync with video", nullptr));
        keepLogCheckBtn->setText(QApplication::translate("ProcessWidget", "Keep log of results", nullptr));
        exportLogBtn->setText(QApplication::translate("ProcessWidget", "Export log...", nullptr));
        resetLogBtn->setText(QApplication::translate("ProcessWidget", "Reset log", nullptr));
        extortCsvRadioBtn->setText(QApplication::translate("ProcessWidget", "CSV", nullptr));
        exportXmlRadioBtn->setText(QApplication::translate("ProcessWidget", "XML", nullptr));
        drawCheckBtn->setText(QApplication::translate("ProcessWidget", "Draw results. ", nullptr));
        label_2->setText(QApplication::translate("ProcessWidget", "History:", nullptr));
        methodParametersGroup->setTitle(QApplication::translate("ProcessWidget", "Method Parameters", nullptr));
    } // retranslateUi

};

namespace Ui {
    class ProcessWidget: public Ui_ProcessWidget {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_PROCESSWIDGET_H
