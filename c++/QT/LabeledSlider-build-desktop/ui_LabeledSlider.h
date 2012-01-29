/********************************************************************************
** Form generated from reading UI file 'LabeledSlider.ui'
**
** Created: Sat Dec 3 17:14:58 2011
**      by: Qt User Interface Compiler version 4.7.4
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_LABELEDSLIDER_H
#define UI_LABELEDSLIDER_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QGridLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QSlider>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_LabeledSlider
{
public:
    QGridLayout *gridLayout;
    QLabel *lblMin;
    QSlider *horizontalSlider;
    QLabel *lblMax;
    QLabel *lblCurrent;

    void setupUi(QWidget *LabeledSlider)
    {
        if (LabeledSlider->objectName().isEmpty())
            LabeledSlider->setObjectName(QString::fromUtf8("LabeledSlider"));
        LabeledSlider->resize(210, 50);
        gridLayout = new QGridLayout(LabeledSlider);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        lblMin = new QLabel(LabeledSlider);
        lblMin->setObjectName(QString::fromUtf8("lblMin"));

        gridLayout->addWidget(lblMin, 0, 0, 1, 1);

        horizontalSlider = new QSlider(LabeledSlider);
        horizontalSlider->setObjectName(QString::fromUtf8("horizontalSlider"));
        horizontalSlider->setMaximum(100);
        horizontalSlider->setOrientation(Qt::Horizontal);

        gridLayout->addWidget(horizontalSlider, 0, 1, 1, 1);

        lblMax = new QLabel(LabeledSlider);
        lblMax->setObjectName(QString::fromUtf8("lblMax"));

        gridLayout->addWidget(lblMax, 0, 2, 1, 1);

        lblCurrent = new QLabel(LabeledSlider);
        lblCurrent->setObjectName(QString::fromUtf8("lblCurrent"));
        lblCurrent->setAlignment(Qt::AlignCenter);

        gridLayout->addWidget(lblCurrent, 1, 1, 1, 1);


        retranslateUi(LabeledSlider);

        QMetaObject::connectSlotsByName(LabeledSlider);
    } // setupUi

    void retranslateUi(QWidget *LabeledSlider)
    {
        LabeledSlider->setWindowTitle(QApplication::translate("LabeledSlider", "Form", 0, QApplication::UnicodeUTF8));
        lblMin->setText(QApplication::translate("LabeledSlider", "min", 0, QApplication::UnicodeUTF8));
        lblMax->setText(QApplication::translate("LabeledSlider", "max", 0, QApplication::UnicodeUTF8));
        lblCurrent->setText(QApplication::translate("LabeledSlider", "current", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class LabeledSlider: public Ui_LabeledSlider {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_LABELEDSLIDER_H
