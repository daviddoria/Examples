/********************************************************************************
** Form generated from reading UI file 'FloatSlider.ui'
**
** Created: Sat Dec 3 17:17:04 2011
**      by: Qt User Interface Compiler version 4.7.4
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_FLOATSLIDER_H
#define UI_FLOATSLIDER_H

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

class Ui_FloatSlider
{
public:
    QGridLayout *gridLayout;
    QLabel *lblMin;
    QSlider *horizontalSlider;
    QLabel *lblMax;
    QLabel *lblCurrent;

    void setupUi(QWidget *FloatSlider)
    {
        if (FloatSlider->objectName().isEmpty())
            FloatSlider->setObjectName(QString::fromUtf8("FloatSlider"));
        FloatSlider->resize(210, 50);
        gridLayout = new QGridLayout(FloatSlider);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        lblMin = new QLabel(FloatSlider);
        lblMin->setObjectName(QString::fromUtf8("lblMin"));

        gridLayout->addWidget(lblMin, 0, 0, 1, 1);

        horizontalSlider = new QSlider(FloatSlider);
        horizontalSlider->setObjectName(QString::fromUtf8("horizontalSlider"));
        horizontalSlider->setMaximum(100);
        horizontalSlider->setOrientation(Qt::Horizontal);

        gridLayout->addWidget(horizontalSlider, 0, 1, 1, 1);

        lblMax = new QLabel(FloatSlider);
        lblMax->setObjectName(QString::fromUtf8("lblMax"));

        gridLayout->addWidget(lblMax, 0, 2, 1, 1);

        lblCurrent = new QLabel(FloatSlider);
        lblCurrent->setObjectName(QString::fromUtf8("lblCurrent"));
        lblCurrent->setAlignment(Qt::AlignCenter);

        gridLayout->addWidget(lblCurrent, 1, 1, 1, 1);


        retranslateUi(FloatSlider);

        QMetaObject::connectSlotsByName(FloatSlider);
    } // setupUi

    void retranslateUi(QWidget *FloatSlider)
    {
        FloatSlider->setWindowTitle(QApplication::translate("FloatSlider", "Form", 0, QApplication::UnicodeUTF8));
        lblMin->setText(QApplication::translate("FloatSlider", "min", 0, QApplication::UnicodeUTF8));
        lblMax->setText(QApplication::translate("FloatSlider", "max", 0, QApplication::UnicodeUTF8));
        lblCurrent->setText(QApplication::translate("FloatSlider", "current", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class FloatSlider: public Ui_FloatSlider {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_FLOATSLIDER_H
