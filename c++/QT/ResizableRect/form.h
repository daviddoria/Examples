#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QtGui>
#include "DragableRect.h"
#include <iostream>

/********************************************************************************
** Form generated from reading ui file 'mainwindow.ui'
**
** Created: Mon 25. Jan 12:57:15 2010
**  	by: Qt User Interface Compiler version 4.5.3
**
** WARNING! All changes made in this file will be lost when recompiling ui file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QGraphicsView>
#include <QtGui/QHeaderView>
#include <QtGui/QMainWindow>
#include <QtGui/QPushButton>
#include <QtGui/QVBoxLayout>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
	QWidget *centralWidget;
	QVBoxLayout *verticalLayout;
	QGraphicsView *canvas;
	QPushButton *addButton;

	void setupUi(QMainWindow *MainWindow)
	{
    	if (MainWindow->objectName().isEmpty())
        	MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
    	MainWindow->resize(828, 451);
    	QSizePolicy sizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
    	sizePolicy.setHorizontalStretch(0);
    	sizePolicy.setVerticalStretch(0);
    	sizePolicy.setHeightForWidth(MainWindow->sizePolicy().hasHeightForWidth());
    	MainWindow->setSizePolicy(sizePolicy);
    	centralWidget = new QWidget(MainWindow);
    	centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
    	verticalLayout = new QVBoxLayout(centralWidget);
    	verticalLayout->setSpacing(0);
    	verticalLayout->setMargin(0);
    	verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
    	canvas = new QGraphicsView(centralWidget);
    	canvas->setObjectName(QString::fromUtf8("canvas"));
    	QSizePolicy sizePolicy1(QSizePolicy::Expanding, QSizePolicy::Expanding);
    	sizePolicy1.setHorizontalStretch(20);
    	sizePolicy1.setVerticalStretch(20);
    	sizePolicy1.setHeightForWidth(canvas->sizePolicy().hasHeightForWidth());
    	canvas->setSizePolicy(sizePolicy1);
    	canvas->setAlignment(Qt::AlignLeading|Qt::AlignLeft|Qt::AlignTop);

    	verticalLayout->addWidget(canvas);

    	addButton = new QPushButton(centralWidget);
    	addButton->setObjectName(QString::fromUtf8("addButton"));

    	verticalLayout->addWidget(addButton);

    	MainWindow->setCentralWidget(centralWidget);

    	retranslateUi(MainWindow);

    	QMetaObject::connectSlotsByName(MainWindow);
	} // setupUi

	void retranslateUi(QMainWindow *MainWindow)
	{
    	MainWindow->setWindowTitle(QApplication::translate("MainWindow", "MainWindow", 0, QApplication::UnicodeUTF8));
    	addButton->setText(QApplication::translate("MainWindow", "add", 0, QApplication::UnicodeUTF8));
    	Q_UNUSED(MainWindow);
	} // retranslateUi

};

namespace Ui {
	class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H

namespace Ui
{
class MainWindow;
}

class MainWindow : public QMainWindow
{
Q_OBJECT

public:
MainWindow(QWidget *parent = 0);
~MainWindow();

protected:
Ui::MainWindow *ui;
QList<QGraphicsItem *> *items;
QGraphicsScene *scene;
int itemWidth;
int itemHeight;

public slots:
void slot_addItem();
};

#endif // MAINWINDOW_H