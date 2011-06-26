#include <QtGui>
#include <iostream>

#include "main.h"

MainWindow::MainWindow(QMainWindow* parent)
    : QMainWindow(parent)
{
    ui.setupUi(this);
    connect( this->ui.comboBox, SIGNAL( activated(int) ), this, SLOT(comboBox_Activated()) );
}

void MainWindow::comboBox_Activated()
{
  std::cout << "Selected index " << this->ui.comboBox->currentIndex() 
	    << " : " << this->ui.comboBox->currentText().toStdString() << std::endl;
}
