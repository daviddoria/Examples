#include <QtGui>
#include <iostream>

#include "main.h"

MainWindow::MainWindow(QMainWindow* parent)
    : QMainWindow(parent)
{
  setupUi(this);
  this->comboBox->addItem("test");
}

void MainWindow::on_comboBox_activated(int value)
{
  std::cout << "Selected index " << this->comboBox->currentIndex()
	    << " : " << this->comboBox->currentText().toStdString() << std::endl;
}
