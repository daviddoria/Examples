#include <QtGui>

#include "form.h"
#include <iostream>

MainWindow::MainWindow()
{
  setupUi(this);
  connect(this->btnDisplayTime, SIGNAL(clicked()), this, SLOT(btnDisplayTime_clicked()));

}

void MainWindow::btnDisplayTime_clicked()
{
  this->timer.start();
  QString format = "hh:mm:ss";
  std::cout << this->timer.toString(format).toStdString() << std::endl;
  this->lblTime->setText(this->timer.toString(format).toStdString().c_str());
  this->lblTime->setText(this->timer.toString(format));
}
