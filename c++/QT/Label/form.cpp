#include <QtGui>

#include "form.h"
#include <iostream>

void Test(const QLabel& test)
{

}

MainWindow::MainWindow()
{
  setupUi(this);
  connect(this->btnSetText, SIGNAL(clicked()), this, SLOT(btnSetText_clicked()));
  QLabel test;
  Test(test);
}


void MainWindow::btnSetText_clicked()
{
  this->lblLabel->setText("test");
}