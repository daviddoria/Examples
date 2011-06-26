#include <QtGui>

#include "form.h"
#include <iostream>

MainWindow::MainWindow()
{
  setupUi(this);
  connect(this->btnSetText, SIGNAL(clicked()), this, SLOT(btnSetText_clicked()));
}

void MainWindow::btnSetText_clicked()
{
  this->lblLabel->setText("test");
}