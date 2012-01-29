#include <QtGui>
#include <qtimer.h>

#include "form.h"
#include <iostream>

MainWindow::MainWindow()
{
  ui.setupUi(this);
  this->ui.progressBar->setMinimum(0);
  this->ui.progressBar->setMaximum(0);
  this->ui.progressBar->hide();
}

void MainWindow::on_btnStop_clicked()
{
  //this->ui.progressBar->setVisible(false);
  this->ui.progressBar->hide();
}

void MainWindow::on_btnStart_clicked()
{
  this->ui.progressBar->show();
}
