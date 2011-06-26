#include <QtGui>
//#include <qtimer.h>
#include <QTimer>

#include "form.h"
#include <iostream>

MainWindow::MainWindow()
{
  ui.setupUi(this);
}

void MainWindow::on_btnStart_clicked()
{
  std::cout << "button clicked." << std::endl;
  
  //QTimer* timer = new QTimer(this);
  connect(&timer, SIGNAL(timeout()), this, SLOT(TimerEvent()));
  this->timer.start(1000);
  
  //QProgressDialog dialog;
  this->dialog.setMaximum(0);
  this->dialog.setMinimum(0);
  this->dialog.exec();
  
}

void MainWindow::TimerEvent()
{
  std::cout << "Timer expired." << std::endl; 
  this->dialog.cancel();
  this->timer.stop();
}