#include <QtGui>
#include <qtimer.h>

#include "form.h"
#include <iostream>

MainWindow::MainWindow()
{
  ui.setupUi(this);
  this->ui.progressBar->setValue(0);
  QTimer *timer = new QTimer(this);
  connect(timer, SIGNAL(timeout()), this, SLOT(TimerEvent()));
  timer->start(1000);

}

void MainWindow::TimerEvent()
{
  
  std::cout << "Timer event." << std::endl;
  int value = this->ui.progressBar->value();
  this->ui.progressBar->setValue(value+1);
}
