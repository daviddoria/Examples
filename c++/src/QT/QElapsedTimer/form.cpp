#include <QtGui>

#include "form.h"
#include <iostream>

MainWindow::MainWindow()
{
  setupUi(this);
  connect(this->btnStart, SIGNAL(clicked()), this, SLOT(btnStart_clicked()));
  connect(this->btnStop, SIGNAL(clicked()), this, SLOT(btnStop_clicked()));

}

void MainWindow::btnStart_clicked()
{
  std::cout << "Start button clicked." << std::endl;

  this->timer.start();
}

void MainWindow::btnStop_clicked()
{
  std::cout << this->timer.elapsed() << "milliseconds have passed." << std::endl;

  QTime mytime = QTime(0, 0, 0).addMSecs(this->timer.elapsed());
  std::cout << mytime.toString("hh:mm:ss").toStdString() << std::endl;
}
