#include <QtGui>
#include <QTime>

#include "form.h"
#include <iostream>

MainWindow::MainWindow()
{
  this->setupUi(this);
}

void MainWindow::on_btnStart_clicked()
{
  std::cout << "button clicked." << std::endl;

  QTime timer;
  timer.start();
  for(unsigned int i = 0; i < 1e8; ++i)
    {
    float a = i;
    }
  std::cout << timer.elapsed() << std::endl; // milliseconds

  std::cout << QTime().addMSecs(timer.elapsed()).second() << std::endl;


}
