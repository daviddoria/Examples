#include <QtGui>

#include "form.h"
#include <iostream>

MainWindow::MainWindow()
{
  setupUi(this);

  QTime time1(10, 12); // ( int h, int m, int s = 0, int ms = 0 )
  QTime time2(10, 14);
  
  int difference = time2.secsTo(time1);
  std::cout << "difference: " << difference << std::endl;
  
  std::cout << "time1: " << time1.toString(Qt::TextDate).toStdString() << std::endl;
  std::cout << "time1: " << time1.toString("hh:mm AP").toStdString() << std::endl;
}
