#include <QtGui>

#include "form.h"
#include <iostream>

MainWindow::MainWindow()
{
  setupUi(this);
  QDate currentDate = this->calendarWidget->selectedDate();
  std::cout << "currentDate: " << currentDate.toString().toStdString() << std::endl;
}

void MainWindow::on_calendarWidget_clicked ( const QDate & date )
{
  std::cout << "clicked: " << date.toString().toStdString() << std::endl;
}
