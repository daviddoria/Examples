#include <QtGui>
#include <qtimer.h>

#include "form.h"
#include <iostream>

MainWindow::MainWindow()
{
  ui.setupUi(this);
}

void MainWindow::on_btnStart_clicked()
{
  std::cout << "button clicked." << std::endl;
  QProgressDialog dialog;
  dialog.setMaximum(0);
  dialog.setMinimum(0);
  dialog.exec();
  
}
