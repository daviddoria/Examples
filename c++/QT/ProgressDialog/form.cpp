#include <QtGui>
#include <qtimer.h>

#include "form.h"
#include <iostream>

MainWindow::MainWindow()
{
  this->setupUi(this);
}

void MainWindow::on_btnStart_clicked()
{
  std::cout << "button clicked." << std::endl;
   
//   QProgressDialog dialog;
//   dialog.setMaximum(0);
//   dialog.setMinimum(0);
//   dialog.exec();

  unsigned int numFiles = 100;
  QProgressDialog progress("Copying files...", "Abort Copy", 0, numFiles, this);
  progress.setWindowModality(Qt::WindowModal);

  for (unsigned int fileId = 0; fileId < numFiles; fileId++)
  {
    std::cout << fileId << std::endl;
    progress.setValue(fileId);

    if (progress.wasCanceled())
    {
        break;
    }

    //sleep(1);
    usleep(100 * 1000);
  }
  progress.setValue(numFiles);
}
