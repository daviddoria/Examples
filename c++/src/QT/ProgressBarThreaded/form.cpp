#include "form.h"

#include <iostream>
#include <cmath>

void MainWindow::StartProgressSlot()
{
  this->progressBar->show();
}

void MainWindow::StopProgressSlot()
{
  this->progressBar->hide();
}

void ProgressThread::run()
{
  emit StartProgressSignal();
  for(unsigned int i = 0; i < 1e8; i++)
    {
    float a = sin(i);
    }

  std::cout << "Done." << std::endl;
  exit();
  emit StopProgressSignal();
}

void ProgressThread::exit()
{
  emit StopProgressSignal();
}

MainWindow::MainWindow()
{
  this->setupUi(this);
  this->progressBar->setMinimum(0);
  this->progressBar->setMaximum(0);
  this->progressBar->hide();

  connect(&myProgressThread, SIGNAL(StartProgressSignal()), this, SLOT(StartProgressSlot()), Qt::QueuedConnection);
  connect(&myProgressThread, SIGNAL(StopProgressSignal()), this, SLOT(StopProgressSlot()), Qt::QueuedConnection);
}

void MainWindow::on_btnStart_clicked()
{
  std::cout << "Clicked start." << std::endl;
  myProgressThread.start();
}
