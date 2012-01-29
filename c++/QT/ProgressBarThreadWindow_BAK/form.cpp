#include "form.h"

#include <iostream>
#include <cmath>



void MainWindow::StartProgressSlot()
{
  std::cout << "StartProgressSlot()" << std::endl;
  //this->progressBar->show();
  this->PB.show();

}

void MainWindow::StopProgressSlot()
{
  std::cout << "StopProgressSlot()" << std::endl;
  this->PB.hide();
}

void ProgressThread::run()
{
  std::cout << "run()" << std::endl;
  emit StartProgressSignal();
  for(unsigned int i = 0; i < 1e8; i++)
    {
    float a = sin(i);
    }

  std::cout << "Done." << std::endl;
  this->exit();
  emit StopProgressSignal();
}

void ProgressThread::exit()
{
  std::cout << "exit()" << std::endl;
  emit StopProgressSignal();
}

MainWindow::MainWindow()
{
  this->setupUi(this);

  connect(&myProgressThread, SIGNAL(StartProgressSignal()), this, SLOT(StartProgressSlot()), Qt::QueuedConnection);
  connect(&myProgressThread, SIGNAL(StopProgressSignal()), this, SLOT(StopProgressSlot()), Qt::QueuedConnection);
}

void MainWindow::on_btnStart_clicked()
{
  std::cout << "Clicked start." << std::endl;
  this->myProgressThread.start(); // calls run()
}

void MainWindow::on_btnStop_clicked()
{
  std::cout << "Clicked stop." << std::endl;
  this->myProgressThread.exit(); // calls exit()
}
