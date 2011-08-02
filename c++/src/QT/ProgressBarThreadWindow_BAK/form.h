#ifndef BUTTONFORM_H
#define BUTTONFORM_H

#include "ui_main.h"

#include "ui_ProgressBar.h"

#include <QThread>

class ProgressBarForm : public QWidget, private Ui::ProgressBarForm
{
public:
  
  ProgressBarForm()
  {
    this->setupUi(this);
    this->progressBar->setMinimum(0);
    this->progressBar->setMaximum(0);
    //this->progressBar->hide(); 
  }
};

class ProgressThread : public QThread
{
Q_OBJECT
public:
  void run();
  void exit();

signals:
  void StartProgressSignal();
  void StopProgressSignal();
};

class MainWindow : public QMainWindow, private Ui::MainWindow
{
Q_OBJECT

public:
  MainWindow();

  ProgressThread myProgressThread;

  ProgressBarForm PB;
public slots:

  void on_btnStart_clicked();
  void on_btnStop_clicked();
  void StartProgressSlot();
  void StopProgressSlot();


};

#endif
