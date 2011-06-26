#ifndef BUTTONFORM_H
#define BUTTONFORM_H

#include "ui_main.h"

#include <QThread>

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

public slots:

  void on_btnStart_clicked();
  void StartProgressSlot();
  void StopProgressSlot();

};

#endif
