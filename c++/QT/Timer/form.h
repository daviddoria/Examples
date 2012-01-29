#ifndef BUTTONFORM_H
#define BUTTONFORM_H

#include "ui_main.h"

//class QTimer;
#include <QTimer>
//class QProgressDialog;
#include <QProgressDialog>

//class MainWindow : public QWidget
class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
  MainWindow();
  
  public slots:
  void on_btnStart_clicked();  
  void TimerEvent();

private:
    Ui::MainWindow ui;
    //QTimer* timer = new QTimer(this);
    QTimer timer;
    QProgressDialog dialog;
};

#endif
