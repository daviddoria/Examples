#ifndef BUTTONFORM_H
#define BUTTONFORM_H

#include "ui_main.h"

//class MainWindow : public QWidget
class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
  MainWindow();
  
  public slots:
  void on_btnStart_clicked();  

private:
    Ui::MainWindow ui;
};

#endif
