#ifndef FORM_H
#define FORM_H

#include "ui_main.h"

class MainWindow : public QMainWindow, public Ui::MainWindow
{
Q_OBJECT

public:
  MainWindow();

public slots:
  void btnSetText_clicked();

};

#endif
