#ifndef FORM_H
#define FORM_H

#include "ui_main.h"

#include <QElapsedTimer>

class MainWindow : public QMainWindow, public Ui::MainWindow
{
Q_OBJECT

public:
  MainWindow();

public slots:
  void btnStart_clicked();
  void btnStop_clicked();

private:
    QElapsedTimer timer;
};

#endif
