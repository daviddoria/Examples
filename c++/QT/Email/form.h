#ifndef FORM_H
#define FORM_H

#include "ui_main.h"

#include <QTime>

class MainWindow : public QMainWindow, public Ui::MainWindow
{
Q_OBJECT

public:
  MainWindow();

};

#endif
