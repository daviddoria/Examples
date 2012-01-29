#ifndef FORM_H
#define FORM_H

#include "ui_main.h"

class MainWindow : public QMainWindow, public Ui::MainWindow
{
Q_OBJECT
Q_PROPERTY( int number READ number WRITE setNumber )
public:
  MainWindow();

  void setNumber(const int num)
  {
    m_number = num;
  }
  
  int number()
  {
    return m_number;
  }
  
protected:
  int m_number;
};

#endif
