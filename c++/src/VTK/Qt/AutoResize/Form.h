#ifndef Form_H
#define Form_H

#include "ui_Form.h"

#include <QMainWindow>

class Form : public QMainWindow, private Ui::Form
{
  Q_OBJECT
public:

  Form();

public slots:

  virtual void slotExit();

};

#endif
