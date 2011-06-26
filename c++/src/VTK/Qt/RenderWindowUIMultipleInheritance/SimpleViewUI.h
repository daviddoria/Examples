#ifndef SimpleView_H
#define SimpleView_H

#include "ui_SimpleViewUI.h"

#include <QMainWindow>

class SimpleView : public QMainWindow, private Ui::SimpleView
{
  Q_OBJECT
public:

  SimpleView();

public slots:

  virtual void slotExit();

};

#endif
