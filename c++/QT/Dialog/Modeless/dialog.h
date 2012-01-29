#ifndef DIALOG_H
#define DIALOG_H

#include "ui_dialog.h"

class Dialog : public QDialog, private Ui::Dialog
{
  Q_OBJECT
public:
  Dialog();

public slots:
  void on_buttonBox_accepted();
  void on_buttonBox_rejected();
  
};

#endif
