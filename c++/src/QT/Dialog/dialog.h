#ifndef DIALOG_H
#define DIALOG_H

#include "ui_dialog.h"

class Dialog : public QDialog, private Ui::Dialog
{
  Q_OBJECT
public:
  Dialog();

  
};

#endif
