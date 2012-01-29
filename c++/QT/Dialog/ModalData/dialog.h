#ifndef DIALOG_H
#define DIALOG_H

#include "ui_dialog.h"

class Dialog : public QDialog, private Ui::Dialog
{
  Q_OBJECT
public:
  Dialog();

  std::string GetUserText();
  
public slots:
  void on_buttonBox_accepted();
  void on_buttonBox_rejected();

protected:
  std::string userText;
};

#endif
