#ifndef FORM_H
#define FORM_H

#include "ui_form.h"

#include "dialog.h"

class Form : public QWidget, private Ui::Form
{
  Q_OBJECT


public:
  Form(QWidget *parent = 0);
  void ShowModeless();
  
public slots:
  void on_btnShowModeless_clicked();
  void ModelessDialog_accepted();
  void ModelessDialog_rejected();

protected:
  Dialog* ModelessDialog;
};

#endif
