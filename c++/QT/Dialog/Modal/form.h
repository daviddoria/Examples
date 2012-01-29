#ifndef FORM_H
#define FORM_H

#include "ui_form.h"

#include "dialog.h"

class Form : public QWidget, private Ui::Form
{
  Q_OBJECT

public:
  Form(QWidget *parent = 0);
void ShowModal();
  
public slots:
  void on_btnShowModal_clicked();
  
};

#endif
