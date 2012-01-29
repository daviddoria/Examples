#ifndef FORM_H
#define FORM_H

#include "ui_form.h"

#include "MyTableModel.h"

class Form : public QWidget, public Ui::Form
{
Q_OBJECT

public:
  Form(QWidget *parent = 0);

protected:
  MyTableModel* model;
};

#endif
