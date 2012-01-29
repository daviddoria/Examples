#ifndef FORM_H
#define FORM_H

#include "ui_form.h"

class AbstractListModel;

class Form : public QWidget, public Ui::Form
{
    Q_OBJECT

public:
    Form(QWidget *parent = 0);

  public slots:
  void editSlot(QModelIndex index);

protected:
  AbstractListModel* Model;
};

#endif
