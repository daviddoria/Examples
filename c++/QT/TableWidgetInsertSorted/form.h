#ifndef FORM_H
#define FORM_H

#include "ui_form.h"

class Form : public QWidget, public Ui::Form
{
    Q_OBJECT

public:
    Form(QWidget *parent = 0);

  public slots:
    //void on_tableWidgetInsertSorted();
    void on_pushButton_clicked();

protected:
  void CreateTable();
};

#endif
