#ifndef FORM_H
#define FORM_H

#include "ui_form.h"

class Form : public QWidget, public Ui::Form
{
    Q_OBJECT

public:
    Form(QWidget *parent = 0);

  public slots:
    void on_tableWidget_cellClicked( int row, int column );

};

#endif
