#ifndef FORM_H
#define FORM_H

#include "ui_form.h"

class Form : public QWidget, public Ui::Form
{
    Q_OBJECT

public:
    Form(QWidget *parent = 0);

  public slots:
    void on_rad1_clicked();
    void on_rad2_clicked();
    void on_radA_clicked();
    void on_radB_clicked();
    
};

#endif
