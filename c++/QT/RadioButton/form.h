#ifndef FORM_H
#define FORM_H

#include "ui_form.h"

class Form : public QWidget
{
    Q_OBJECT

public:
    Form(QWidget *parent = 0);

  public slots:
    void pushButton_SetLabelText();
    void radioButton_SetLabelText();
    void radioButton_2_SetLabelText();
    
private:
    Ui::Form ui;
};

#endif
