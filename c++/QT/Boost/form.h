#ifndef FORM_H
#define FORM_H

#include "ui_form.h"

class MyForm : public QWidget, private Ui::Form
{
	Q_OBJECT
public:
    MyForm(QWidget *parent = 0);

public Q_SLOTS:
    void pushButton_SetLabelText();
};

#endif
