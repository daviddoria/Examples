#ifndef BUTTONFORM_H
#define BUTTONFORM_H

#include "ui_buttonform.h"

// This example currently does nothing. See 
// http://doc.qt.nokia.com/stable/qsignalmapper.html#details
// for details

class MyForm : public QWidget, private Ui::ButtonForm
{
	Q_OBJECT
public:
    MyForm(QWidget *parent = 0);

public slots:
    void pushButton_SetLabelText();
};

#endif
