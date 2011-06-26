#ifndef FORM_H
#define FORM_H

#include "ui_form.h"

#include <boost/signal.hpp>
#include <boost/bind.hpp>

class MyForm : public QWidget, private Ui::Form
{
	Q_OBJECT
public:
    MyForm(QWidget *parent = 0);

//public slots:
    Q_SLOTS
    void pushButton_SetLabelText();
};

#endif
