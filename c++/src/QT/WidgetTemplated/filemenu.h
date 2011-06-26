#ifndef FILEMENUFORM_H
#define FILEMENUFORM_H

#include "ui_filemenu.h"

class MyForm : public QMainWindow, private Ui::FileMenuForm
{
	Q_OBJECT
public:
    MyForm(QWidget *parent = 0);

public slots:
    void mnuCreateIntForm_triggered();
    void mnuCreateDoubleForm_triggered();
};

#endif
