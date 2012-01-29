#ifndef FILEMENUFORM_H
#define FILEMENUFORM_H

#include "ui_filemenu.h"

class MyForm : public QMainWindow, private Ui::FileMenuForm
{
	Q_OBJECT
public:
    MyForm(QWidget *parent = 0);

public slots:
    void mnuOpenInnerForm_triggered();
};

#endif
