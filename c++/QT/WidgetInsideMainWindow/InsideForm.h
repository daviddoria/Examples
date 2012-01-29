#ifndef INSIDEFORM_H
#define INSIDEFORM_H

#include "ui_InsideForm.h"

class MyInsideForm : public QWidget, private Ui::InsideForm
{
	Q_OBJECT
public:
    MyInsideForm(QWidget *parent = 0);

public slots:
    void btnButton_clicked();
};

#endif
