#ifndef BUTTONFORM_H
#define BUTTONFORM_H

#include <QWidget>
#include "ui_form.h"

class Form : public QWidget, private Ui::Form
{
    Q_OBJECT
public:
    Form(QWidget *parent = 0);

public slots:
    void pushButton_clicked();
};

#endif
