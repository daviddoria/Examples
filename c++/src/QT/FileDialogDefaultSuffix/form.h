#ifndef FORM_H
#define FORM_H

#include <QWidget>

#include "ui_form.h"

class Form : public QWidget, private Ui::Form
{
    Q_OBJECT

public:
    Form(QWidget *parent = 0);

private slots:
    void on_btnSave_clicked();

};

#endif
