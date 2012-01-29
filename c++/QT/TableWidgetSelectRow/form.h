#ifndef FORM_H
#define FORM_H

#include "ui_form.h"

#include <QTableWidget>

class Form : public QWidget, public Ui::Form
{
    Q_OBJECT

public:
    Form(QWidget *parent = 0);
    void SetupTable(QTableWidget* table);

};

#endif
