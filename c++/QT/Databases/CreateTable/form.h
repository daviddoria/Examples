#ifndef BUTTONFORM_H
#define BUTTONFORM_H

#include "ui_CreateTable.h"

class QSqlTableModel;

class MyForm : public QWidget, private Ui::Form
{
Q_OBJECT
public:
    MyForm(QWidget *parent = 0);
    ~MyForm();

public slots:
    void on_pushButton_clicked();

protected:
  QSqlTableModel* Model;
};

#endif
