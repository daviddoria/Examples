#ifndef FORM_H
#define FORM_H

#include "ui_form.h"

#include "MyTableModel.h"

class Form : public QWidget, public Ui::Form
{
    Q_OBJECT

public:
    Form(QWidget *parent = 0);

public slots:
  void TableChanged( const QModelIndex & , const QModelIndex & );
protected:
  MyTableModel* model;
};

#endif
