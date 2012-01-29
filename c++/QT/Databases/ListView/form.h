#ifndef ListView_H
#define ListView_H

#include "ui_ListView.h"

class QSqlTableModel;

class MyForm : public QWidget, private Ui::Form
{
Q_OBJECT
public:
  MyForm(QWidget *parent = 0);
  ~MyForm();

protected:
  void showEvent ( QShowEvent * event );

protected:
  QSqlTableModel* TableModel;
};

#endif
