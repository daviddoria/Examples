#include <QAbstractTableModel>
#include <QtGui>

#include <iostream>
#include <sstream>

#include "form.h"




Form::Form(QWidget *parent)
    : QWidget(parent)
{
  setupUi(this);
  
  this->model = new MyTableModel;
  this->tableView->setModel(model);
  
  this->tableView->resizeColumnsToContents();
  
}

void Form::on_btnUpdate_clicked()
{
  this->model->ChangeData();
  //this->model->ChangeData();
}
