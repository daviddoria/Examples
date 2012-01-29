#include <QtGui>

#include <iostream>

#include "form.h"

Form::Form(QWidget *parent)
    : QMainWindow(parent)
{
  setupUi(this);

  this->Model = new QStringListModel;
  this->listView->setModel(this->Model);
  connect(this->Model, SIGNAL(dataChanged(QModelIndex,QModelIndex)), this, SLOT(slot_modelChanged(QModelIndex,QModelIndex)));
  
}

void Form::on_btnAdd_clicked()
{
  this->Model->insertRows(this->Model->rowCount(), 1);
  this->Model->setData(this->Model->index(this->Model->rowCount()-1), "new line " + QString::number(this->Model->rowCount()));
}

void Form::slot_modelChanged(const QModelIndex &topLeft, const QModelIndex  &bottomRight)
{
  std::cout << "Model changed." << std::endl;
}
