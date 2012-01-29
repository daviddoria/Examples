#include <QAbstractTableModel>
#include <QtGui>

#include <iostream>
#include <sstream>

#include "form.h"
#include "MyTableModel.h"

Form::Form(QWidget *parent)
    : QWidget(parent)
{
  setupUi(this);
  
  MyTableModel* model = new MyTableModel;
  this->tableView->setModel(model);
  
  this->tableView->resizeColumnsToContents();
  
  this->connect(this->tableView->selectionModel(), SIGNAL(currentChanged (const QModelIndex & , const QModelIndex & )), SLOT(SomethingChanged(const QModelIndex & , const QModelIndex & )));
}

void Form::SomethingChanged(const QModelIndex & current, const QModelIndex & previous)
{
  std::cout << "Current row: " << current.row() << std::endl;
}
