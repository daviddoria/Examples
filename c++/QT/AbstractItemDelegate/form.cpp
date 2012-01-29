#include <QAbstractTableModel>
#include <QtGui>

#include <iostream>
#include <sstream>

#include "form.h"

#include "LabelDelegate.h"

Form::Form(QWidget *parent)
    : QWidget(parent)
{
  setupUi(this);
  
  this->model = new MyTableModel;
  this->tableView->setModel(model);

  LabelDelegate* delegate = new LabelDelegate;
  
  //this->tableView->setItemDelegate(delegate);
  this->tableView->setItemDelegateForColumn(1, delegate);
  
  
  this->tableView->resizeColumnsToContents();
  this->tableView->resizeRowsToContents();
  
  this->connect(this->model, SIGNAL(dataChanged ( const QModelIndex & , const QModelIndex & )), SLOT(TableChanged( const QModelIndex & , const QModelIndex & )));
  
}


void Form::TableChanged( const QModelIndex &topLeft , const QModelIndex &bottomRight )
{
  std::cout << "TableChanged()" << std::endl;
}
