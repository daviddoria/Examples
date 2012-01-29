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
  
  this->connect(delegate, SIGNAL(UpdateSize(int,int,int,int)), SLOT(slot_UpdateSize(int,int,int,int)));
  //this->tableView->setItemDelegate(delegate);
  this->tableView->setItemDelegateForColumn(1, delegate);
  
  this->connect(this->model, SIGNAL(dataChanged ( const QModelIndex & , const QModelIndex & )), SLOT(TableChanged( const QModelIndex & , const QModelIndex & )));
  
}


void Form::TableChanged( const QModelIndex &topLeft , const QModelIndex &bottomRight )
{
  std::cout << "TableChanged()" << std::endl;
}

void Form::slot_UpdateSize(int row, int column, int width, int height)
{
  this->tableView->setColumnWidth(column, width);
  this->tableView->setRowHeight(row, height);
    
}
