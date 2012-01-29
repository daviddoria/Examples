#include <QAbstractTableModel>
#include <QtGui>

#include <iostream>
#include <sstream>

#include "form.h"
#include "ListModel.h"

Form::Form(QWidget *parent)
    : QWidget(parent)
{
  setupUi(this);
  
  ListModel* model = new ListModel;
  this->tableView->setModel(model);
  
  this->tableView->resizeColumnsToContents();
  
}
