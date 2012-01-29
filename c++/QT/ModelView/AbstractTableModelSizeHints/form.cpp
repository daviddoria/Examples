/*
 * Note: this doesn't seem to work at all.
 */

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
  
}
