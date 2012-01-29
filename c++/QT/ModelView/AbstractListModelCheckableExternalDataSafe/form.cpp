#include <QAbstractTableModel>
#include <QtGui>

#include <iostream>
#include <sstream>

#include "form.h"
#include "AbstractListModelCheckableExternalDataSafe.h"

Form::Form(QWidget *parent)
    : QWidget(parent)
{
  setupUi(this);

  AbstractListModelCheckable* model = new AbstractListModelCheckable;

  this->Items.push_back(Item("Test0", Qt::Unchecked));
  this->Items.push_back(Item("Test1", Qt::Checked));
  this->Items.push_back(Item("Test2", Qt::Unchecked));
  model->setItems(&this->Items);
  this->tableView->setModel(model);
}
