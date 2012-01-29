#include <QAbstractTableModel>
#include <QtGui>

#include <iostream>
#include <sstream>

#include "form.h"
#include "AbstractListModelCheckable.h"

Form::Form(QWidget *parent)
    : QWidget(parent)
{
  setupUi(this);

  AbstractListModelCheckable* model = new AbstractListModelCheckable;

  QVector<Item> items;
  items.push_back(Item("Test0", Qt::Unchecked));
  items.push_back(Item("Test1", Qt::Checked));
  items.push_back(Item("Test2", Qt::Unchecked));
  model->setItems(items);
  this->tableView->setModel(model);
}
