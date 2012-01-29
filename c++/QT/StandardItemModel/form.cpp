#include <QtGui>

#include <iostream>

#include "form.h"

Form::Form(QWidget *parent)
    : QWidget(parent)
{
  setupUi(this);

  QStandardItemModel* model = new QStandardItemModel;

  QStandardItem* item0 = new QStandardItem ("test");
  QList<QStandardItem*> items;
  items.insert(0, item0);
  //model->appendRow (items);
  //model->insertRow (0, items);
  model->setItem(0, 0, item0);
  this->tableView->setModel(model);
}
