#include <QAbstractListModel>
#include <QtGui>

#include <iostream>

#include "form.h"
#include "AbstractListModelFileDialog.h"

Form::Form(QWidget *parent)
    : QWidget(parent)
{
  setupUi(this);

  this->Model = new AbstractListModel;

  connect (this->listView, SIGNAL(clicked(QModelIndex)), this, SLOT(editSlot(QModelIndex)));

  QVector<Item> items;
  items.push_back(Item("Test0"));
  items.push_back(Item("Test1"));
  items.push_back(Item("Test2"));
  this->Model->setItems(items);
  this->listView->setModel(this->Model);
}

void Form::editSlot(QModelIndex index)
{
  if (index.row() == 0)
    {
    QString fileName = QFileDialog::getOpenFileName(this, "Open", "", "Files (*.*)");
    this->Model->setData(index, fileName, Qt::EditRole);
    }
}
