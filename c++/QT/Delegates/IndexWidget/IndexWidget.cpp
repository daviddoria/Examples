#include <QAbstractTableModel>
#include <QtGui>

#include <iostream>
#include <sstream>

#include "IndexWidget.h"

Form::Form(QWidget *parent)
    : QWidget(parent)
{
  setupUi(this);

  QStringListModel* model = new QStringListModel;
  this->listView->setModel(model);
  this->listView->setIndexWidget(QModelIndex(0, 0), new QComboBox);
}
