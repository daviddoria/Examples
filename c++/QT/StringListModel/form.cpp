#include <QStringListModel>
#include <QtGui>

#include <iostream>
#include <sstream>

#include "form.h"

Form::Form(QWidget *parent)
    : QWidget(parent)
{
  setupUi(this);
  
  QStringListModel* model = new QStringListModel;
  QStringList list;
  list << "a" << "b" << "c";
  model->setStringList(list);
  this->listView->setModel(model);
  
}
