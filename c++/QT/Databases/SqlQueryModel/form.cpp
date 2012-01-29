#include "form.h"

#include <iostream>

#include <QSqlDatabase>
#include <QStringList>
#include <QSqlQuery>
#include <QDebug>
#include <QSqlError>
#include <QVariant>
#include <QSqlQueryModel>
#include <QSqlRecord>

MyForm::MyForm(QWidget *parent)
    : QWidget(parent)
{
  setupUi(this);

  QSqlDatabase db = QSqlDatabase::addDatabase("QSQLITE");
  db.setDatabaseName("MyDatabase.sqlite");

  QSqlQueryModel model;
  model.setQuery("select * from TestTable");

  this->tableView->setModel(&model);
}

