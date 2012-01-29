#include "form.h"

#include <iostream>

#include <QSqlDatabase>
#include <QStringList>
#include <QSqlQuery>
#include <QDebug>
#include <QSqlError>
#include <QVariant>
#include <QSqlTableModel>
#include <QSqlRecord>

MyForm::MyForm(QWidget *parent)
    : QWidget(parent)
{
  setupUi(this);
  connect( this->pushButton, SIGNAL( clicked() ), this, SLOT(pushButton_clicked()) );
}

void MyForm::pushButton_clicked()
{
  QSqlDatabase db = QSqlDatabase::addDatabase("QSQLITE");
  db.setDatabaseName("MyDatabase.sqlite");

  db.open();

  QSqlQuery createQuery;
  bool createSuccess = createQuery.exec("create table people "
              "(id integer primary key, "
              "name TEXT)");
  QSqlQuery insertQuery(db);
  insertQuery.prepare("insert into people values(NULL,:id,:name)");
  insertQuery.bindValue(":id", 0);
  insertQuery.bindValue(":name", "david");
  bool insertSuccess = insertQuery.exec();

  QSqlTableModel *model = new QSqlTableModel;
  model->setTable("people");
  model->setEditStrategy(QSqlTableModel::OnManualSubmit);
  model->select();

  this->tableView->setModel(model);

  this->tableView->setItemDelegateForColumn(1, new DateEditDelegate());
  
}
