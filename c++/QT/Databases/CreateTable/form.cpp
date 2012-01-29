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

MyForm::MyForm(QWidget *parent) : QWidget(parent), Model(NULL)
{
  setupUi(this);
}

MyForm::~MyForm()
{
  if(this->Model)
    {
    delete this->Model;
    }
}

void MyForm::on_pushButton_clicked()
{
  std::cout << "Clicked." << std::endl;

  if(this->Model)
    {
    delete this->Model;
    }
    
  QString fileName = "test.sqlite";

  // Delete all connections
  for(unsigned int i = 0; i < QSqlDatabase::connectionNames().size(); ++i)
    {
    //QSqlDatabase::removeDatabase("TestConnection");
    QSqlDatabase::removeDatabase(QSqlDatabase::connectionNames()[i]);
    }
  
  //QSqlDatabase database = QSqlDatabase::addDatabase("QSQLITE");
  QSqlDatabase database = QSqlDatabase::addDatabase("QSQLITE", "TestConnection");
  database.setDatabaseName(fileName);
  if (database.open())
    {
    std::cout << "Opened." << std::endl;
    }
  else
    {
    std::cerr << "Could not open database" << std::endl;
    std::cerr << database.lastError().text().toStdString() << std::endl;
    }

  QSqlQuery createQuery(database);
  bool createSuccess = createQuery.exec("create table CompanyTable "
              "(id integer primary key, "
              "CompanyName TEXT)");

  QSqlQuery insertQuery(database);
  insertQuery.prepare("INSERT INTO CompanyTable (id, CompanyName) "
                "VALUES (:id, :CompanyName)");
  insertQuery.bindValue(":id", 0);
  insertQuery.bindValue(":CompanyName", "CompanyName");
  bool insertSuccess = insertQuery.exec();

  //this->Model = new QSqlTableModel;
  this->Model = new QSqlTableModel(NULL, database);
  this->Model->setTable("CompanyTable");
  this->Model->setEditStrategy(QSqlTableModel::OnManualSubmit);
  this->Model->select();

  this->tableView->setModel(this->Model);
}
