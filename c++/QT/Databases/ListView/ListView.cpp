#include <QApplication>
#include <QSqlDatabase>
#include <QSqlError>
#include <QSqlQuery>
#include <QVariant>
#include <QFile>

#include <iostream>

#include "form.h"

int main(int argc, char *argv[])
{
  QApplication app(argc, argv);

  QFile::remove ("test.sqlite");
  
  QSqlDatabase database = QSqlDatabase::addDatabase("QSQLITE");
  database.setDatabaseName("test.sqlite");
  if(!database.open())
    {
    std::cerr << "Could not open database" << std::endl;
    std::cerr << "Last error: " << database.lastError().text().toStdString() << std::endl;
    }

  QSqlQuery createQuery;
  bool createSuccess = createQuery.exec("create table AssociateTable "
              "(id integer primary key, "
              "AssociateName TEXT, "
              "SocialSecurityNumber TEXT)");

  if(createSuccess)
    {
    std::cout << "Table created successfully!" << std::endl;
    }
  else
    {
    std::cerr << "Could not create table." << std::endl;
    std::cerr << "Last database error: " << database.lastError().text().toStdString() << std::endl;
    std::cerr << "Last query error: " << createQuery.lastError().text().toStdString() << std::endl;
    }


  // Populate the table with default values
  QSqlQuery insertQuery;
  insertQuery.prepare("INSERT INTO AssociateTable (id, AssociateName, SocialSecurityNumber) "
                "VALUES (:id, :AssociateName, :SocialSecurityNumber)");
  insertQuery.bindValue(":id", 0);
  insertQuery.bindValue(":AssociateName", "AssociateName");
  insertQuery.bindValue(":SocialSecurityNumber", "SocialSecurityNumber");

  bool insertSuccess = insertQuery.exec();
  if(!insertSuccess)
    {
    std::cerr << "Could not insert values!" << std::endl;
    std::cerr << "insertQuery last error: " << insertQuery.lastError().text().toStdString() << std::endl;
    return -1;
    }
    
  MyForm form;

  form.show();
  return app.exec();
}
