#include <iostream>

#include <QSqlDatabase>
#include <QStringList>
#include <QSqlQuery>
#include <QDebug>
#include <QSqlError>
#include <QVariant>

int main(int, char *[])
{
  QSqlDatabase db = QSqlDatabase::addDatabase("QSQLITE");
  db.setDatabaseName("MyDatabase.sqlite");

  if (db.open())
  {
    // Try to locate the contacts database.
    // If it is not available create it.
    if (db.tables().indexOf("people") == -1)
    {
      QSqlQuery query(db);
      bool success = query.exec("create table people "
                  "(id integer primary key, "
                  "name TEXT, "
                  "age integer)");
      if (!success)
        {
        qCritical() << query.lastError();
        }
      else
        {
        std::cout << "Created table" << std::endl;
        }
    }
    else
    {
      std::cout << "Table already exists" << std::endl;
      // NULL = is the keyword for the autoincrement to generate next value

        QSqlQuery insertQuery;
        bool success = insertQuery.exec(QString("insert into person values(NULL,'%1','%2')")
        .arg("David").arg(25));
        std::cout << "Successfull insertion? " << success << std::endl;

        QSqlQuery searchQuery(QString("select * from person where id = %1").arg(1));
        if (searchQuery.next())
          {
          std::cout << "id: " << searchQuery.value(0).toInt() << std::endl;
          std::cout << "Name: " << searchQuery.value(1).toString().toStdString() << std::endl;
          std::cout << "age: " << searchQuery.value(2).toInt() << std::endl;
          }
    }
  }
  else
  {
    std::cout << "Could not open database" << std::endl;
    qCritical() << db.lastError();
  }

  return 0;
}
