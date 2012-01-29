#include <QtGui>
#include <QtSql>

#include "connection.h"

int main(int argc, char *argv[])
{
  QApplication app(argc, argv);
  if (!createConnection())
      return 1;

  QSqlTableModel model;

  model.setTable("person");
  model.setEditStrategy(QSqlTableModel::OnManualSubmit);
  model.select();

//   model.setHeaderData(0, Qt::Horizontal, QObject::tr("ID"));
//   model.setHeaderData(1, Qt::Horizontal, QObject::tr("First name"));
//   model.setHeaderData(2, Qt::Horizontal, QObject::tr("Last name"));

  QTableView *view = new QTableView;
  view->setModel(&model);
  view->setWindowTitle("Table Model View");

  view->show();

  return app.exec();
}
