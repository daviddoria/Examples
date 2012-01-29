#include <QApplication>
#include <QHeaderView>
#include <QItemSelectionModel>
#include <QStandardItemModel>
#include <QTableView>

#include <iostream>

#include "IndexWidget.h"

int main(int argc, char *argv[])
{
  std::cout << "Enter." << std::endl;
  QApplication app(argc, argv);
  Form form;

  form.show();

  return app.exec();
}
