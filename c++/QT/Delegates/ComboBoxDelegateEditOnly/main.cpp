/*
  main.cpp

  A simple example that shows how a view can use a custom delegate to edit
  data obtained from a model.
*/

#include <QApplication>
#include <QHeaderView>
#include <QItemSelectionModel>
#include <QStandardItemModel>
#include <QTableView>

#include <iostream>

#include "ComboBoxDelegate.h"


int main(int argc, char *argv[])
{
  std::cout << "Enter." << std::endl;
  QApplication app(argc, argv);

  QStandardItemModel model(4, 2);
  QTableView tableView;
  tableView.setModel(&model);

  ComboBoxDelegate delegate;
  //tableView.setItemDelegate(&delegate);
  tableView.setItemDelegateForColumn(1, &delegate); // Column 0 can take any value, column 1 can only take values up to 8.

  for (int row = 0; row < 4; ++row)
    {
    for (int column = 0; column < 2; ++column)
      {
      QModelIndex index = model.index(row, column, QModelIndex());
      model.setData(index, QVariant((row+1) * (column+1)));
      }
    }

  tableView.show();
  return app.exec();
}

