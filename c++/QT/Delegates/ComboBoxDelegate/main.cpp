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
  
  tableView.setItemDelegateForColumn(1, &delegate); // Column 0 can take any value, column 1 can only take values up to 8.

  for (int row = 0; row < 4; ++row)
    {
    for (int column = 0; column < 2; ++column)
      {
      QModelIndex index = model.index(row, column, QModelIndex());
      int value = (row+1) * (column+1);
      std::cout << "Setting (" << row << ", " << column << ") to " << value << std::endl;
      model.setData(index, QVariant(value));
      }
    }

  // Make the combo boxes always displayed.
  for ( int i = 0; i < model.rowCount(); ++i )
    {
    tableView.openPersistentEditor( model.index(i, 1) );
    }

  tableView.show();
  return app.exec();
}
