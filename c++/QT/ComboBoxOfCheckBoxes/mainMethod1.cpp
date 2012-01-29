#include <QtGui>

#include <iostream>

#include "main.h"

Qt::ItemFlags CheckBoxModel::flags(const QModelIndex& index) const
{
  //return Qt::ItemIsUserCheckable;
  std::cout << "Set flags." << std::endl;
  return QStandardItemModel::flags(index) | Qt::ItemIsUserCheckable;
}

MainWindow::MainWindow(QMainWindow* parent) : QMainWindow(parent)
{
  setupUi(this);
  checkboxModel = new CheckBoxModel;
  QStandardItem* item = new QStandardItem;
  item->setText("test");
  item->setFlags(Qt::ItemIsUserCheckable | Qt::ItemIsEnabled);
  item->setData(Qt::Unchecked, Qt::CheckStateRole);
  checkboxModel->insertRow(0, item);
  //std::cout << comboBox->model()->rowCount() << " rows before." << std::endl;

  this->comboBox->setModel(checkboxModel);

  std::cout << comboBox->model()->rowCount() << " rows after." << std::endl;
}
