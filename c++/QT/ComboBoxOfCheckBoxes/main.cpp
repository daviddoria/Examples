#include <QtGui>

#include <iostream>

#include "main.h"

MainWindow::MainWindow(QMainWindow* parent) : QMainWindow(parent)
{
  setupUi(this);
  this->Model = new QStandardItemModel;
  this->Item1 = new QStandardItem;

  this->Item1->setText("test");
  this->Item1->setFlags(Qt::ItemIsUserCheckable | Qt::ItemIsEnabled);
  this->Item1->setData(Qt::Unchecked, Qt::CheckStateRole);

  this->Item2 = new QStandardItem;
  this->Item2->setText("test2");
  this->Item2->setFlags(Qt::ItemIsUserCheckable | Qt::ItemIsEnabled);
  this->Item2->setData(Qt::Unchecked, Qt::CheckStateRole);

  connect(this->Model, SIGNAL(dataChanged ( const QModelIndex&, const QModelIndex&)), this, SLOT(slot_changed(const QModelIndex&, const QModelIndex&)));

  this->Model->insertRow(0, this->Item1);
  this->Model->insertRow(1, this->Item2);

  this->Items.push_back(this->Item1);
  this->Items.push_back(this->Item2);

  this->comboBox->setModel(this->Model);

  std::cout << comboBox->model()->rowCount() << " rows after." << std::endl;
}

void MainWindow::slot_changed(const QModelIndex& topLeft, const QModelIndex& bottomRight)
{
  //std::cout << "topLeft: " << topLeft.row() << std::endl;
  //std::cout << "bottomRight: " << bottomRight.row() << std::endl;
  std::cout << "Item " << topLeft.row() << " " << std::endl;
  QStandardItem* item = this->Items[topLeft.row()];
  if(item->checkState() == Qt::Unchecked)
    {
    std::cout << "Unchecked!" << std::endl;
    }
  else if(item->checkState() == Qt::Checked)
    {
    std::cout << "Checked!" << std::endl;
    }

}
