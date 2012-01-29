#include <QtGui>

#include "form.h"

#include <QTableWidgetItem>

void Form::CreateTable()
{
  unsigned int rows = 5;
  unsigned int cols = 2;
  
  this->tableWidget->setColumnCount(cols);
  this->tableWidget->setRowCount(rows);

  for(unsigned int row = 0; row < rows; ++row)
    {
    QTableWidgetItem* textItem = new QTableWidgetItem;
    textItem->setData(Qt::DisplayRole, row);
    this->tableWidget->setItem(row, 0, textItem);
    }

  for(unsigned int row = 0; row < rows; ++row)
    {
    QTableWidgetItem* textItem = new QTableWidgetItem;
    textItem->setData(Qt::DisplayRole, rows - row);
    this->tableWidget->setItem(row, 1, textItem);
    }
}

Form::Form(QWidget *parent)
    : QWidget(parent)
{
  setupUi(this);
  
  CreateTable();
}

void Form::on_pushButton_clicked()
{
  this->tableWidget->setRowCount(0);
  CreateTable();
}
