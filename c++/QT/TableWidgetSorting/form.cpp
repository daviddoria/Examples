#include <QtGui>

#include "form.h"

#include <QTableWidgetItem>

Form::Form(QWidget *parent)
    : QWidget(parent)
{
  setupUi(this);
  
  // You must have the "sortingEnabled" checkbox checked on the widget in QtDesigner.
  
  this->tableWidget->setColumnCount(1);
  this->tableWidget->setRowCount(3);
  
  QTableWidgetItem* textItem0 = new QTableWidgetItem;
  //textItem0->setText("0"); // If you use this method to set the values, the sorting will sort by string sorting (alphabetical) instead of numerical comparisons. E.g. 5 50, 1, 10 insetad of 1 10 5 50
  textItem0->setData(Qt::DisplayRole, 0);
  this->tableWidget->setItem(0, 0, textItem0);
  
  QTableWidgetItem* textItem1 = new QTableWidgetItem;
  textItem1->setData(Qt::DisplayRole, 20);
  this->tableWidget->setItem(1, 0, textItem1);
  
  QTableWidgetItem* textItem2 = new QTableWidgetItem;
  textItem2->setData(Qt::DisplayRole, 5);
  this->tableWidget->setItem(2, 0, textItem2);
}
