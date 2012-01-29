#include <QtGui>

#include "form.h"

#include "ClickableLabel.h"

#include <QTableWidgetItem>

Form::Form(QWidget *parent)
    : QWidget(parent)
{
  setupUi(this);
  
  this->tableWidget->setColumnCount(2);
  this->tableWidget->setRowCount(3);
  
  this->tableWidget->setSortingEnabled(false);
  
  //QTableWidgetItem* item=new QTableWidgetItem("A");
  // The above is the same as:
  
  QTableWidgetItem* textItem = new QTableWidgetItem;
  textItem->setText("A");
  
  this->tableWidget->setItem(0,0, textItem);
  
  // Add an image to the table
  QColor black(0,0,0);
  unsigned int imageSize = 100;
  QPixmap pixmap(imageSize,imageSize);
  pixmap.fill(black);
  
  // This works:
  ClickableLabel* imageLabel = new ClickableLabel;
  imageLabel->setPixmap(pixmap);
  imageLabel->Id = 22;
  //imageLabel->setScaledContents(true);
  imageLabel->setScaledContents(false);
  this->tableWidget->setCellWidget(1,0,imageLabel);
  this->tableWidget->resizeRowsToContents();
  this->tableWidget->resizeColumnsToContents();

}
