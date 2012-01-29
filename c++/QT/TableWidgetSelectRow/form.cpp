#include <iostream>

#include "form.h"

#include <QtGui>
#include <QStyleFactory>
#include <QTableWidgetItem>

Form::Form(QWidget *parent)
    : QWidget(parent)
{
  setupUi(this);

  SetupTable(this->tableWidget);
  SetupTable(this->tableWidget_2);

  /*
  //QPalette palette = this->tableWidget->viewport()->palette();
  QPalette palette = this->tableWidget->palette();
  
  //palette.setColor(QPalette::Inactive, QPalette::Window, palette.color(QPalette::Active, QPalette::Window));
  palette.setColor( QPalette::Inactive, QPalette::Highlight, palette.color(QPalette::Active, QPalette::Highlight) );
  palette.setColor( QPalette::Inactive, QPalette::HighlightedText, palette.color(QPalette::Active, QPalette::HighlightedText) );
  
  QApplication::setPalette(palette);
  */
  //this->tableWidget->setStyle("cleanlooks");
  std::cout << QStyleFactory::create("cleanlooks") << std::endl;
  this->tableWidget->setStyle(QStyleFactory::create("cleanlooks"));


  
  //this->tableWidget->viewport()->setPalette( palette );
  //this->tableWidget->setPalette( palette );
  //this->tableWidget_2->setPalette( palette );
}

void Form::SetupTable(QTableWidget* table)
{
  table->setColumnCount(1);
  table->setRowCount(3);
  
  QTableWidgetItem* textItem0 = new QTableWidgetItem;
  textItem0->setText("A");
  table->setItem(0,0, textItem0);
  
  QTableWidgetItem* textItem1 = new QTableWidgetItem;
  textItem1->setText("B");
  table->setItem(1,0, textItem1);
  
  QTableWidgetItem* textItem2 = new QTableWidgetItem;
  textItem2->setText("C");
  table->setItem(2,0, textItem2);
  
  table->selectRow(1);
}
