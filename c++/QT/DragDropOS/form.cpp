// Smarter handling explained here:
// http://www.informit.com/articles/article.aspx?p=1405546

#include "form.h"

#include <iostream>

#include <QDropEvent>

MyForm::MyForm(QWidget *parent) : QWidget(parent)
{
  setupUi(this);
  this->setAcceptDrops(true);
}

void MyForm::dropEvent ( QDropEvent * event )
{
  std::cout << "dropEvent." << std::endl;
  
  //QString filename = event->mimeData()->data("FileName");
  QString filename = event->mimeData()->text();
  std::cout << "Loaded " << filename.toStdString() << std::endl;
}

void MyForm::dragEnterEvent ( QDragEnterEvent * event ) 
{
  std::cout << "dragEnterEvent." << std::endl;
  
  event->accept();
}