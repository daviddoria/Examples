#include "MyWidget.h"

#include <iostream>

void MyWidget::myslot(const std::string& mystring)
{
  std::cout << mystring << std::endl;
}

void MyWidget::EmitSignal()
{
  emit mysignal("test");
}

MyWidget::MyWidget()
{
  connect( this, SIGNAL( mysignal(const std::string&) ), this, SLOT(myslot(const std::string&)) );
}
