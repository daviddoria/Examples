#include "MyWidget.h"

#include <iostream>

void MyWidget::myslot(MyClass* myClass)
{
  std::cout << "Slot called!" << std::endl;
  std::cout << myClass->a << std::endl;
}

void MyWidget::EmitSignal()
{
  MyClass* myClass = new MyClass;
  myClass->a = 3;
  emit mysignal(myClass);
}

MyWidget::MyWidget()
{
  connect( this, SIGNAL( mysignal(MyClass*) ), this, SLOT(myslot(MyClass*)) );
}
