#include "MyWidget.h"

#include <iostream>

void MyWidget::myslot()
{
  std::cout << "Slot called!" << std::endl;
}

void MyWidget::EmitSignal()
{
  emit mysignal();
}

MyWidget::MyWidget()
{
  connect( this, SIGNAL( mysignal() ), this, SLOT(myslot()) );
}