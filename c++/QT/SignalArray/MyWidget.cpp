#include "MyWidget.h"
#include "SmallWidget.h"

#include <iostream>


MyWidget::MyWidget()
{
  SmallWidget* smallWidget = new SmallWidget;
  connect( smallWidget, SIGNAL( mysignal() ), this, SLOT(myslot()) );
  smallWidget->EmitSignal();
}

void MyWidget::myslot()
{
  std::cout << "Slot called!" << std::endl;
}
