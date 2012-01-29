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
  // NOTE: DO NOT include the name of the paramter when using signals/slots with parameters.
  // It will compile, but give you "undefined signal" errors at runtime.
  // I.e. use
  // connect( this, SIGNAL( mysignal(QVector<double>) ), this, SLOT(myslot(QVector<double>)) );
  // instead of
  // connect( this, SIGNAL( mysignal(QVector<double> coord) ), this, SLOT(myslot(QVector<double> coord)) );
}
