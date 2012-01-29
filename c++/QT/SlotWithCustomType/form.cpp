// This example does not actually work, it causes a deadlock. But if this signal were emitted from another thread as it would be
// in a real application, this example demonstrates how to use Q_DECLARE_METATYPE and qRegisterMetaType to allow it to work.
#include "form.h"

#include <iostream>
#include <string>

Q_DECLARE_METATYPE(MyType)

MyForm::MyForm(QWidget *parent) : QWidget(parent)
{
  setupUi(this);
  //connect( this, SIGNAL( MySignal(const MyType&) ), this, SLOT(MySlot(const MyType&)) );
  qRegisterMetaType<MyType>("MyType");
  connect( &otherClass, SIGNAL( MySignal(const MyType&) ), this, SLOT(MySlot(const MyType&)), Qt::BlockingQueuedConnection);
  
}

void MyForm::on_btnButton_clicked()
{
  otherClass.EmitSignal();
}

void MyForm::MySlot(const MyType& myObject)
{
  std::cout << "MySlot()" << std::endl;
}
