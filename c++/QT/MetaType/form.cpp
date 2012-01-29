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

  // You cannot use a QueuedConnection if the type is not registered with Q_DECLARE_METATYPE (and for this it must have a default constructor)
  connect( &otherClass, SIGNAL( MySignal(const ClassWithNoDefaultConstructor&) ), this, SLOT(MySlot(const ClassWithNoDefaultConstructor&)), Qt::DirectConnection);
  //connect( &otherClass, SIGNAL( MySignal(const ClassWithNoDefaultConstructor&) ),
  //         this, SLOT(MySlot(const ClassWithNoDefaultConstructor&)), Qt::BlockingQueuedConnection);
  
}

void MyForm::on_btnButton_clicked()
{
  otherClass.EmitSignal();
}

void MyForm::MySlot(const MyType& myObject)
{
  std::cout << "MySlot() MyType" << std::endl;
}

void MyForm::MySlot(const ClassWithNoDefaultConstructor& myObject)
{
  std::cout << "MySlot() ClassWithNoDefaultConstructor" << std::endl;
}
