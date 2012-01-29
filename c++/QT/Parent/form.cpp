#include <QtGui>
#include <QImage>

#include "form.h"
#include "MyClass.h"

#include <iostream>

Form::Form(QWidget *parent)
    : QWidget(parent)
{
  setupUi(this);
}

void Form::on_btnOpen_clicked()
{
  QThread* thread = new QThread;

  MyClass* myClass = new MyClass;
  myClass->moveToThread(thread);
  connect(thread, SIGNAL(started()), myClass, SLOT(start()));
  connect(myClass, SIGNAL(finished()), thread, SLOT(quit()));
  thread->start();
  //a->wait();
  
  std::cout << "exit." << std::endl;
}
