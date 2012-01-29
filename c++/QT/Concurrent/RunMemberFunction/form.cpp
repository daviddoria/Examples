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
  MyClass test;
  QFuture<void> result = QtConcurrent::run (&test, &MyClass::start);
  //result.waitForFinished();
  
  std::cout << "exit." << std::endl;
}
