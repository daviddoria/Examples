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
  QFuture<void> result = QtConcurrent::run ( LongFunction );
  //result.waitForFinished();
  
  std::cout << "exit." << std::endl;
}
