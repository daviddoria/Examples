#include <QtGui>
#include <QImage>

#include "form.h"
#include "MyClass.h"

#include <iostream>

Form::Form(QWidget *parent)
    : QWidget(parent)
{
  setupUi(this);

  this->ProgressDialog = new QProgressDialog();

  connect(&this->FutureWatcher, SIGNAL(finished()), this, SLOT(slot_finished()));
  connect(&this->FutureWatcher, SIGNAL(finished()), this->ProgressDialog , SLOT(cancel())); // Without this line, the progress bar doesn't disappear when it is done.

}

void Form::slot_finished()
{
  std::cout << "Finshed" << std::endl;
}

void Form::on_pushButton_clicked()
{
  // Start the computation.
  //QFuture<void> future = QtConcurrent::run(&this->MyObject, &MyClass::LongFunction); // This only works for superclass
  //QFuture<void> future = QtConcurrent::run(&this->MyObject, static_cast<void(ChildClass::*)()>(&ChildClass::LongFunction));// works
  
  // This also works
  ChildClass testObject;
  QFuture<void> future = QtConcurrent::run(&testObject, static_cast<void(ChildClass::*)()>(&ChildClass::LongFunction));
  
  this->FutureWatcher.setFuture(future);

  this->ProgressDialog->setMinimum(0);
  this->ProgressDialog->setMaximum(0);
  this->ProgressDialog->setWindowModality(Qt::WindowModal);
  this->ProgressDialog->exec();

}
