#include <QtGui>
#include <QImage>

#include "form.h"

#include <iostream>

Form::Form(QWidget *parent)
    : QWidget(parent)
{
  setupUi(this);
  
  this->progressBar->setMinimum(0);
  this->progressBar->setMaximum(0);
  this->progressBar->hide();


  connect(&this->FutureWatcher, SIGNAL(finished()), this, SLOT(slot_finished()));

}

void Form::slot_finished()
{
  this->progressBar->hide();
}

void Form::on_pushButton_clicked()
{
  this->progressBar->show();

  // Start the computation.
  QFuture<void> future = QtConcurrent::run(&this->MyObject, &MyClass::LongFunction);
  this->FutureWatcher.setFuture(future);
}
