#include <QtGui>

#include "form.h"

Form::Form(QWidget *parent)
    : QWidget(parent)
{
  this->setupUi(this);
}

void Form::on_spinBox_valueChanged(int value)
{
  //this->label->setText(value);
  this->label->setNum(value);
}
