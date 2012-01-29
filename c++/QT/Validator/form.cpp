#include <QtGui>

#include <iostream>

#include "form.h"

Form::Form(QWidget *parent)
    : QWidget(parent)
{
  setupUi(this);

  // Limit input to valid values between 0 and 255. Either parent ('this' or 'lineEdit') works, I'm not sure what difference it makes.
  QIntValidator* validator =
  //new QIntValidator(0, 255, this->lineEdit);
  new QIntValidator(0, 255, this);
  this->lineEdit1->setValidator(validator);
  this->lineEdit2->setValidator(validator);
  
}

void Form::on_lineEdit1_textEdited ( const QString & text )
{
  std::cout << "text1: " << text.toStdString() << std::endl;
  std::cout << "number1: " << text.toUInt() << std::endl;
}

void Form::on_lineEdit2_textEdited ( const QString & text )
{
  std::cout << "text2: " << text.toStdString() << std::endl;
}
