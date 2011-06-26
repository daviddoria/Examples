#include <QtGui>

#include "form.h"

Form::Form(QWidget *parent)
    : QWidget(parent)
{
  setupUi(this);
  connect( this->pushButton, SIGNAL( clicked() ), this, SLOT(pushButton_SetLabelText()) );
  
  // Limit input to valid values
  QIntValidator* validator =
  new QIntValidator(0, 255, this->lineEdit);
  this->lineEdit->setValidator(validator);
  
}

void Form::pushButton_SetLabelText()
{
  this->label->setText(this->lineEdit->text());
  
}
