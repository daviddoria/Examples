#include <QtGui>
#include <iostream>

#include "form.h"

Form::Form(QWidget *parent) : QWidget(parent)
{
  setupUi(this);
  connect( this->pushButton, SIGNAL( clicked() ), this, SLOT(pushButton_SetLabelText()) );
  connect( this->horizontalScrollBar, SIGNAL( valueChanged(int)), this, SLOT(pushButton_SetLabelText()) );
}

void Form::pushButton_SetLabelText()
{
  this->label->setText(QString::number(this->horizontalScrollBar->value()));
  std::cout << this->label->text().toDouble() << std::endl;
}
