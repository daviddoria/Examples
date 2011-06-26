#include <QtGui>
#include <iostream>

#include "form.h"

Form::Form(QWidget *parent)
    : QWidget(parent)
{
  ui.setupUi(this);
  connect( this->ui.pushButton, SIGNAL( clicked() ), this, SLOT(pushButton_SetLabelText()) );
  connect( this->ui.horizontalSlider, SIGNAL( valueChanged(int)), this, SLOT(pushButton_SetLabelText()) );
}

void Form::pushButton_SetLabelText()
{
  this->ui.label->setText(QString::number(this->ui.horizontalSlider->value()));
  std::cout << this->ui.label->text().toDouble() << std::endl;
}
