#include "form.h"

MyForm::MyForm(QWidget *parent)
    : QWidget(parent)
{
  setupUi(this);
  connect( this->pushButton, SIGNAL( clicked() ), this, SLOT(pushButton_SetLabelText()) );
}

void MyForm::pushButton_SetLabelText()
{
  this->label->setText("hello");
}

