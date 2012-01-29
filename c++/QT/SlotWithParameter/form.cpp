#include "form.h"

#include <string>

MyForm::MyForm(QWidget *parent)
    : QWidget(parent)
{
  setupUi(this);
  connect( this->btnButton, SIGNAL( clicked() ), this, SLOT(btnButton_clicked()) );
}

void MyForm::btnButton_clicked()
{
  this->lblLabel->setText("hello");
}
