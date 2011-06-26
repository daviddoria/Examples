#include "InsideForm.h"

#include <iostream>

MyInsideForm::MyInsideForm(QWidget *parent)
{
  setupUi(this);
  connect( this->btnButton, SIGNAL( clicked() ), this, SLOT(btnButton_clicked()) );
}

void MyInsideForm::btnButton_clicked()
{
  this->lblLabel->setText("Clicked");
}
