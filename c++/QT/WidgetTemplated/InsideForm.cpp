#include "InsideForm.h"

#include <iostream>

InsideFormObject::InsideFormObject(QWidget *parent)
{
  setupUi(this);
  connect( this->btnButton, SIGNAL( clicked() ), this, SLOT(btnButton_clicked()) );
}

