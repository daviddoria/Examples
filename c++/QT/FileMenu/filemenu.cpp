#include "filemenu.h"

#include <iostream>

MyForm::MyForm(QWidget *parent)
{
  setupUi(this);
  connect( this->mnuTest, SIGNAL( triggered() ), this, SLOT(mnuTest_triggered()) );
}

void MyForm::mnuTest_triggered()
{
  std::cout << "Clicked." << std::endl;
}
