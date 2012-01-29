#include "filemenu.h"
#include "InsideForm.h"

#include <iostream>

MyForm::MyForm(QWidget *parent)
{
  setupUi(this);
  connect( this->mnuOpenInnerForm, SIGNAL( triggered() ), this, SLOT(mnuOpenInnerForm_triggered()) );

}

void MyForm::mnuOpenInnerForm_triggered()
{
  std::cout << "Inner form." << std::endl;
  MyInsideForm* insideForm = new MyInsideForm(this);
  this->setCentralWidget(insideForm);
}
