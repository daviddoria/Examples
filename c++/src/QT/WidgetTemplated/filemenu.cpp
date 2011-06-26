#include "filemenu.h"
#include "InsideForm.h"

#include <iostream>

MyForm::MyForm(QWidget *parent)
{
  setupUi(this);
  connect( this->mnuCreateIntForm, SIGNAL( triggered() ), this, SLOT(mnuCreateIntForm_triggered()) );
  connect( this->mnuCreateDoubleForm, SIGNAL( triggered() ), this, SLOT(mnuCreateDoubleForm_triggered()) );
}

void MyForm::mnuCreateIntForm_triggered()
{
  std::cout << "Inner form." << std::endl;
  MyInsideForm<int>* insideForm = new MyInsideForm<int>(this);
  this->setCentralWidget(insideForm);
}

void MyForm::mnuCreateDoubleForm_triggered()
{
  std::cout << "Inner form." << std::endl;
  MyInsideForm<double>* insideForm = new MyInsideForm<double>(this);
  this->setCentralWidget(insideForm);
}
