#include <QtGui>

#include <iostream>

#include "form.h"

Form::Form(QWidget *parent) : QWidget(parent)
{
  setupUi(this);
  //connect( this->ui.pushButton, SIGNAL( clicked() ), this, SLOT(pushButton_SetLabelText()) );
  QTimer::singleShot(0, this, SLOT(AppReady()));
}

void Form::AppReady()
{
  std::cout << "Test." << std::endl;
}
