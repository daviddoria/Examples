#include "form.h"
#include "thread.h"

Form::Form(QWidget *parent) : QWidget(parent)
{
  setupUi(this);
  connect( this->pushButton, SIGNAL( clicked() ), this, SLOT(pushButton_clicked()) );
}

void Form::pushButton_clicked()
{
  Thread a;
  a.start();
}


