#include <QtGui>

#include "form.h"

Form::Form(QWidget *parent)
    : QWidget(parent)
{
  ui.setupUi(this);
  connect( this->ui.pushButton, SIGNAL( clicked() ), this, SLOT(pushButton_SetLabelText()) );
}

void Form::pushButton_SetLabelText()
{
  this->ui.label->setText(this->ui.textEdit->document()->toPlainText());
  
}
