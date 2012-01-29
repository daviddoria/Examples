#include <QtGui>

#include "form.h"

Form::Form(QWidget *parent)
    : QWidget(parent)
{
  ui.setupUi(this);
  connect( this->ui.pushButton, SIGNAL( clicked() ), this, SLOT(SetLabelText()) );
  connect( this->ui.checkBox, SIGNAL( clicked() ), this, SLOT(SetLabelText()) );
}

void Form::SetLabelText()
{
  if(this->ui.checkBox->isChecked())
  {
    this->ui.label->setText("click: checked");
  }
  else
  {
    this->ui.label->setText("click: not checked");
  }

}
