#include <QtGui>

#include "form.h"

Form::Form(QWidget *parent)
    : QWidget(parent)
{
  ui.setupUi(this);
  connect( this->ui.pushButton, SIGNAL( clicked() ), this, SLOT(pushButton_SetLabelText()) );
  connect( this->ui.radioButton, SIGNAL( clicked() ), this, SLOT(radioButton_SetLabelText()) );
  connect( this->ui.radioButton_2, SIGNAL( clicked() ), this, SLOT(radioButton_2_SetLabelText()) );
}

void Form::pushButton_SetLabelText()
{
  
  if(this->ui.radioButton->isChecked())
  {
    this->ui.label->setText("1 selected");
  }
  
  if(this->ui.radioButton_2->isChecked())
  {
      this->ui.label->setText("2 selected");
  }

}

void Form::radioButton_SetLabelText()
{
  
  if(this->ui.radioButton->isChecked())
  {
    this->ui.label->setText("1 clicked");
  }

}

void Form::radioButton_2_SetLabelText()
{
  
  if(this->ui.radioButton_2->isChecked())
  {
    this->ui.label->setText("2 clicked");
  }

}
