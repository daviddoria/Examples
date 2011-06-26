#include <QtGui>

#include "form.h"

Form::Form(QWidget *parent)
    : QWidget(parent)
{
  ui.setupUi(this);
  connect( this->ui.btnUpdate, SIGNAL( clicked() ), this, SLOT(btnUpdate_SetLabelText()) );
  connect( this->ui.lineEdit, SIGNAL( returnPressed()), this, SLOT(btnUpdate_SetLabelText()) );
  connect( this->ui.lineEdit, SIGNAL( editingFinished()), this, SLOT(btnUpdate_SetLabelText()) );
}

void Form::btnUpdate_SetLabelText()
{
  this->ui.label->setText(this->ui.lineEdit->text());
}
