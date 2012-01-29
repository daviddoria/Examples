#include <QtGui>

#include "form.h"

Form::Form(QWidget *parent)
    : QWidget(parent)
{
  this->setupUi(this);
  connect( this->btnUpdate, SIGNAL( clicked() ), this, SLOT(btnUpdate_SetLabelText()) );
  connect( this->lineEdit, SIGNAL( returnPressed()), this, SLOT(btnUpdate_SetLabelText()) );
  connect( this->lineEdit, SIGNAL( editingFinished()), this, SLOT(btnUpdate_SetLabelText()) );
}

void Form::on_lineEdit_textEdited( const QString & text )
{
  this->label->setText(this->lineEdit->text());
}


void Form::btnUpdate_SetLabelText()
{
  this->label->setText(this->lineEdit->text());
}
