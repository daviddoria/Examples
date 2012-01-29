#include <QtGui>

#include "form.h"

Form::Form(QWidget *parent)
    : QWidget(parent)
{
  this->setupUi(this);
}

void Form::on_lineEdit_textEdited( const QString & text )
{
  this->label->setText(this->lineEdit->text());
}
