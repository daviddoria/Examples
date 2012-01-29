#include <QtGui>

// This lets you set something like "000-000" to force the user to enter 6 digits separated by a -.

#include "form.h"

Form::Form(QWidget *parent)
    : QWidget(parent)
{
  this->setupUi(this);
}

void Form::on_lineEdit_textEdited( const QString & text )
{
  this->label->setText(this->lineEdit->text().remove(QChar('-')));
}

