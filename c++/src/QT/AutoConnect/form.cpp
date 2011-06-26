#include <QtGui>

#include "form.h"

Form::Form(QWidget *parent)
    : QWidget(parent)
{
  setupUi(this);
}

void Form::on_pushButton_clicked()
{
  this->label->setText(this->textEdit->document()->toPlainText());
}
