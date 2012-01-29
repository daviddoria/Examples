#include <QtGui>
#include <QDialogButtonBox>

#include "form.h"

Form::Form(QWidget *parent) : QWidget(parent)
{
  QDialogButtonBox* buttonBox = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, Qt::Horizontal, this);
  buttonBox->button(QDialogButtonBox::Ok)->setEnabled(false);
  
  this->show();
}
