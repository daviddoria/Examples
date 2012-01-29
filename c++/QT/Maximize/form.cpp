#include <QtGui>

#include "form.h"

Form::Form(QWidget *parent)
    : QWidget(parent)
{
  setupUi(this);
  
  this->setWindowState(Qt::WindowMaximized);

}
