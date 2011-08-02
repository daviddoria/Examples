#include <QtGui>

#include "form.h"

Form::Form(QWidget *parent)
    : QWidget(parent)
{
  ui.setupUi(this);
  
  this->setWindowState(Qt::WindowMaximized);

}
