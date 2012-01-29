#include <QtGui>

#include "form.h"

Form::Form(QWidget *parent) : QWidget(parent)
{
  #ifdef QT_VERSION
    #pragma message("Using QT!")
  #else
    #pragma message("NOT using QT!")
  #endif

}
