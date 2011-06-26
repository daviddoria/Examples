#include <QtGui>
#include <QImage>

#include "form.h"

Form::Form(QWidget *parent)
    : QWidget(parent)
{
  setupUi(this);
  QImage image;
  image.load("test.jpg");

}
