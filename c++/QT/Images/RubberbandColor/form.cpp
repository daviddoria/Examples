#include <QtGui>
#include <QImage>
#include <QRubberBand>

#include "form.h"

#include <iostream>

Form::Form(QWidget *parent)
    : QWidget(parent)
{
  setupUi(this);
  
  QRubberBand* band = new QRubberBand(QRubberBand::Rectangle);

  QPalette pal;
  pal.setBrush(QPalette::Highlight, QBrush(Qt::red));
  band->setPalette(pal);

  band->resize(30, 30);
  band->show();
  
}
