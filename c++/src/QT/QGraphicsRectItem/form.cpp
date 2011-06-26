//#include <QGraphicsRectItem>
#include <QRectF>
#include <QtGui>
#include <QGraphicsScene>
#include <QGraphicsView>

#include <iostream>

#include "form.h"

Form::Form(QWidget *parent)
    : QWidget(parent)
{
  setupUi(this);
  QGraphicsScene scene;

  //QGraphicsRectItem rect;
  QRectF rect;
  rect.setHeight(10);
  rect.setWidth(10);
  rect.setLeft(10);
  rect.setRight(20);

  scene.addRect(rect);

  graphicsView->setScene(&scene);
  graphicsView->show();

}
