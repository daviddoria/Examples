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
  QGraphicsScene* scene = new QGraphicsScene;

  QRectF rect;
  rect.setHeight(10);
  rect.setWidth(10);
  rect.setLeft(10);
  rect.setRight(20);

  QGraphicsRectItem* rectItem = scene->addRect(rect);
  rectItem->setBrush(QBrush(QColor(255, 0, 0)));
  
  graphicsView->setScene(scene);
  graphicsView->show();

}
