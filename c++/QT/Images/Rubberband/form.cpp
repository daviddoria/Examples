#include <QtGui>
#include <QImage>
#include <QRubberBand>

#include "form.h"

#include <iostream>

Form::Form(QWidget *parent)
    : QWidget(parent)
{
  setupUi(this);
  
  QGraphicsScene* scene = new QGraphicsScene();
  this->graphicsView->setScene(scene);
  //this->graphicsView->setDragMode(QGraphicsView::RubberBandDrag);
  this->graphicsView->show();
  
  QPixmap pixmap(100,100);
  pixmap.fill(QColor(255,0,0));
  
  QGraphicsPixmapItem* item = scene->addPixmap(pixmap);
  item->setFlag(QGraphicsItem::ItemIsMovable);

}

void Form::SelectionChanged()
{
  std::cout << "Selection changed." << std::endl;
}
