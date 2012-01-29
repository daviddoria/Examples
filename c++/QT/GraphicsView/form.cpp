#include <QtGui>

#include <iostream>

#include "form.h"

Form::Form(QWidget *parent) : QWidget(parent)
{
  this->setupUi(this);

  QGraphicsScene* scene = new QGraphicsScene;
  
  //// Add a qpixmap to the scene.
  QPixmap pixmap(40, 40);
  QColor black(0,0,0);
  pixmap.fill(black);
  this->BlackItem = scene->addPixmap(pixmap);
  //this->graphicsView->fitInView(blackItem);
  
  //Add another qpixmap to the scene. Drag one of them around and watch the scene change.
  QPixmap pixmap2(30, 30);
  QColor red(255,0,0);
  pixmap2.fill(red);
  QGraphicsPixmapItem * redItem = scene->addPixmap(pixmap2);
  redItem->setFlag(QGraphicsItem::ItemIsMovable);

  scene->setSceneRect(pixmap.rect());
  this->graphicsView->setScene(scene);
}

void Form::showEvent ( QShowEvent * event )
{
  std::cout << "ShowEvent" << std::endl;
  this->graphicsView->fitInView(this->BlackItem);
}

void Form::resizeEvent ( QResizeEvent * event )
{
  std::cout << "ResizeEvent" << std::endl;
  this->graphicsView->fitInView(this->BlackItem);
}
