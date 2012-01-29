#include "MyGraphicsView.h"

#include <QGraphicsSceneMouseEvent>
#include <QMouseEvent>

#include <iostream>

MyGraphicsView::MyGraphicsView(QWidget *parent) : QGraphicsView(parent)
{
  
}

//void MyGraphicsView::mousePressEvent(QGraphicsSceneMouseEvent* pMouseEvent)
void MyGraphicsView::mousePressEvent(QMouseEvent* pMouseEvent)
{
  //QGraphicsItem* pItemUnderMouse = itemAt(pMouseEvent->scenePos().x(), pMouseEvent->scenePos().y());
  //std::cout << "pItemUnderMouse: " << pItemUnderMouse << std::endl;
  std::cout << "mouseEvent" << std::endl;
  
  //QGraphicsSceneMouseEvent* event = dynamic_cast<QGraphicsSceneMouseEvent*>(pMouseEvent);
  QGraphicsSceneMouseEvent* event = reinterpret_cast<QGraphicsSceneMouseEvent*>(pMouseEvent);
  QGraphicsItem* pItemUnderMouse = itemAt(event->scenePos().x(), event->scenePos().y());
  std::cout << "pItemUnderMouse: " << pItemUnderMouse << std::endl;
}
