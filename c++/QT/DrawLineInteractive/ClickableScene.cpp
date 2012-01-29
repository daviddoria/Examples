#include "ClickableScene.h"

#include <QGraphicsSceneMouseEvent>
#include <QMouseEvent>

#include <iostream>

ClickableScene::ClickableScene(QWidget *parent) : QGraphicsScene()
{
  this->LastClick.setX(0);
  this->LastClick.setY(0);
}

void ClickableScene::mousePressEvent(QGraphicsSceneMouseEvent* pMouseEvent)
{
  std::cout << "clicked " << pMouseEvent->scenePos().x() << " " << pMouseEvent->scenePos().y() << std::endl;
  
  QLineF line(this->LastClick.x(), this->LastClick.y(), pMouseEvent->scenePos().x(), pMouseEvent->scenePos().y());
  this->addLine(line);
  
  this->LastClick.setX(pMouseEvent->scenePos().x());
  this->LastClick.setY(pMouseEvent->scenePos().y());
}
