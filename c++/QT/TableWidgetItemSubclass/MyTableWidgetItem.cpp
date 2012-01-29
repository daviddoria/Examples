#include "MyGraphicsItem.h"

#include <QGraphicsSceneMouseEvent>

#include <iostream>

void MyGraphicsItem::mousePressEvent(QGraphicsSceneMouseEvent* pMouseEvent)
{
  //std::cout << "mousePressEvent: " << this->Id << std::endl;
  emit ClickedSignal(this->Id);
}
