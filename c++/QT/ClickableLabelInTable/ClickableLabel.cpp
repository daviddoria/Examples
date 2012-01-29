#include "ClickableLabel.h"

#include <QMouseEvent>

#include <iostream>

void ClickableLabel::mousePressEvent(QMouseEvent* pMouseEvent)
{
  std::cout << "mousePressEvent: " << this->Id << std::endl;
  emit ClickedSignal(this->Id);
}
