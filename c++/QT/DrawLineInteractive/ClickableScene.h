#ifndef ClickableScene_H
#define ClickableScene_H

#include <QGraphicsScene>
#include <QMouseEvent>

class ClickableScene : public QGraphicsScene
{
public:
  ClickableScene(QWidget *parent = 0);
  
  void mousePressEvent(QGraphicsSceneMouseEvent* pMouseEvent);

protected:
  QPointF LastClick;
};

#endif
