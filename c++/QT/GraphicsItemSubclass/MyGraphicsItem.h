#ifndef MyGraphicsItem_H
#define MyGraphicsItem_H

#include <QGraphicsPixmapItem>
#include <QObject>

class MyGraphicsItem : public QObject, public QGraphicsPixmapItem // These must be inherited in this order to prevent "staticMetaObject is not a member" errors!
{
  Q_OBJECT
public:
  
  void mousePressEvent(QGraphicsSceneMouseEvent* pMouseEvent);
  unsigned int Id;

signals:
  void ClickedSignal(const unsigned int);
};

#endif
