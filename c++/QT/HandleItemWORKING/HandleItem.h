#ifndef HANDLEITEM_H
#define HANDLEITEM_H

#include <QGraphicsItem>
#include <QGraphicsRectItem>

class HandleItem;

class HandleItem : public QGraphicsItem
{
public:

  enum HandleRole
  {
    CenterHandle,
    RightHandle,
    TopHandle,
    LeftHandle,
    BottomHandle
  };

  HandleItem( QGraphicsRectItem *item, QGraphicsScene *scene, QColor color, HandleRole role = CenterHandle, QList<HandleItem*> handles = QList<HandleItem*>() );
  
  void paint( QPainter *paint, const QStyleOptionGraphicsItem *option, QWidget *widget );
  QRectF boundingRect() const;

protected:
  void mousePressEvent( QGraphicsSceneMouseEvent *event );
  void mouseReleaseEvent( QGraphicsSceneMouseEvent *event );
  
  QVariant itemChange( GraphicsItemChange change, const QVariant &data );

private:
  QGraphicsRectItem *m_item;
  
  HandleRole m_role;
  QColor m_color;
  
  QList<HandleItem*> m_handles;

  bool m_pressed;
};

#endif // HANDLEITEM_H
