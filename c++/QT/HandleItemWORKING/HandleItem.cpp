#include "HandleItem.h"

#include <QPainter>
#include <QPointF>

#include <iostream>

HandleItem::HandleItem( QGraphicsRectItem *item, QGraphicsScene *scene, QColor color, HandleItem::HandleRole role, QList<HandleItem*> handles ) : QGraphicsItem( 0, scene )
{
  m_role = role;
  m_color = color;
  
  m_item = item;
  m_handles = handles;
  
  m_pressed = false;
  setZValue( 100 );

  setFlag( ItemIsMovable );
  setFlag(ItemSendsGeometryChanges);
}

void HandleItem::paint( QPainter *paint, const QStyleOptionGraphicsItem *option, QWidget *widget )
{
  paint->setPen( m_color );
  paint->setBrush( m_color );
    
  QRectF rect = boundingRect();
  QVector<QPointF> points;
    
  switch( m_role )
  {
  case RightHandle:
    points << rect.center()+QPointF(3,0) << rect.center()+QPointF(-3,-5) << rect.center()+QPointF(-3,5);
    paint->drawConvexPolygon( QPolygonF(points) );
    break;
  }
}


QRectF HandleItem::boundingRect() const
{
  QPointF point = m_item->boundingRect().center();

  switch( m_role )
  {
  case RightHandle:
    //point.setX( m_item->boundingRect().right() );
    point.setX( m_item->boundingRect().right() - pos().x() );
    return QRectF( point-QPointF(3, 5), QSize( 6, 10 ) );
  }
  
  return QRectF();
}

QVariant HandleItem::itemChange( GraphicsItemChange change, const QVariant &data )
{
  if( change == ItemPositionChange && m_pressed )
  {

    QPointF movement;
    QPointF newData = data.toPointF();
    QRectF newRect = m_item->rect();
    
    switch( m_role )
    {
    case RightHandle:
      std::cout << "itemChange RightHandle()" << std::endl;
      // Prevent the rectangle from collapsing.
      if( m_item->sceneBoundingRect().width() + movement.x() <= 5 )
	{
	std::cout << "too small! " << std::endl;
	return QGraphicsItem::itemChange( change, newData );
	}
    
      // Snap the movement to the X direction
      newData.setY(0);

      movement = newData - pos();
      std::cout << "current pos: " << pos().x() << " , " << pos().y() << std::endl;
      std::cout << "input data: " << data.toPoint().x() << " , " << data.toPoint().y() << std::endl;
      std::cout << "new data: " << newData.toPoint().x() << " , " << newData.toPoint().y() << std::endl;
      std::cout << "movement: " << movement.x() << " , " << movement.y() << std::endl;
      
      // Resize the rectangle
      std::cout << "Old right rect pos: " << m_item->rect().right() << std::endl;
      newRect.setRight(m_item->rect().right() + movement.x());
      
      std::cout << "Set new right rect pos: " << newRect.right() << std::endl;
      m_item->setRect(newRect);
    
      break;
    } // end switch

    return QGraphicsItem::itemChange( change, newData);
  } // end if pressed
  
  return QGraphicsItem::itemChange( change, data );
}

void HandleItem::mousePressEvent( QGraphicsSceneMouseEvent *event )
{
  m_pressed = true;  
  QGraphicsItem::mousePressEvent( event );
}

void HandleItem::mouseReleaseEvent( QGraphicsSceneMouseEvent *event )
{
  m_pressed = false;
  QGraphicsItem::mouseReleaseEvent( event );
}
