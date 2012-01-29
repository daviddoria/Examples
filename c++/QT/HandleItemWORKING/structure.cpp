/*
 * Copyright (c) 2006-2007, Johan Thelin
 * 
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification, 
 * are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright notice, 
 *       this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright notice,  
 *       this list of conditions and the following disclaimer in the documentation 
 *       and/or other materials provided with the distribution.
 *     * Neither the name of APress nor the names of its contributors 
 *       may be used to endorse or promote products derived from this software 
 *       without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

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
  case CenterHandle:
    paint->drawEllipse( rect );
    break;
  case RightHandle:
    points << rect.center()+QPointF(3,0) << rect.center()+QPointF(-3,-5) << rect.center()+QPointF(-3,5);
    paint->drawConvexPolygon( QPolygonF(points) );
    break;
  case TopHandle:
    points << rect.center()+QPointF(0,-3) << rect.center()+QPointF(-5,3) << rect.center()+QPointF(5,3);
    paint->drawConvexPolygon( QPolygonF(points) );
    break;
  case LeftHandle:
    points << rect.center()+QPointF(-3,0) << rect.center()+QPointF(3,-5) << rect.center()+QPointF(3,5);
    paint->drawConvexPolygon( QPolygonF(points) );
    break;
  case BottomHandle:
    points << rect.center()+QPointF(0,3) << rect.center()+QPointF(-5,-3) << rect.center()+QPointF(5,-3);
    paint->drawConvexPolygon( QPolygonF(points) );
    break;
  default:
    break;
  }
}


QRectF HandleItem::boundingRect() const
{
  QPointF point = m_item->boundingRect().center();

  switch( m_role )
  {
  case CenterHandle:
    return QRectF( point-QPointF(5, 5), QSize( 10, 10 ) );
  case RightHandle:
    point.setX( m_item->boundingRect().right() );
    return QRectF( point-QPointF(3, 5), QSize( 6, 10 ) );
  case LeftHandle:
    point.setX( m_item->boundingRect().left() );
    return QRectF( point-QPointF(3, 5), QSize( 6, 10 ) );
  case TopHandle:
    point.setY( m_item->boundingRect().top() );
    return QRectF( point-QPointF(5, 3), QSize( 10, 6 ) );
  case BottomHandle:
    point.setY( m_item->boundingRect().bottom() );
    return QRectF( point-QPointF(5, 3), QSize( 10, 6 ) );
  default:
    break;
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
    case CenterHandle:
      std::cout << "itemChange CenterHandle()" << std::endl;
      m_item->moveBy( movement.x(), movement.y() );
      
      foreach( HandleItem *handle, m_handles )
        handle->translate( movement.x(), movement.y() );

      break;
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
      
    case LeftHandle:
      std::cout << "itemChange LeftHandle()" << std::endl;
      if( 2*movement.x() + m_item->sceneBoundingRect().width() <= 5 )
        return QGraphicsItem::itemChange( change, newData );
    
      movement.setY( 0 );
/*      
      m_item->translate( center.x(), center.y() );      
      m_item->scale( 1.0+2.0*movement.x()/(m_item->sceneBoundingRect().width()), 1 );
      m_item->translate( -center.x(), -center.y() );
        */
      break;
    case TopHandle:
      std::cout << "itemChange TopHandle()" << std::endl;
      if( -2*movement.y() + m_item->sceneBoundingRect().height() <= 5 )
        return QGraphicsItem::itemChange( change, newData );
    
      movement.setX( 0 );
      
//       m_item->translate( center.x(), center.y() );
//       m_item->scale( 1, 1.0-2.0*movement.y()/(m_item->sceneBoundingRect().height()) );
//       m_item->translate( -center.x(), -center.y() );
      break;
    case BottomHandle:
      std::cout << "itemChange BottomHandle()" << std::endl;
      if( -2*movement.y() + m_item->sceneBoundingRect().height() <= 5 )
        return QGraphicsItem::itemChange( change, newData );
    
      movement.setX( 0 );
/*      
      m_item->translate( center.x(), center.y() );
      m_item->scale( 1, 1.0-2.0*movement.y()/(m_item->sceneBoundingRect().height()) );
      m_item->translate( -center.x(), -center.y() );*/
      break;
    default:
      break;
    } // end switch
    
    //return QGraphicsItem::itemChange( change, pos()+movement );
    return QGraphicsItem::itemChange( change, newData);
    //return QGraphicsItem::itemChange( change, data);
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
