#include <QtGui>

#include "form.h"
#include "HandleItem.h"

#include <iostream>

Form::Form(QWidget *parent) : QWidget(parent)
{
  setupUi(this);
  
  QGraphicsScene* scene = new QGraphicsScene( 0, 0, 200, 200 );
  QGraphicsRectItem *rectItem = new QGraphicsRectItem( QRect( 10, 10, 50, 100 ), 0, scene );
  
  HandleItem *trh = new HandleItem( rectItem, scene, Qt::red, HandleItem::TopHandle );
  HandleItem *rrh = new HandleItem( rectItem, scene, Qt::red, HandleItem::RightHandle );
  HandleItem *crh = new HandleItem( rectItem, scene, Qt::red, HandleItem::CenterHandle, QList<HandleItem*>() << trh << rrh );
  
  this->graphicsView->setScene( scene );
  
}
