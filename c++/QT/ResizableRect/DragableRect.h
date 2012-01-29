#ifndef DRAGABLERECT_H
#define DRAGABLERECT_H

#include <QtGui>
#include <iostream>

class DragableRect : public QGraphicsRectItem
{
public:
DragableRect();

protected:
void mousePressEvent(QGraphicsSceneMouseEvent *);
void mouseReleaseEvent(QGraphicsSceneMouseEvent *);
void mouseMoveEvent(QGraphicsSceneMouseEvent *);
bool drag;
};

#endif // DRAGABLERECT_H