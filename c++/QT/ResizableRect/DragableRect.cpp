#include "DragableRect.h"

DragableRect::DragableRect()
{
drag = false;
setFlag(QGraphicsItem::ItemIsMovable);

QColor color = QColor(Qt::yellow);
QPen pen = QPen(color);
QBrush brush = QBrush(Qt::yellow, Qt::SolidPattern);
setPen(pen);
setBrush(brush);
}

void DragableRect::mousePressEvent(QGraphicsSceneMouseEvent *event)
{
drag = true;
QGraphicsRectItem::mousePressEvent(event);
}

void DragableRect::mouseReleaseEvent(QGraphicsSceneMouseEvent *event)
{
drag = false;
QGraphicsRectItem::mouseReleaseEvent(event);
}

void DragableRect::mouseMoveEvent(QGraphicsSceneMouseEvent *event)
{
if (drag)
{
QGraphicsRectItem::mouseMoveEvent(event);
update();
}
}