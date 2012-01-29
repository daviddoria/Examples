#ifndef MyGraphicsView_H
#define MyGraphicsView_H

#include <QGraphicsView>

// To use this class as the type of a QGraphicsView, you must "promote" the widget in QtDesigner to a MyGraphicsView
class MyGraphicsView : public QGraphicsView
{
public:
  MyGraphicsView(QWidget *parent = 0);
  //void mousePressEvent(QGraphicsSceneMouseEvent* pMouseEvent);
  void mousePressEvent(QMouseEvent* pMouseEvent);

};

#endif
