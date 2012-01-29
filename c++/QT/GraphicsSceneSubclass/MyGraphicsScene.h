#ifndef MyGraphicsScene_H
#define MyGraphicsScene_H

#include <QGraphicsScene>

class MyGraphicsView : public QGraphicsScene
{
public:
  MyGraphicsView(QWidget *parent = 0);
  
  void mousePressEvent(QMouseEvent* pMouseEvent);

};

#endif
