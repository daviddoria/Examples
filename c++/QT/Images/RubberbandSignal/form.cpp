#include <QtGui>
#include <QImage>
#include <QRubberBand>
#include <QGraphicsView>

#include "form.h"

#include <iostream>

class MyGraphicsView : public QGraphicsView
{
public:
  
  MyGraphicsView( QWidget * parent = 0 ) : QGraphicsView(parent){}
  
  void mouseReleaseEvent(QMouseEvent *event);
  void mouseMoveEvent(QMouseEvent *event);
  void mousePressEvent(QMouseEvent *event);
};

void MyGraphicsView::mousePressEvent(QMouseEvent *event)
{
  QGraphicsView::mousePressEvent(event);
  QPoint point = event->pos();
  std::cout << "mousePressEvent: " << point.x() << " " << point.y() << std::endl;
}

void MyGraphicsView::mouseMoveEvent(QMouseEvent *event)
{
  QGraphicsView::mouseMoveEvent(event);
  QPoint point = event->pos();
  std::cout << "mouseMoveEvent: " << point.x() << " " << point.y() << std::endl;
  
}

void MyGraphicsView::mouseReleaseEvent(QMouseEvent *event)
{
  QGraphicsView::mouseReleaseEvent(event);
  QPoint point = event->pos();
  std::cout << "mouseReleaseEvent: " << point.x() << " " << point.y() << std::endl;
}

Form::Form(QWidget *parent)
    : QWidget(parent)
{
  setupUi(this);
  
  MyGraphicsView* graphicsView = new MyGraphicsView(this );
  graphicsView->show();
  
  QGraphicsScene* scene = new QGraphicsScene();
  graphicsView->setScene(scene);
  graphicsView->setDragMode(QGraphicsView::RubberBandDrag);
  graphicsView->show();
  
  QPixmap pixmap(100,100);
  pixmap.fill(QColor(255,0,0));
  
  QGraphicsPixmapItem* item = scene->addPixmap(pixmap);

}
