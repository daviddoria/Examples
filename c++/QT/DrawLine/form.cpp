#include <QtGui>
#include <QLineF>

#include "form.h"

Form::Form(QWidget *parent)
    : QWidget(parent)
{
  setupUi(this);
  
  QColor sceneBackgroundColor;
  sceneBackgroundColor.setRgb(153, 255, 0);
  
  QGraphicsScene* scene = new QGraphicsScene();
  QBrush brush;
  brush.setStyle(Qt::SolidPattern);
  brush.setColor(sceneBackgroundColor);
  scene->setBackgroundBrush(brush);
  
  //QGraphicsLineItem *	addLine ( const QLineF & line, const QPen & pen = QPen() )
  QLineF line(0,0, 50, 50);
  scene->addLine(line);
  
  this->graphicsView->setScene(scene);
  
  
}
