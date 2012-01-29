#include <QtGui>

#include "form.h"

Form::Form(QWidget *parent)
    : QWidget(parent)
{
  this->setupUi(this);
  //connect( this->ui.pushButton, SIGNAL( clicked() ), this, SLOT(pushButton_SetLabelText()) );

  QColor sceneBackgroundColor;
  sceneBackgroundColor.setRgb(153, 255, 0);
  
  QGraphicsScene* scene = new QGraphicsScene();
  QBrush brush;
  brush.setStyle(Qt::SolidPattern);
  brush.setColor(sceneBackgroundColor);
  scene->setBackgroundBrush(brush);
  this->graphicsView->setScene(scene);
}
