#include <QFont>
#include <QtGui>
#include <QGraphicsScene>
#include <QGraphicsView>

#include <iostream>
#include <sstream>

#include "form.h"

Form::Form(QWidget *parent)
    : QWidget(parent)
{
  setupUi(this);
}

void Form::on_btnOpen_clicked()
{
  const unsigned int patchSize = 30;
  QPixmap image(patchSize, patchSize);
  image.fill(QColor(0,0,0));
  
  QGraphicsScene* scene = new QGraphicsScene();
  
  this->graphicsView->setScene(scene);
  
  for(unsigned int i = 0; i < 5; ++i)
    {
    QGraphicsPixmapItem * actor = scene->addPixmap(image);
    
    actor->setOffset(0, patchSize*2 * i);
  
    std::stringstream ss;
    ss << i;
    QGraphicsSimpleTextItem * textActor = scene->addSimpleText(ss.str().c_str());
    textActor->setPos(patchSize, patchSize*2 * i);
    }
  
  

}
