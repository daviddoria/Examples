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
  
  const unsigned int patchSize = 30;
  QPixmap image(patchSize, patchSize);
  image.fill(QColor(0,0,0));
  
  QGraphicsScene* scene = new QGraphicsScene();
  
  this->graphicsView->setScene(scene);
    
  for(unsigned int i = 0; i < 5; ++i)
    {
    QGraphicsPixmapItem * actor = scene->addPixmap(image);
    this->Items.push_back(actor);
    actor->setFlag(QGraphicsItem::ItemIsSelectable, true);
    actor->setOffset(0, patchSize*2 * i);
  
    std::stringstream ss;
    ss << i;
    QGraphicsSimpleTextItem * textActor = scene->addSimpleText(ss.str().c_str());
    textActor->setPos(patchSize, patchSize*2 * i);
    }
}

//       std::stringstream ss;
//       ss << i;
//       this->lblSelected->setText(ss.str().c_str());
