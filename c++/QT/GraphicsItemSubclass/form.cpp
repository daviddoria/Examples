#include <QFont>
#include <QtGui>
#include <QGraphicsScene>
#include <QGraphicsView>

#include <iostream>
#include <sstream>

#include "form.h"

#include "MyGraphicsItem.h"

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
    MyGraphicsItem* item = new MyGraphicsItem;
    connect( item, SIGNAL( ClickedSignal(unsigned int) ), this, SLOT(ClickedSlot(unsigned int)) );
    item->setPixmap(image);
    item->Id = i;
    
    item->setFlag(QGraphicsItem::ItemIsSelectable, true);
    item->setOffset(0, patchSize*2 * i);
    scene->addItem(item);
    
    std::stringstream ss;
    ss << i;
    QGraphicsSimpleTextItem * textActor = scene->addSimpleText(ss.str().c_str());
    textActor->setPos(patchSize, patchSize*2 * i);
    }
}

void Form::ClickedSlot(const unsigned int value)
{
  //std::cout << "ClickedSlot " << value << std::endl;
  std::stringstream ss;
  ss << value;
  this->lblSelected->setText(ss.str().c_str());
}
