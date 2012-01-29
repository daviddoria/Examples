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
  
  Scene = new QGraphicsScene();

  this->graphicsView->setScene(Scene);
  
}

void Form::showEvent ( QShowEvent * event )
{
  for(unsigned int i = 0; i < 5; ++i)
    {

    std::stringstream ss;
    ss << i;
    QGraphicsTextItem* simpleTextItem = Scene->addText(ss.str().c_str());
    simpleTextItem->setPos(0, simpleTextItem->boundingRect().height() * i);
    this->graphicsView->fitInView(simpleTextItem, Qt::KeepAspectRatio);
    }
  this->graphicsView->fitInView(this->Scene->sceneRect(), Qt::KeepAspectRatio);
}

void Form::resizeEvent ( QResizeEvent * event )
{
  this->graphicsView->fitInView(this->Scene->sceneRect(), Qt::KeepAspectRatio);
}
