#include <QtGui>
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QPixmap>

#include <iostream>

#include "form.h"

Form::Form(QWidget *parent) : QWidget(parent)
{
  setupUi(this);

  this->Pixmap.load("glasses.jpg");
  
  this->Scene = new QGraphicsScene;
  this->PixmapItem = this->Scene->addPixmap(this->Pixmap);

  this->View = new CustomGraphicsView;
  this->View->setScene(this->Scene);

  this->horizontalLayout->addWidget(this->View);

  this->connect(this->View, SIGNAL(resized()), SLOT(SizeImage()));
}

void Form::SizeImage()
{
  this->View->fitInView (this->PixmapItem);
}

