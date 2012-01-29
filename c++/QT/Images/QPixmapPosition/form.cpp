#include <QtGui>
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QPixmap>

#include <iostream>

#include "form.h"

Form::Form(QWidget *parent) : QWidget(parent)
{
  setupUi(this);

  this->Pixmap.load("/home/doriad/glasses.jpg");
  this->Pixmap.scaledToHeight(200);
  
  this->Scene = new QGraphicsScene;
  this->graphicsView->setScene(this->Scene);

  this->connect(this->Scene, SIGNAL(sceneRectChanged(QRectF)), SLOT(slot_Scene_sceneRectChanged(QRectF)));
}

void Form::on_pushButton_clicked()
{
  //std::cout << "Clicked." << std::endl;
  SizeImage();
}

void Form::SizeImage()
{


  this->Pixmap = this->Pixmap.scaledToWidth(this->graphicsView->width());

  QGraphicsPixmapItem * item = this->Scene->addPixmap(this->Pixmap);
  this->graphicsView->fitInView (item);
}

//void Form::on_graphicsView_sceneRectChanged( const QRectF & rect )
void Form::slot_Scene_sceneRectChanged( const QRectF & rect )
{
  std::cout << "slot_Scene_sceneRectChanged()" << std::endl;
  SizeImage();
}
