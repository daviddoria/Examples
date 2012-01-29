#include <QtGui>
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QPixmap>

#include "form.h"

Form::Form(QWidget *parent)
    : QWidget(parent)
{
  setupUi(this);

  QColor transparentRed(255,0,0, 0);
  QColor semiTransparentRed(255,0,0, 127);
  QColor opaqueRed(255,0,0, 255);
  
  QImage image(100,100,QImage::Format_ARGB32);
  image.fill(opaqueRed.rgba());

  // Set the top left corner to totally transparent
  for(unsigned int i = 0; i < 50; ++i)
    {
    for(unsigned int j = 0; j < 50; ++j)
      {
      image.setPixel(i,j,transparentRed.rgba());
      }
    }

  // Set the top right corner to semi-transparent red
  for(unsigned int i = 50; i < 100; ++i)
    {
    for(unsigned int j = 0; j < 50; ++j)
      {
      image.setPixel(i,j,semiTransparentRed.rgba());
      }
    }
  
  QPixmap pixmap = QPixmap::fromImage(image);
  QGraphicsScene* scene = new QGraphicsScene;
  scene->addPixmap(pixmap);
  
  this->graphicsView->setScene(scene);

}
