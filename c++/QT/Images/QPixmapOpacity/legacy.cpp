#include <QtGui>
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QPixmap>

#include "form.h"

Form::Form(QWidget *parent)
    : QWidget(parent)
{
  setupUi(this);

  QPixmap pixmap(100,100);
  QColor black(255,0,0);
  pixmap.fill(black);

  //QBitmap alpha(100,100); // If you use a QBitmap, pixels can only take transparency values 1 or 0.
  QPixmap alpha(100,100);
  QPainter painter( &alpha);

  // Set everything to opaque
  //painter.setPen(Qt::color0);
  painter.setPen(QColor( 255, 0, 0, 255 ));
  for(unsigned int i = 0; i < 100; ++i)
    {
    for(unsigned int j = 0; j < 100; ++j)
      {
      painter.drawPoint(i, j);
      }
    }

  // Set the top left corner to transparent
  //painter.setPen(Qt::color1);
  painter.setPen(QColor( 255, 0, 0, 0 ));
  for(unsigned int i = 0; i < 50; ++i)
    {
    for(unsigned int j = 0; j < 50; ++j)
      {
      painter.drawPoint(i, j);
      }
    }

  // Set the top right corner to semi-transparent
  painter.setPen( QColor( 255, 0, 0, 127 ) );
  for(unsigned int i = 50; i < 100; ++i)
    {
    for(unsigned int j = 0; j < 50; ++j)
      {
      painter.drawPoint(i, j);
      }
    }

  // A pixel value of 1 on the mask means the pixmap's pixel is unchanged; a value of 0 means the pixel is transparent.
  pixmap.setAlphaChannel(alpha);
  //pixmap.setMask(alpha);

  QGraphicsScene* scene = new QGraphicsScene;
  scene->addPixmap(pixmap);

  this->graphicsView->setScene(scene);

}
