#include <QtGui>
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QPixmap>

#include "form.h"

Form::Form(QWidget *parent)
    : QWidget(parent)
{
  setupUi(this);

  //QPixmap pixmap;
  //pixmap.load("test.jpg");

  // Manual:
  //pixmap = pixmap.scaled(20,20);
  
  // Works
//   QPixmap pixmap(20,20);
//   QColor black(0,0,0);
//   pixmap.fill(black);
  
  //QPixmap pixmap; // Doesn't work (probably because the default size is (0,0) so scaling 0 to anything doesn't make sense)
  QPixmap pixmap(1,1); // Works
  pixmap = pixmap.scaled(20,20);
  QColor black(0,0,0);
  pixmap.fill(black);

  QGraphicsScene* scene = new QGraphicsScene;
  scene->addPixmap(pixmap);
  
  this->graphicsView->setScene(scene);

}
