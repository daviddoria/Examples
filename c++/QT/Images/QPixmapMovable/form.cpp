#include <QtGui>
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QPixmap>

#include "form.h"

Form::Form(QWidget *parent)
    : QWidget(parent)
{
  setupUi(this);

  // Doesn't work
  QImage image(20,20, QImage::Format_ARGB32);
  QColor black(0,0,0, 122);
  image.fill(black.rgba());

  // Works
//   QImage image(20,20, QImage::Format_RGB32);
//   QColor black(0,0,0);
//   image.fill(black.rgb());

  QGraphicsScene* scene = new QGraphicsScene;
  QGraphicsPixmapItem* item = scene->addPixmap(QPixmap::fromImage(image));
  item->setFlags(QGraphicsItem::ItemIsMovable | QGraphicsItem::ItemIsSelectable);

  this->graphicsView->setScene(scene);

}
