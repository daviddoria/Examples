#include <QtGui>
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QPixmap>

#include "form.h"

Form::Form(QWidget *parent)
    : QWidget(parent)
{
  setupUi(this);

  QPixmap pixmap;
  pixmap.load("test.jpg");

  QGraphicsScene scene;
  scene.addPixmap(pixmap);

  graphicsView->setScene(&scene);
  graphicsView->show();

}
