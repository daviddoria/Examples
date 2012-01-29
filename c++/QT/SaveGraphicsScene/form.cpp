#include <QtGui>

#include "form.h"

Form::Form(QWidget *parent)
    : QWidget(parent)
{
  this->setupUi(this);

  Scene = new QGraphicsScene();
  this->graphicsView->setScene(Scene);

  QPixmap blackPixmap(20, 20);
  QColor black(0,0,0);
  blackPixmap.fill(black);

  QPixmap redPixmap(20, 20);
  QColor red(255,0,0);
  redPixmap.fill(red);

  Scene->addPixmap(redPixmap);
  QGraphicsPixmapItem* item = Scene->addPixmap(blackPixmap);
  item->setPos(20,20);

}

void Form::on_btnSave_clicked()
{
  QString fileName = QFileDialog::getSaveFileName(this, "Save Scene", "", "Image (*.png)");

  QPixmap sceneImage(Scene->sceneRect().width(), Scene->sceneRect().height());
  QColor white(255,255,255);
  sceneImage.fill(white);
  
  QPainter painter(&sceneImage);
  painter.setRenderHint(QPainter::Antialiasing);
  Scene->render(&painter);
  sceneImage.save(fileName);

}
