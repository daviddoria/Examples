#include <QtGui>

#include "form.h"

Form::Form(QWidget *parent)
    : QWidget(parent)
{
  this->setupUi(this);

  QGraphicsScene* scene = new QGraphicsScene();
  this->graphicsView->setScene(scene);

  QPixmap pixmap(20, 20);
  QColor black(0,0,0);
  pixmap.fill(black);

  scene->addPixmap(pixmap);

}

void Form::on_btnSave_clicked()
{
  QString fileName = QFileDialog::getSaveFileName(this, "Save Scene", "", "Image (*.png)");
  QPixmap pixMap = QPixmap::grabWidget(this->graphicsView);
  pixMap.save(fileName);

}
