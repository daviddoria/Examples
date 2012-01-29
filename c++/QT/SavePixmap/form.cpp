#include <QtGui>

#include "form.h"

Form::Form(QWidget *parent)
    : QWidget(parent)
{
  this->setupUi(this);

  Scene = new QGraphicsScene();
  this->graphicsView->setScene(Scene);

  Pixmap = QPixmap(20, 20);
  QColor black(0,0,0);
  Pixmap.fill(black);

  Scene->addPixmap(Pixmap);

}

void Form::on_btnSave_clicked()
{
  QString fileName = QFileDialog::getSaveFileName(this, "Save Scene", "", "Image (*.png)");

  Pixmap.save(fileName);

}
