#include <QFont>
#include <QtGui>
#include <QGraphicsScene>
#include <QGraphicsView>

#include <iostream>

#include "form.h"

Form::Form(QWidget *parent)
    : QWidget(parent)
{
  setupUi(this);

  QGraphicsScene* scene = new QGraphicsScene();
  this->graphicsView->setScene(scene);
  QFont font("MS Serif", -1, -1, true);
  QFontInfo info(font);
  scene->addText("text", font);
  this->graphicsView->show();
}
