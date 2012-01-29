#include <QtGui>
#include <QImage>
#include <QRubberBand>

#include "form.h"

#include <iostream>

Form::Form(QWidget *parent)
    : QWidget(parent)
{
  setupUi(this);
  
  QGraphicsScene* scene = new QGraphicsScene();
  
  connect(scene, SIGNAL(selectionChanged()), this, SLOT(SelectionChanged()));
  //QRubberBand* rubberBand = new QRubberBand(QRubberBand::Rectangle, scene);
  //QRubberBand* rubberBand = new QRubberBand(QRubberBand::Rectangle);
  QRubberBand* rubberBand = new QRubberBand(QRubberBand::Rectangle, this->graphicsView);
  
  QPalette palette;
  palette.setBrush(QPalette::Foreground, QBrush(Qt::green));
  palette.setBrush(QPalette::Base, QBrush(Qt::red));

  rubberBand->setPalette(palette);
  rubberBand->resize(30, 30);
  rubberBand->show();
  
  //scene->addItem(rubberBand);
  
  this->graphicsView->setScene(scene);
  this->graphicsView->setDragMode(QGraphicsView::RubberBandDrag);
  this->graphicsView->show();
  
  //QGraphicsScene::selectionArea().boundingRect()
}

void Form::SelectionChanged()
{
  std::cout << "Selection changed." << std::endl;
}
