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
}

void Form::on_btnOpen_clicked()
{
  QString fileName = QFileDialog::getOpenFileName(this, "Open File", ".", "Image Files (*.jpg *.jpeg *.bmp *.png)");

  std::cout << "Got filename: " << fileName.toStdString() << std::endl;
  if(fileName.toStdString().empty())
    {
    std::cout << "Filename was empty." << std::endl;
    return;
    }

  QPixmap image;
  image.load(fileName);  

  QPixmap image2;
  image2.load(fileName);  
  
  QGraphicsScene* scene = new QGraphicsScene();
  
  QGraphicsPixmapItem * image1actor = scene->addPixmap(image);
  QGraphicsPixmapItem * image2actor = scene->addPixmap(image2);
  
  image2actor->setOffset(500,500);  
  
  this->graphicsView->setScene(scene);

}
