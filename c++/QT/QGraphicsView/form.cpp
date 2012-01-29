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
  
  QGraphicsScene* scene = new QGraphicsScene();
  
  //scene->addSimpleText("text");
  
  scene->addPixmap(image);
  
  this->graphicsView->setScene(scene);
  this->graphicsView->show();

}

// Complicated text:
//QFont font("MS Serif", -1, -1, true);
//QFontInfo info(font);
//scene->addText("text", font);
  