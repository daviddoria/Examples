#include <QtGui>
#include <QImage>

#include "form.h"

#include <iostream>

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
    
  // This works
  /*
  QImage image;
  image.load(fileName);
  this->label->setPixmap(QPixmap::fromImage(image));
  this->label->setScaledContents(true); // This forces the image to fit into the label
  */
  
  // This is slightly easier
  QPixmap image;
  image.load(fileName);
  this->label->setPixmap(image);
  this->label->setScaledContents(true); // This forces the image to fit into the label
}
