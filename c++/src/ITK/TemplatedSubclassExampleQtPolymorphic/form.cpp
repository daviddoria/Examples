#include "form.h"

#include <iostream>

#include "itkImage.h"

MyForm::MyForm(QWidget *parent)
    : QWidget(parent)
{
  setupUi(this);
  connect( this->btnCompute, SIGNAL( clicked() ), this, SLOT(btnCompute_clicked()) );

//  this->Segmentation = new Segmentation<itk::Image<unsigned char, 2> >();
}

void MyForm::btnCompute_clicked()
{
  if(this->radGrayscale->isChecked())
    {
    std::cout << "Grayscale" << std::endl;
    }

  if(this->radRGB->isChecked())
    {
    std::cout << "RGB" << std::endl;
    }
}
