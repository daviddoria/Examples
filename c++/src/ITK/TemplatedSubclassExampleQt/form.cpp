#include "form.h"

#include <iostream>

#include "itkImage.h"
#include "itkCovariantVector.h"

MyForm::MyForm(QWidget *parent)
    : QWidget(parent)
{
  setupUi(this);
  connect( this->btnCompute, SIGNAL( clicked() ), this, SLOT(btnCompute_clicked()) );
}

void MyForm::btnCompute_clicked()
{
  if(this->radGrayscale->isChecked())
    {
    std::cout << "Grayscale" << std::endl;
    //this->Segmentation = new Segmentation<itk::Image<itk::CovariantVector<float, 1>, 2> >();
    }

  if(this->radRGB->isChecked())
    {
    std::cout << "RGB" << std::endl;
    //this->Segmentation = new Segmentation<itk::Image<itk::CovariantVector<float, 3>, 2> >();
    }
}
