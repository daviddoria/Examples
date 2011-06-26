#include "itkImage.h"

#include <iostream>

class SegmentationBase
{
public:
  virtual itk::DataObject::Pointer GetImage() = 0;
};

template<typename T>
class Segmentation : public SegmentationBase
{
public:
  typename T::Pointer Image;

  itk::DataObject::Pointer GetImage()
  //typename T::Pointer GetImage()
  {
    return this->Image;
  }
};

class Form
{
public:
  SegmentationBase* MySegmentation;
  void Init()
  {
    this->MySegmentation = new Segmentation<itk::Image<unsigned char, 2> >();
  }
};

int main(int argc, char *argv[])
{
  Form myForm;
  myForm.Init();

  // How do I specify the type of A?
  myForm.MySegmentation->GetImage();

  return 0;
}