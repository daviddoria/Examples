#include "itkImage.h"

#include <iostream>

class SegmentationBase
{

};

template<typename T>
class Segmentation : public SegmentationBase
{
public:
  T GetImage();
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
  // A = myForm.MySegmentation->GetImage();

  return 0;
}