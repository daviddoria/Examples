#ifndef QuickView_H
#define QuickView_H

#include <vector>

#include <itkImage.h>

class QuickView
{
public:
  void AddImage(itk::Image<unsigned char, 2>::Pointer  image);
  void Visualize();

private:
  std::vector<itk::Image<unsigned char, 2>::Pointer > Images;
};

#endif
