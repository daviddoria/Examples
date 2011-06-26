#ifndef SEGMENTATION_H
#define SEGMENTATION_H

template<typename T>
class Segmentation
{
public:

  typename T::Pointer Image;

  typename T::Pointer GetImage()
  {
    return this->Image;
  }
};

#endif