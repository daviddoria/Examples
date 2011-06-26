#ifndef __itkImageFilter_h
#define __itkImageFilter_h

#include "itkImageSource.h"

namespace itk
{
template< class TOutputImage>
class ImageFilter:public ImageSource< TOutputImage >
{
public:
  /** Standard class typedefs. */
  typedef ImageFilter             Self;
  typedef ImageSource< TOutputImage > Superclass;
  typedef SmartPointer< Self >        Pointer;

  /** Method for creation through the object factory. */
  itkNewMacro(Self);

  /** Run-time type information (and related methods). */
  itkTypeMacro(ImageFilter, ImageSource);

protected:
  ImageFilter(){}
  ~ImageFilter(){}

  /** Does the real work. */
  virtual void GenerateData();

private:
  ImageFilter(const Self &); //purposely not implemented
  void operator=(const Self &);  //purposely not implemented

};
} //namespace ITK


#ifndef ITK_MANUAL_INSTANTIATION
#include "ImageFilter.txx"
#endif


#endif // __itkImageFilter_h