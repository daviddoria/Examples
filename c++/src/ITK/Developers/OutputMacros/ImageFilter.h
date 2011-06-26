#ifndef __itkImageFilter_h
#define __itkImageFilter_h

#include "itkImageToImageFilter.h"

#include <set>

namespace itk
{
template< class TImage>
class ImageFilter:public ImageToImageFilter< TImage, TImage >
{
public:

  /** Standard class typedefs. */
  typedef ImageFilter             Self;
  typedef ImageToImageFilter< TImage, TImage > Superclass;
  typedef SmartPointer< Self >        Pointer;

  /** Method for creation through the object factory. */
  itkNewMacro(Self);

  /** Run-time type information (and related methods). */
  itkTypeMacro(ImageFilter, ImageToImageFilter);

  itkSetMacro( Variable, double );
  itkGetMacro( Variable, double);

protected:
  ImageFilter(){}
  ~ImageFilter(){}

  /** Does the real work. */
  virtual void GenerateData();

  double m_Variable;

private:
  ImageFilter(const Self &); //purposely not implemented
  void operator=(const Self &);  //purposely not implemented

};
} //namespace ITK


#ifndef ITK_MANUAL_INSTANTIATION
#include "ImageFilter.txx"
#endif


#endif // __itkImageFilter_h