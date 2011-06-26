#ifndef __itkImageFilter_txx
#define __itkImageFilter_txx

#include "ImageFilter.h"
#include "itkObjectFactory.h"
#include "itkImageRegionIterator.h"
#include "itkImageRegionConstIterator.h"

namespace itk
{

template< class TImage>
void ImageFilter< TImage>
::GenerateData()
{
  double a = 2.1;
  itkExceptionMacro ("Here is a variable: " << a << " and then more text.");
}

}// end namespace


#endif