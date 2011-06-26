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
  itkDebugMacro("Doing something...");
  itkWarningMacro("Some warning.");
  //itkErrorMacro("Some error!");
}

}// end namespace


#endif