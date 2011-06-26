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
  typename TImage::ConstPointer input = this->GetInput();
  
  typename TImage::Pointer output = this->GetOutput();
  output->SetRegions(input->GetLargestPossibleRegion());
  output->Allocate();  
  
  itk::ImageRegionIterator<TImage> outputIterator(output, output->GetLargestPossibleRegion());
  itk::ImageRegionConstIterator<TImage> inputIterator(input, input->GetLargestPossibleRegion());

  while(!outputIterator.IsAtEnd())
    {
    if(inputIterator.GetIndex()[0] == inputIterator.GetIndex()[1])
      {
      outputIterator.Set(255);
      }
    else
      {
      outputIterator.Set(inputIterator.Get());
      }

    ++inputIterator;
    ++outputIterator;
    }

}

}// end namespace


#endif