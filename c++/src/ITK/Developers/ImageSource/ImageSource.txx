#ifndef __itkImageFilter_txx
#define __itkImageFilter_txx

#include "ImageFilter.h"

#include "itkObjectFactory.h"

namespace itk
{

template< class TOutputImage>
void ImageFilter< TOutputImage>
::GenerateData()
{
  typename TOutputImage::Pointer output = this->GetOutput();
  typename TOutputImage::RegionType region;
  typename TOutputImage::IndexType start;
  start[0] = 0;
  start[1] = 0;

  typename TOutputImage::SizeType size;
  size[0] = 200;
  size[1] = 300;

  region.SetSize(size);
  region.SetIndex(start);

  output->SetRegions(region);
  output->Allocate();

  itk::ImageRegionIterator<TOutputImage> imageIterator(output,output->GetLargestPossibleRegion());

  while(!imageIterator.IsAtEnd())
    {
    if(imageIterator.GetIndex()[0] == imageIterator.GetIndex()[1])
      {
      imageIterator.Set(255);
      }
    else
      {
      imageIterator.Set(0);
      }

    ++imageIterator;
    }

}

}// end namespace


#endif