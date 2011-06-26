#ifndef __itkImageFilterMultipleInputs_txx
#define __itkImageFilterMultipleInputs_txx

#include "ImageFilterMultipleInputsDifferentType.h"

#include "itkObjectFactory.h"
#include "itkImageRegionIterator.h"
#include "itkImageRegionConstIterator.h"

namespace itk
{

template< typename TImage, typename TMask>
ImageFilterMultipleInputsDifferentType<TImage, TMask>::ImageFilterMultipleInputsDifferentType()
{
  this->SetNumberOfRequiredInputs(2);
}

template< typename TImage, typename TMask>
void ImageFilterMultipleInputsDifferentType<TImage, TMask>::SetInputImage(const TImage* image)
{
  SetNthInput(0, const_cast<TImage*>(image));
}

template< typename TImage, typename TMask>
void ImageFilterMultipleInputsDifferentType<TImage, TMask>::SetInputMask(const TMask* mask)
{
  SetNthInput(1, const_cast<TMask*>(mask));
}

template< typename TImage, typename TMask>
typename TImage::ConstPointer ImageFilterMultipleInputsDifferentType<TImage, TMask>::GetInputImage()
{
  return static_cast< const TImage * >
         ( this->ProcessObject::GetInput(0) );
}

template< typename TImage, typename TMask>
typename TMask::ConstPointer ImageFilterMultipleInputsDifferentType<TImage, TMask>::GetInputMask()
{
  return static_cast< const TMask * >
         ( this->ProcessObject::GetInput(1) );
}

template< typename TImage, typename TMask>
void ImageFilterMultipleInputsDifferentType<TImage, TMask>::GenerateData()
{
  typename TImage::ConstPointer input = this->GetInputImage();
  typename TMask::ConstPointer mask = this->GetInputMask();

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