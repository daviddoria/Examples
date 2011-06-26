#ifndef __itkImageFilterMultipleOutputs_txx
#define __itkImageFilterMultipleOutputs_txx

#include "ImageFilterMultipleOutputs.h"

#include "itkObjectFactory.h"
#include "itkImageRegionIterator.h"
#include "itkImageRegionConstIterator.h"

namespace itk
{

template< class TImage>
ImageFilterMultipleOutputs<TImage>::ImageFilterMultipleOutputs()
{
  this->SetNumberOfRequiredOutputs(2);
  this->SetNumberOfRequiredInputs(0);

  this->SetNthOutput( 0, this->MakeOutput(0) );
  this->SetNthOutput( 1, this->MakeOutput(1) );
}

template< class TImage>
void ImageFilterMultipleOutputs<TImage>::GenerateData()
{
  typename TImage::IndexType start;
  start[0] = 0;
  start[1] = 0;

  typename TImage::SizeType size;
  size[0] = 20;
  size[1] = 20;

  typename TImage::RegionType region;
  region.SetSize(size);
  region.SetIndex(start);
  
  // Setup output 1
  typename TImage::Pointer output1 = this->GetOutput1();
  output1->SetRegions(region);
  output1->Allocate();  
  
  itk::ImageRegionIterator<TImage> outputIterator1(output1, output1->GetLargestPossibleRegion());
  outputIterator1.GoToBegin();
  
  while(!outputIterator1.IsAtEnd())
    {
    if(outputIterator1.GetIndex()[0] == outputIterator1.GetIndex()[1])
      {
      outputIterator1.Set(255);
      }
    else
      {
      outputIterator1.Set(0);
      }

    ++outputIterator1;
    }

  // Setup output 2
  typename TImage::Pointer output2 = this->GetOutput2();
  output2->SetRegions(region);
  output2->Allocate();

  itk::ImageRegionIterator<TImage> outputIterator2(output2, output2->GetLargestPossibleRegion());
  outputIterator2.GoToBegin();

  while(!outputIterator2.IsAtEnd())
    {
    if(outputIterator2.GetIndex()[0] > 10)
      {
      outputIterator2.Set(255);
      }
    else
      {
      outputIterator2.Set(0);
      }

    ++outputIterator2;
    }

}

template< class TImage>
DataObject::Pointer ImageFilterMultipleOutputs<TImage>::MakeOutput(unsigned int idx)
{
  DataObject::Pointer output;

  switch ( idx )
    {
    case 0:
      output = ( TImage::New() ).GetPointer();
      break;
    case 1:
      output = ( TImage::New() ).GetPointer();
      break;
    default:
      std::cerr << "No output " << idx << std::endl;
      output = NULL;
      break;
    }
  return output.GetPointer();
}

template< class TImage>
TImage* ImageFilterMultipleOutputs<TImage>::GetOutput1()
{
  return dynamic_cast< TImage * >(
           this->ProcessObject::GetOutput(0) );
}

template< class TImage>
TImage* ImageFilterMultipleOutputs<TImage>::GetOutput2()
{
  return dynamic_cast< TImage * >(
           this->ProcessObject::GetOutput(1) );
}

}// end namespace


#endif