#include "itkVectorImage.h"
#include "itkMaskImageFilter.h"
#include "itkImageFileWriter.h"

typedef itk::VectorImage<unsigned char, 2>  VectorImageType;
typedef itk::Image<unsigned char, 2>  MaskImageType;

int main(int argc, char *argv[])
{
  itk::Index<2> start;
  start.Fill(0);

  itk::Size<2> size;
  size.Fill(3);

  itk::ImageRegion<2> region(start, size);

  itk::Index<2> center;
  center[0] = 1;
  center[1] = 1;
  
  // Create a vector image
  VectorImageType::Pointer vectorImage = VectorImageType::New();
  vectorImage->SetRegions(region);
  vectorImage->SetVectorLength(2);
  vectorImage->Allocate();
  itk::VariableLengthVector<unsigned char> nonZeroVector;
  nonZeroVector.SetSize(2);
  nonZeroVector[0] = 1;
  nonZeroVector[1] = 2;
  vectorImage->FillBuffer(nonZeroVector);

  {
  typedef  itk::ImageFileWriter< VectorImageType  > WriterType;
  WriterType::Pointer writer = WriterType::New();
  writer->SetFileName("input.mhd");
  writer->SetInput(vectorImage);
  writer->Update();
  }
  
  // Create a mask
  MaskImageType::Pointer maskImage = MaskImageType::New();
  maskImage->SetRegions(region);
  maskImage->Allocate();
  maskImage->FillBuffer(0);
  maskImage->SetPixel(center, 1);

  {
  typedef  itk::ImageFileWriter< MaskImageType  > WriterType;
  WriterType::Pointer writer = WriterType::New();
  writer->SetFileName("mask.mhd");
  writer->SetInput(maskImage);
  writer->Update();
  }
  
  typedef itk::MaskImageFilter< VectorImageType, MaskImageType > MaskFilterType;
  MaskFilterType::Pointer maskFilter = MaskFilterType::New();
  maskFilter->SetInput(vectorImage);
  maskFilter->SetMaskImage(maskImage);
  maskFilter->SetOutsideValue(itk::NumericTraits<VectorImageType::PixelType>::ZeroValue());
  maskFilter->Update();

  {
  typedef  itk::ImageFileWriter< VectorImageType  > WriterType;
  WriterType::Pointer writer = WriterType::New();
  writer->SetFileName("output.mhd");
  writer->SetInput(maskFilter->GetOutput());
  writer->Update();
  }
  return EXIT_SUCCESS;
}
