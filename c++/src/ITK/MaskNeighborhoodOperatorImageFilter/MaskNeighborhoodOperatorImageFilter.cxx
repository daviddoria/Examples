#include "itkImage.h"
#include "itkImageFileWriter.h"
#include "itkMaskImageFilter.h"
#include "itkRescaleIntensityImageFilter.h"
#include "itkMaskNeighborhoodOperatorImageFilter.h"
#include "itkSobelOperator.h"

typedef itk::Image<unsigned char, 2>  UnsignedCharImageType;
typedef itk::Image<float, 2>  FloatImageType;

void CreateImage(UnsignedCharImageType::Pointer image);
void CreateHalfMask(UnsignedCharImageType::Pointer image, UnsignedCharImageType::Pointer mask);

int main(int argc, char *argv[])
{
  UnsignedCharImageType::Pointer image = UnsignedCharImageType::New();
  CreateImage(image);

  UnsignedCharImageType::Pointer mask = UnsignedCharImageType::New();
  CreateHalfMask(image, mask);

  typedef itk::SobelOperator<float, 2> SobelOperatorType;
  SobelOperatorType sobelOperator;
  itk::Size<2> radius;
  radius.Fill(1); // a radius of 1x1 creates a 3x3 operator
  sobelOperator.SetDirection(0); // Create the operator for the X axis derivative
  sobelOperator.CreateToRadius(radius);

  // Visualize mask image
  typedef itk::MaskNeighborhoodOperatorImageFilter< UnsignedCharImageType, UnsignedCharImageType, FloatImageType, float> MaskNeighborhoodOperatorImageFilterType;
  MaskNeighborhoodOperatorImageFilterType::Pointer maskNeighborhoodOperatorImageFilter = 
    MaskNeighborhoodOperatorImageFilterType::New();
  maskNeighborhoodOperatorImageFilter->SetInput(image);
  maskNeighborhoodOperatorImageFilter->SetMaskImage(mask);
  maskNeighborhoodOperatorImageFilter->SetOperator(sobelOperator);
  maskNeighborhoodOperatorImageFilter->Update();
  
  typedef itk::RescaleIntensityImageFilter< FloatImageType, UnsignedCharImageType > RescaleFilterType;
  RescaleFilterType::Pointer rescaleFilter = RescaleFilterType::New();
  rescaleFilter->SetInput(maskNeighborhoodOperatorImageFilter->GetOutput());
  rescaleFilter->Update();

  typedef  itk::ImageFileWriter< UnsignedCharImageType  > WriterType;
  WriterType::Pointer writer = WriterType::New();
  writer->SetFileName("output.png");
  writer->SetInput(rescaleFilter->GetOutput());
  writer->Update();

  return EXIT_SUCCESS;
}


void CreateHalfMask(UnsignedCharImageType::Pointer image, UnsignedCharImageType::Pointer mask)
{
  itk::ImageRegion<2> region = image->GetLargestPossibleRegion();

  mask->SetRegions(region);
  mask->Allocate();
  mask->FillBuffer(0);

  itk::Size<2> regionSize = region.GetSize();

  itk::ImageRegionIterator<UnsignedCharImageType> imageIterator(mask,region);

  // Make the left half of the mask white and the right half black
  while(!imageIterator.IsAtEnd())
  {
    if(imageIterator.GetIndex()[0] > regionSize[0] / 2)
        {
        imageIterator.Set(0);
        }
      else
        {
        imageIterator.Set(1);
        }

    ++imageIterator;
  }

  typedef itk::RescaleIntensityImageFilter< UnsignedCharImageType, UnsignedCharImageType > RescaleFilterType;
  RescaleFilterType::Pointer rescaleFilter = RescaleFilterType::New();
  rescaleFilter->SetInput(mask);
  rescaleFilter->Update();
  
  typedef  itk::ImageFileWriter< UnsignedCharImageType  > WriterType;
  WriterType::Pointer writer = WriterType::New();
  writer->SetFileName("mask.png");
  writer->SetInput(rescaleFilter->GetOutput());
  writer->Update();

}


void CreateImage(UnsignedCharImageType::Pointer image)
{
  itk::Index<2> start;
  start.Fill(0);

  itk::Size<2> size;
  size.Fill(100);

  itk::ImageRegion<2> region(start,size);

  image->SetRegions(region);
  image->Allocate();
  image->FillBuffer(0);

  // Make a square
  for(unsigned int r = 30; r < 70; r++)
    {
    for(unsigned int c = 30; c < 70; c++)
      {
      FloatImageType::IndexType pixelIndex;
      pixelIndex[0] = r;
      pixelIndex[1] = c;

      image->SetPixel(pixelIndex, 255);
      }
    }
    
  typedef  itk::ImageFileWriter< UnsignedCharImageType  > WriterType;
  WriterType::Pointer writer = WriterType::New();
  writer->SetFileName("input.png");
  writer->SetInput(image);
  writer->Update();

}
