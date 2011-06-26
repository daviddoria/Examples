#include "itkImage.h"
#include "itkHConcaveImageFilter.h"
#include "itkImageRegionIterator.h"
#include "itkImageFileWriter.h"

typedef itk::Image<unsigned char, 2>  ImageType;
static void CreateImage(ImageType::Pointer image);

int main(int, char *[])
{
  ImageType::Pointer image = ImageType::New();
  CreateImage(image);

  typedef itk::HConcaveImageFilter<ImageType, ImageType> HConcaveImageFilterType;
  HConcaveImageFilterType::Pointer hConcaveImageFilter = HConcaveImageFilterType::New();
  hConcaveImageFilter->SetInput(image);
  hConcaveImageFilter->SetHeight(1);
  hConcaveImageFilter->Update();

  typedef  itk::ImageFileWriter< ImageType  > WriterType;
  WriterType::Pointer writer = WriterType::New();
  writer->SetFileName("output.png");
  writer->SetInput(hConcaveImageFilter->GetOutput());
  writer->Update();

  return EXIT_SUCCESS;
}

void CreateImage(ImageType::Pointer image)
{
  ImageType::IndexType start;
  start.Fill(0);

  ImageType::SizeType size;
  size.Fill(100);

  ImageType::RegionType region(start,size);

  image->SetRegions(region);
  image->Allocate();

  itk::ImageRegionIterator<ImageType> imageIterator(image,region);

  while(!imageIterator.IsAtEnd())
    {
    if(imageIterator.GetIndex()[0] < 70)
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
