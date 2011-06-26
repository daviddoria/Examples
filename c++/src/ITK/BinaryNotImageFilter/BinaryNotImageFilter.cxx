#include "itkImage.h"
#include "itkSimpleFilterWatcher.h"
#include "itkBinaryNotImageFilter.h"
#include "itkImageRegionIterator.h"
#include "itkImageFileWriter.h"

typedef itk::Image<unsigned char, 2>  ImageType;
static void CreateImage(ImageType::Pointer image);

int main(int, char *[])
{
  ImageType::Pointer image = ImageType::New();
  CreateImage(image);

  typedef  itk::ImageFileWriter< ImageType  > WriterType;
  WriterType::Pointer writer = WriterType::New();
  writer->SetFileName("input.png");
  writer->SetInput(image);
  writer->Update();

  typedef itk::BinaryNotImageFilter <ImageType>
          BinaryNotImageFilterType;

  BinaryNotImageFilterType::Pointer binaryNotFilter
          = BinaryNotImageFilterType::New();
  binaryNotFilter->SetInput(image);
  binaryNotFilter->Update();

  writer->SetFileName("output.png");
  writer->SetInput(binaryNotFilter->GetOutput());
  writer->Update();

  return EXIT_SUCCESS;
}
void CreateImage(ImageType::Pointer image)
{
  ImageType::IndexType start;
  start.Fill(0);

  ImageType::SizeType size;
  size.Fill(100);

  ImageType::RegionType region;
  region.SetSize(size);
  region.SetIndex(start);

  image->SetRegions(region);
  image->Allocate();

  itk::ImageRegionIterator<ImageType> imageIterator(image,region);

  while(!imageIterator.IsAtEnd())
    {
    if(imageIterator.GetIndex()[0] > 50)
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
