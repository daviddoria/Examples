#include "itkImage.h"
#include "itkSimpleFilterWatcher.h"
#include "itkOrImageFilter.h"
#include "itkImageRegionIterator.h"
#include "itkImageFileWriter.h"

typedef itk::Image<unsigned char, 2>  ImageType;
static void CreateImage1(ImageType::Pointer image);
static void CreateImage2(ImageType::Pointer image);

int main(int, char *[])
{
  ImageType::Pointer image1 = ImageType::New();
  CreateImage1(image1);

  typedef  itk::ImageFileWriter< ImageType  > WriterType;
  WriterType::Pointer writer = WriterType::New();
  writer->SetFileName("input1.png");
  writer->SetInput(image1);
  writer->Update();

  ImageType::Pointer image2 = ImageType::New();
  CreateImage2(image2);

  writer->SetFileName("input2.png");
  writer->SetInput(image2);
  writer->Update();

  typedef itk::OrImageFilter <ImageType> OrImageFilterType;
  OrImageFilterType::Pointer orFilter = OrImageFilterType::New();
  //orFilter->SetInput(0, image1);
  //orFilter->SetInput(1, image2);
  orFilter->SetInput1(image1);
  orFilter->SetInput2(image2);
  orFilter->Update();

  writer->SetFileName("output.png");
  writer->SetInput(orFilter->GetOutput());
  writer->Update();

  return EXIT_SUCCESS;
}

void CreateImage1(ImageType::Pointer image)
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

void CreateImage2(ImageType::Pointer image)
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
    if(imageIterator.GetIndex()[0] > 30)
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
