#include "itkImage.h"
#include "itkImageFileWriter.h"
#include "itkMaskNegatedImageFilter.h"

typedef itk::Image<unsigned char, 2>  ImageType;

static void CreateHalfMask(ImageType::Pointer image, ImageType::Pointer mask);
static void CreateImage(ImageType::Pointer image);

int main(int, char *[])
{
  ImageType::Pointer image = ImageType::New();
  CreateImage(image);

  ImageType::Pointer mask = ImageType::New();
  CreateHalfMask(image, mask);

  typedef itk::MaskNegatedImageFilter< ImageType, ImageType > MaskNegatedImageFilterType;
  MaskNegatedImageFilterType::Pointer maskNegatedImageFilter = MaskNegatedImageFilterType::New();
  maskNegatedImageFilter->SetInput(image);
  maskNegatedImageFilter->SetMaskImage(mask);
  maskNegatedImageFilter->Update();;
  
  typedef itk::ImageFileWriter<ImageType> FileWriterType;
  FileWriterType::Pointer writer = FileWriterType::New();
  writer->SetFileName("output.png");
  writer->SetInput(maskNegatedImageFilter->GetOutput());
  writer->Update();
  
  return EXIT_SUCCESS;
}


void CreateHalfMask(ImageType::Pointer image, ImageType::Pointer mask)
{
  ImageType::RegionType region = image->GetLargestPossibleRegion();

  mask->SetRegions(region);
  mask->Allocate();

  ImageType::SizeType regionSize = region.GetSize();

  itk::ImageRegionIterator<ImageType> imageIterator(mask,region);

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
  image->FillBuffer(122);
}
