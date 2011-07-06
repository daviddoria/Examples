#include "itkBinaryImageToLabelMapFilter.h"
#include "itkImage.h"
#include "itkImageFileWriter.h"
#include "itkImageRegionIterator.h"
#include "itkLabelMapToLabelImageFilter.h"
#include "itkLabelMapOverlayImageFilter.h"
#include "itkRGBPixel.h"
 
typedef itk::Image<unsigned char, 2>  ImageType;
void CreateImage(ImageType::Pointer image);
 
int main(int, char *[])
{
  ImageType::Pointer image = ImageType::New();
  CreateImage(image);
 
  typedef itk::BinaryImageToLabelMapFilter<ImageType> BinaryImageToLabelMapFilterType;
  BinaryImageToLabelMapFilterType::Pointer binaryImageToLabelMapFilter = BinaryImageToLabelMapFilterType::New();
  binaryImageToLabelMapFilter->SetInput(image);
  binaryImageToLabelMapFilter->Update();
 
  typedef itk::RGBPixel<unsigned char> RGBPixelType;
  typedef itk::Image<RGBPixelType> RGBImageType;
 
  typedef itk::LabelMapOverlayImageFilter<BinaryImageToLabelMapFilterType::OutputImageType, ImageType, RGBImageType>
                                       LabelMapOverlayImageFilterType;
  LabelMapOverlayImageFilterType::Pointer labelMapOverlayImageFilter = LabelMapOverlayImageFilterType::New();
  labelMapOverlayImageFilter->SetInput(binaryImageToLabelMapFilter->GetOutput());
  labelMapOverlayImageFilter->SetFeatureImage(image);
  labelMapOverlayImageFilter->SetOpacity(.5);
  labelMapOverlayImageFilter->Update();
 
  typedef  itk::ImageFileWriter< RGBImageType  > WriterType;
  WriterType::Pointer writer = WriterType::New();
  writer->SetFileName("output.png");
  writer->SetInput(labelMapOverlayImageFilter->GetOutput());
  writer->Update();
 
  return EXIT_SUCCESS;
}
 
void CreateImage(ImageType::Pointer image)
{
  // Create a black image with a white square
  ImageType::IndexType start;
  start.Fill(0);
 
  ImageType::SizeType size;
  size.Fill(100);
 
  ImageType::RegionType region;
  region.SetSize(size);
  region.SetIndex(start);
  image->SetRegions(region);
  image->Allocate();
  image->FillBuffer(0);
 
  itk::ImageRegionIterator<ImageType> imageIterator(image,image->GetLargestPossibleRegion());
 
  // Make two squares
  while(!imageIterator.IsAtEnd())
    {
    if((imageIterator.GetIndex()[0] > 5 && imageIterator.GetIndex()[0] < 20) &&
      (imageIterator.GetIndex()[1] > 5 && imageIterator.GetIndex()[1] < 20) )
        {
        imageIterator.Set(255);
        }
 
    if((imageIterator.GetIndex()[0] > 50 && imageIterator.GetIndex()[0] < 70) &&
      (imageIterator.GetIndex()[1] > 50 && imageIterator.GetIndex()[1] < 70) )
        {
        imageIterator.Set(255);
        }
    ++imageIterator;
    }
 
  typedef  itk::ImageFileWriter< ImageType  > WriterType;
  WriterType::Pointer writer = WriterType::New();
  writer->SetFileName("image.png");
  writer->SetInput(image);
  writer->Update();
}
