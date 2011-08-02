#include "itkImage.h"
#include "itkImageFileWriter.h"
#include "itkImageRegionIterator.h"
#include "itkBinaryImageToLabelMapFilter.h"
#include "itkLabelMapToLabelImageFilter.h"
#include "itkLabelGeometryImageFilter.h"

typedef itk::Image<unsigned char, 2>  ImageType;
static void CreateImage(ImageType::Pointer image);

int main(int, char *[])
{
  ImageType::Pointer image = ImageType::New();
  CreateImage(image);

  typedef itk::BinaryImageToLabelMapFilter<ImageType> BinaryImageToLabelMapFilterType;
  BinaryImageToLabelMapFilterType::Pointer binaryImageToLabelMapFilter = BinaryImageToLabelMapFilterType::New();
  binaryImageToLabelMapFilter->SetInput(image);
  binaryImageToLabelMapFilter->Update();

  typedef itk::LabelMapToLabelImageFilter<BinaryImageToLabelMapFilterType::OutputImageType, ImageType> LabelMapToLabelImageFilterType;
  LabelMapToLabelImageFilterType::Pointer labelMapToLabelImageFilter = LabelMapToLabelImageFilterType::New();
  labelMapToLabelImageFilter->SetInput(binaryImageToLabelMapFilter->GetOutput());
  labelMapToLabelImageFilter->Update();

  typedef itk::LabelGeometryImageFilter< ImageType > LabelGeometryImageFilterType;
  LabelGeometryImageFilterType::Pointer labelGeometryImageFilter = LabelGeometryImageFilterType::New();
  labelGeometryImageFilter->SetInput( labelMapToLabelImageFilter->GetOutput() );

  // These generate optional outputs.
  labelGeometryImageFilter->CalculatePixelIndicesOn();
  labelGeometryImageFilter->CalculateOrientedBoundingBoxOn();
  labelGeometryImageFilter->CalculateOrientedLabelRegionsOn();
  
  labelGeometryImageFilter->Update();
  
  LabelGeometryImageFilterType::LabelsType allLabels = labelGeometryImageFilter->GetLabels();
  LabelGeometryImageFilterType::LabelsType::iterator allLabelsIt;
  std::cout << "Number of labels: " << labelGeometryImageFilter->GetNumberOfLabels() << std::endl;
  std::cout << std::endl;

  for( allLabelsIt = allLabels.begin(); allLabelsIt != allLabels.end(); allLabelsIt++ )
    {
    LabelGeometryImageFilterType::LabelPixelType labelValue = *allLabelsIt;
    std::cout << "\tVolume: " << labelGeometryImageFilter->GetVolume(labelValue) << std::endl;
    std::cout << "\tIntegrated Intensity: " << labelGeometryImageFilter->GetIntegratedIntensity(labelValue) << std::endl;
    std::cout << "\tCentroid: " << labelGeometryImageFilter->GetCentroid(labelValue) << std::endl;
    std::cout << "\tWeighted Centroid: " << labelGeometryImageFilter->GetWeightedCentroid(labelValue) << std::endl;
    std::cout << "\tAxes Length: " << labelGeometryImageFilter->GetAxesLength(labelValue) << std::endl;
    std::cout << "\tMajorAxisLength: " << labelGeometryImageFilter->GetMajorAxisLength(labelValue) << std::endl;
    std::cout << "\tMinorAxisLength: " << labelGeometryImageFilter->GetMinorAxisLength(labelValue) << std::endl;
    std::cout << "\tEccentricity: " << labelGeometryImageFilter->GetEccentricity(labelValue) << std::endl;
    std::cout << "\tElongation: " << labelGeometryImageFilter->GetElongation(labelValue) << std::endl;
    std::cout << "\tOrientation: " << labelGeometryImageFilter->GetOrientation(labelValue) << std::endl;
    std::cout << "\tBounding box: " << labelGeometryImageFilter->GetBoundingBox(labelValue) << std::endl;
    
    std::cout << std::endl << std::endl;
    
    }

  return EXIT_SUCCESS;
}

void CreateImage(ImageType::Pointer image)
{
  // Create a black image with a white square
  ImageType::IndexType start;
  start.Fill(0);

  ImageType::SizeType size;
  size.Fill(20);

  ImageType::RegionType region;
  region.SetSize(size);
  region.SetIndex(start);
  image->SetRegions(region);
  image->Allocate();

  itk::ImageRegionIterator<ImageType> imageIterator(image,image->GetLargestPossibleRegion());

  // Make a square
  while(!imageIterator.IsAtEnd())
    {
    if((imageIterator.GetIndex()[0] > 5 && imageIterator.GetIndex()[0] < 10) &&
      (imageIterator.GetIndex()[1] > 5 && imageIterator.GetIndex()[1] < 10) )
        {
        imageIterator.Set(255);
        }
      else
        {
        imageIterator.Set(0);
        }

    ++imageIterator;
    }

  typedef  itk::ImageFileWriter< ImageType  > WriterType;
  WriterType::Pointer writer = WriterType::New();
  writer->SetFileName("image.png");
  writer->SetInput(image);
  writer->Update();
}
