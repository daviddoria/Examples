#include "itkImage.h"
#include "itkImageFileWriter.h"
#include "itkImageRegionIterator.h"
#include "itkBinaryImageToShapeLabelMapFilter.h"
#include "itkShapeOpeningLabelMapFilter.h"
#include "itkLabelMapToLabelImageFilter.h"
#include "itkScalarToRGBColormapImageFilter.h"

typedef itk::Image<unsigned char, 2>  ImageType;
typedef itk::Image<unsigned char, 2>  LabelImageType;
static void CreateImage(ImageType::Pointer image);

int main(int, char *[])
{
  ImageType::Pointer image = ImageType::New();
  CreateImage(image);

  // Create a ShapeLabelMap from the image
  typedef itk::BinaryImageToShapeLabelMapFilter<ImageType> BinaryImageToShapeLabelMapFilterType;
  BinaryImageToShapeLabelMapFilterType::Pointer binaryImageToShapeLabelMapFilter = BinaryImageToShapeLabelMapFilterType::New();
  binaryImageToShapeLabelMapFilter->SetInput(image);
  binaryImageToShapeLabelMapFilter->Update();

  // Remove label objects that have PERIMETER less than 50
  typedef itk::ShapeOpeningLabelMapFilter< BinaryImageToShapeLabelMapFilterType::OutputImageType > ShapeOpeningLabelMapFilterType;
  ShapeOpeningLabelMapFilterType::Pointer shapeOpeningLabelMapFilter = ShapeOpeningLabelMapFilterType::New();
  shapeOpeningLabelMapFilter->SetInput( binaryImageToShapeLabelMapFilter->GetOutput() );
  shapeOpeningLabelMapFilter->SetLambda( 50 );
  shapeOpeningLabelMapFilter->ReverseOrderingOn();
  shapeOpeningLabelMapFilter->SetAttribute( ShapeOpeningLabelMapFilterType::LabelObjectType::PERIMETER);
  shapeOpeningLabelMapFilter->Update();
 
  // Create a label image
  typedef itk::LabelMapToLabelImageFilter<BinaryImageToShapeLabelMapFilterType::OutputImageType, LabelImageType> LabelMapToLabelImageFilterType;
  LabelMapToLabelImageFilterType::Pointer labelMapToLabelImageFilter = LabelMapToLabelImageFilterType::New();
  labelMapToLabelImageFilter->SetInput(shapeOpeningLabelMapFilter->GetOutput());
  labelMapToLabelImageFilter->Update();
  
  typedef itk::RGBPixel<unsigned char>   RGBPixelType;
  typedef itk::Image<RGBPixelType, 2>    RGBImageType;

  // Color each label/object a different color
  typedef itk::ScalarToRGBColormapImageFilter<LabelImageType, RGBImageType> RGBFilterType;
  RGBFilterType::Pointer colormapImageFilter = RGBFilterType::New();
  colormapImageFilter->SetInput(labelMapToLabelImageFilter->GetOutput());
  colormapImageFilter->SetColormap( RGBFilterType::Jet );
  colormapImageFilter->Update();
  
  // Write the output
  typedef itk::ImageFileWriter< RGBImageType > WriterType;
  WriterType::Pointer writer = WriterType::New();
  writer->SetInput( colormapImageFilter->GetOutput() );
  writer->SetFileName("output.png");
  writer->Update();

  return EXIT_SUCCESS;
}

void CreateImage(ImageType::Pointer image)
{
  // Create a black image with three white squares
  ImageType::IndexType start;
  start.Fill(0);

  ImageType::SizeType size;
  size.Fill(200);

  ImageType::RegionType region;
  region.SetSize(size);
  region.SetIndex(start);
  image->SetRegions(region);
  image->Allocate();

  itk::ImageRegionIterator<ImageType> imageIterator(image,image->GetLargestPossibleRegion());

  // Make a square
  while(!imageIterator.IsAtEnd())
    {
    if(((imageIterator.GetIndex()[0] > 5 && imageIterator.GetIndex()[0] < 10) &&
      (imageIterator.GetIndex()[1] > 5 && imageIterator.GetIndex()[1] < 10)) ||
      ((imageIterator.GetIndex()[0] > 50 && imageIterator.GetIndex()[0] < 60) &&
      (imageIterator.GetIndex()[1] > 50 && imageIterator.GetIndex()[1] < 60)) ||
      ((imageIterator.GetIndex()[0] > 100 && imageIterator.GetIndex()[0] < 130) &&
      (imageIterator.GetIndex()[1] > 100 && imageIterator.GetIndex()[1] < 130))  )
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
  writer->SetFileName("input.png");
  writer->SetInput(image);
  writer->Update();
  
}
