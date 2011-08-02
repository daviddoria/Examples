#include "itkImage.h"
#include "itkImageFileWriter.h"
#include "itkImageRegionIterator.h"
#include "itkBinaryImageToLabelMapFilter.h"
#include "itkObjectByObjectLabelMapFilter.h"
#include "itkLabelMapToLabelImageFilter.h"
#include "itkLabelImageToLabelMapFilter.h"
#include "itkScalarToRGBColormapImageFilter.h"
#include "itkConnectedComponentImageFilter.h"

typedef itk::Image<unsigned char, 2>  UnsignedCharImageType;

typedef itk::RGBPixel<unsigned char>   RGBPixelType;
typedef itk::Image<RGBPixelType, 2>    RGBImageType;

static void CreateImage(UnsignedCharImageType::Pointer image);

int main(int, char *[])
{
  UnsignedCharImageType::Pointer image = UnsignedCharImageType::New();
  CreateImage(image);

  typedef itk::LabelImageToLabelMapFilter<UnsignedCharImageType> LabelImageToLabelMapFilterType;
  LabelImageToLabelMapFilterType::Pointer labelImageToLabelMapFilter = LabelImageToLabelMapFilterType::New();
  labelImageToLabelMapFilter->SetInput(image);
  labelImageToLabelMapFilter->Update();
  
  typedef itk::ConnectedComponentImageFilter <UnsignedCharImageType, UnsignedCharImageType >
    ConnectedComponentImageFilterType;
  ConnectedComponentImageFilterType::Pointer connectedComponentImageFilter
    = ConnectedComponentImageFilterType::New();
  
  typedef itk::ObjectByObjectLabelMapFilter< LabelImageToLabelMapFilterType::OutputImageType > ObjectByObjectLabelMapFilterType;
  ObjectByObjectLabelMapFilterType::Pointer objectByObjectLabelMapFilter = ObjectByObjectLabelMapFilterType::New();
  objectByObjectLabelMapFilter->SetInput( labelImageToLabelMapFilter->GetOutput() );
  //objectByObjectLabelMapFilter->SetBinaryInternalOutput(true);
  objectByObjectLabelMapFilter->SetBinaryInternalOutput(false);
  objectByObjectLabelMapFilter->SetFilter(connectedComponentImageFilter);
  objectByObjectLabelMapFilter->Update();

  typedef itk::LabelMapToLabelImageFilter<LabelImageToLabelMapFilterType::OutputImageType, UnsignedCharImageType> LabelMapToLabelImageFilterType;
  LabelMapToLabelImageFilterType::Pointer labelMapToLabelImageFilter = LabelMapToLabelImageFilterType::New();
  labelMapToLabelImageFilter->SetInput(objectByObjectLabelMapFilter->GetOutput());
  labelMapToLabelImageFilter->Update();
  
  // Color each label/object a different color
  typedef itk::ScalarToRGBColormapImageFilter<UnsignedCharImageType, RGBImageType> RGBFilterType;
  RGBFilterType::Pointer colormapImageFilter = RGBFilterType::New();
  colormapImageFilter->SetInput(labelMapToLabelImageFilter->GetOutput());
  colormapImageFilter->SetColormap( RGBFilterType::Jet );
  colormapImageFilter->Update();
  
  typedef itk::ImageFileWriter< RGBImageType > WriterType;
  WriterType::Pointer writer = WriterType::New();
  writer->SetInput( colormapImageFilter->GetOutput() );
  writer->SetFileName("output.png");
  writer->Update();

  return EXIT_SUCCESS;
}

void CreateImage(UnsignedCharImageType::Pointer image)
{
  // Create a black image with three white squares
  itk::Index<2> start;
  start.Fill(0);

  itk::Size<2> size;
  size.Fill(200);

  itk::ImageRegion<2> region(start,size);
  image->SetRegions(region);
  image->Allocate();

  itk::ImageRegionIterator<UnsignedCharImageType> imageIterator(image,image->GetLargestPossibleRegion());

  // Make a region of two disjoint squares
  while(!imageIterator.IsAtEnd())
    {
    if(((imageIterator.GetIndex()[0] > 5 && imageIterator.GetIndex()[0] < 20) &&
      (imageIterator.GetIndex()[1] > 5 && imageIterator.GetIndex()[1] < 20)) ||
      ((imageIterator.GetIndex()[0] > 50 && imageIterator.GetIndex()[0] < 60) &&
      (imageIterator.GetIndex()[1] > 50 && imageIterator.GetIndex()[1] < 60)) )
      {
      imageIterator.Set(10);
      }
    else if(((imageIterator.GetIndex()[0] > 100 && imageIterator.GetIndex()[0] < 130) &&
    (imageIterator.GetIndex()[1] > 100 && imageIterator.GetIndex()[1] < 130)))
      {
      imageIterator.Set(20);
      }
    else
      {
      imageIterator.Set(0);
      }

    ++imageIterator;
    }

  // Color each label/object a different color
  typedef itk::ScalarToRGBColormapImageFilter<UnsignedCharImageType, RGBImageType> RGBFilterType;
  RGBFilterType::Pointer colormapImageFilter = RGBFilterType::New();
  colormapImageFilter->SetInput(image);
  colormapImageFilter->SetColormap( RGBFilterType::Jet );
  colormapImageFilter->Update();
  
  typedef  itk::ImageFileWriter< RGBImageType  > WriterType;
  WriterType::Pointer writer = WriterType::New();
  writer->SetFileName("input.png");
  writer->SetInput(colormapImageFilter->GetOutput());
  writer->Update();
  
}
