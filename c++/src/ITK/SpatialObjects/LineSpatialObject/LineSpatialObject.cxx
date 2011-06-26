#include "itkSpatialObjectToImageFilter.h"
#include "itkLineSpatialObject.h"
#include "itkLineSpatialObjectPoint.h"
#include "itkImageFileWriter.h"

int main( int argc, char *argv[] )
{
  typedef unsigned char PixelType;
  const unsigned int Dimension = 2;

  typedef itk::Image< PixelType, Dimension >    ImageType;

  typedef itk::LineSpatialObject< Dimension >   LineType;

  typedef itk::SpatialObjectToImageFilter<
    LineType, ImageType >   SpatialObjectToImageFilterType;


  // Create a list of points
  std::vector<LineType::LinePointType> points;
  for(unsigned int i = 0; i < 20; i++)
    {
    LineType::LinePointType point;
    point.SetPosition(10,i);

    LineType::LinePointType::VectorType normal;
    normal[0] = 0;
    normal[1] = 1;
    point.SetNormal(normal,0);
    points.push_back(point);

    }

  // Create a line from the list of points
  LineType::Pointer line = LineType::New();
  line->SetPoints(points);


  // Test accessing
  LineType::SpatialObjectPointType* testPoint = line->GetPoint(0);
  std::cout << "(" << testPoint->GetPosition()[0] << ", " << testPoint->GetPosition()[1] << ")" << std::endl;

  //LineType::SpatialObjectPointType testPoint;
  //std::cout << testPoint.GetPosition()[0] << std::endl;

  // Convert the spatial object to an image
  SpatialObjectToImageFilterType::Pointer imageFilter =
    SpatialObjectToImageFilterType::New();
  itk::Size<2> size;
  size.Fill(50);
  imageFilter->SetInsideValue(255); // white
  imageFilter->SetSize(size);
  imageFilter->SetInput(line);
  imageFilter->Update();

  typedef itk::ImageFileWriter< ImageType >     WriterType;
  WriterType::Pointer writer = WriterType::New();
  writer->SetFileName("line.png");
  writer->SetInput( imageFilter->GetOutput() );
  writer->Update();

  return EXIT_SUCCESS;
}
